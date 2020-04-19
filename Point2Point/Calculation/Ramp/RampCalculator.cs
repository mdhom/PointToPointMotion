using System;

namespace Shuttles.Base.Devices.Shuttles.Motion.Ramp
{
    internal class RampCalculator
    {
        public double InitialAcceleration { get; }
        public double InitialVelocity { get; }
        public RampMotionParameter MotionParameter { get; }

        public bool Inverted { get; set; }

        public RampCalculator(double initialAcceleration, double initialVelocity, RampMotionParameter motionParameter)
        {
            InitialAcceleration = initialAcceleration;
            InitialVelocity = initialVelocity;
            MotionParameter = motionParameter;
        }

        public static RampCalculationResult Calculate(double initialAcceleration, double initialVelocity, RampMotionParameter motionParameter, double targetVelocity)
        {
            var calculator = new RampCalculator(initialAcceleration, initialVelocity, motionParameter);
            return calculator.Calculate(targetVelocity);
        }

        public RampCalculationResult Calculate(double targetVelocity)
        {
            var result = new RampCalculationResult();

            //1. Bestimmung ob Bremsen oder Beschleuningen
            result.MotionState = targetVelocity < InitialVelocity ? RampMotionState.Brake : RampMotionState.Accelerate;

            var decMax = MotionParameter.MaximumDecceleration;
            var jPos = MotionParameter.PositiveJerk;
            var jNeg = MotionParameter.NegativeJerk;
            if (result.MotionState == RampMotionState.Accelerate)
            {
                decMax = MotionParameter.MaximumAcceleration;
                jPos = MotionParameter.NegativeJerk;
                jNeg = MotionParameter.PositiveJerk;
            }

            var a0 = InitialAcceleration;
            var v0 = InitialVelocity;
            var t1 = (decMax - a0) / jNeg;
            var t2 = 0.0;
            var t3 = -(decMax / jPos);

            //2. Fahrprofil mit oder ohne flacher Phase
            var v_bya0_Ph1 = t1 * a0;
            var v_byjD_Ph1 = 0.5 * jNeg * t1 * t1;
            var v_bya1_Ph3 = t3 * (a0 + jNeg * t1);
            var v_byjA_Ph3 = 0.5 * jPos * t3 * t3;
            var v_total = v0 + v_bya0_Ph1 + v_byjD_Ph1 + v_bya1_Ph3 + v_byjA_Ph3;

            if (CheckForFlatRampPart(v_total, targetVelocity, result))                 // Flache Phase wird benötigt, um das gewüschte Ziel zu erreichen!
            {
                t1 = (decMax - a0) / jNeg;                                           // Zeit um aMaxDec zu erreichen
                t3 = Math.Abs(decMax / jPos);                                      // Zeit um aMaxDec wirder abzubauen 
                var v_Decc = 0.5 * jNeg * (t1 * t1) + a0 * t1;                 // v die in t1 erreicht wird
                var v_Acc = -0.5 * jPos * (t3 * t3);                           // v die in t3 erzeugt wird
                t2 = (targetVelocity - (v_Decc + v_Acc + v0)) / decMax;
            }
            else                                                                // Flache Phase wird nicht benötigt --> berechnen welche Beschleunigung erreicht werden darf!
            {
                double jerk = 0;
                double t = 0;
                if (a0 < 0)
                {
                    jerk = MotionParameter.PositiveJerk;
                    t = (0 - a0) / jerk;
                }
                else if (a0 > 0)
                {
                    jerk = MotionParameter.NegativeJerk;
                    t = (0 - a0) / jerk;
                }
                else if (a0 == 0)
                {
                    t = 0;
                    jerk = 0;
                }

                //3. Bestimmung ob trotz Bremsen, beschleunigt werden muss und umgekehrt!
                // Geschwindigkeit die erreicht werden würde (v_bya0_to0), falls a0 auf 0 gezogen wird. Dies ist das aussschlagebende Kriterium, ob beschleunigt oder gebremst werden muss
                // Bsp.: v0 =200, a0= -112 , vTarget = 190  Nur durch den Abbau von a0 auf 0 wird eine Geschwindigkeit von ca. 187 erreicht --> es muss mathematisch beschleunigt werden, um die Zieglgeschwindigkeit zu erreichen
                var v_bya0_to0 = v0 + t * a0 + 0.5 * jerk * t * t;

                if (v_bya0_to0 > targetVelocity)
                {
                    Inverted = result.MotionState == RampMotionState.Accelerate;
                }
                else
                {
                    Inverted = result.MotionState != RampMotionState.Accelerate;
                }

                if (Inverted)  // Beschleuningen falls wir bremsen ---> Bremsen falls beschleunigen
                {
                    result.Invert();

                    var tmp = jNeg;
                    jNeg = jPos;
                    jPos = tmp;
                }

                // 4. Gleichungssystem, um t1,t2 und t3 zu bestimmen
                var a = 0.5 * jNeg - 0.5 * (jNeg * jNeg / jPos);
                var b = a0 - a0 * (jNeg / jPos);
                var c = v0 - targetVelocity - a0 * a0 / (2 * jPos);

                if (QuadraticEquation(a, b, c, out var x1, out var x2))
                {
                    CalculateAllTimes(x1, x2, jPos, jNeg, a0, out t1, out t2, out t3);
                }
            }


            var a1 = a0 + jNeg * t1;
            var v1 = v0 + a0 * t1 + 0.5 * jNeg * t1 * t1;
            var s1 = v0 * t1 + 0.5 * a0 * t1 * t1 + 1.0 / 6.0 * jNeg * t1 * t1 * t1;

            var a2 = a1;
            var v2 = v1 + a1 * t2;
            var s2 = v1 * t2 + 0.5 * a1 * t2 * t2;

            var s3 = v2 * t3 + 0.5 * a2 * t3 * t3 + 1.0 / 6.0 * jPos * t3 * t3 * t3;

            result.TotalDistance = s1 + s2 + s3;
            result.TotalDuration = t1 + t2 + t3;
            result.t1 = t1;
            result.t2 = t2;
            result.t3 = t3;
            return result;
        }

        private static void CalculateAllTimes(double x1, double x2, double jA, double jD, double a0, out double t1, out double t2, out double t3)
        {
            t1 = 0;
            t2 = 0;
            t3 = 0;
            if (x1 >= 0 && x2 >= 0)
            {
                var t1_1 = x1;
                var t1_2 = x2;
                t2 = 0;
                var t3_1 = (-jD * t1_1 - a0) / jA;
                var t3_2 = (-jD * t1_2 - a0) / jA;
                t1 = t3_1 > 0 ? t1_1 : t1_2;
                t3 = t3_1 > 0 ? t3_1 : t3_2;
            }
            else if (x1 >= 0 || x2 >= 0)
            {
                t1 = x1 > 0 ? x1 : x2;
                t2 = 0;
                t3 = (-jD * t1 - a0) / jA;
            }
        }

        private static bool CheckForFlatRampPart(double v_total, double vTarget, RampCalculationResult result)
        {
            if (result.MotionState == RampMotionState.Brake)
            {
                return v_total > vTarget;
            }
            else
            {
                return v_total <= vTarget;
            }
        }

        public static bool QuadraticEquation(double a, double b, double c, out double x1, out double x2)
        {
            x1 = 0;
            x2 = 0;
            var root = Math.Sqrt(b * b - 4 * a * c);
            if (root >= 0 && a != 0)
            {
                x1 = (-b + root) / (2 * a);
                x2 = (-b - root) / (2 * a);
                return true;
            }

            return false;
        }
    }
}
