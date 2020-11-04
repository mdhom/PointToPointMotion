using System;
using System.Collections.Generic;
using OxyPlot;
using Point2Point.Mathematics.SimpleP2P;
using Servus.Core;

namespace Point2Point.UI
{
    public class P2PCalculatorViewModel : NotifyPropertyChangedBase
    {
        private double _targetDistance = 15000;
        public double TargetDistance
        {
            get => _targetDistance;
            set
            {
                ChangeProperty(value, ref _targetDistance, Update);
            }
        }

        private double _velocityMax = 1500;
        public double VelocityMax
        {
            get => _velocityMax;
            set
            {
                ChangeProperty(value, ref _velocityMax, Update);
            }
        }

        private double _jerkMax = 2000;
        public double JerkMax
        {
            get => _jerkMax;
            set
            {
                ChangeProperty(value, ref _jerkMax, Update);
            }
        }

        private double _accelerationMax = 500;
        public double AccelerationMax
        {
            get => _accelerationMax;
            set
            {
                ChangeProperty(value, ref _accelerationMax, Update);
            }
        }

        public double ResultDuration { get; private set; }
        public double ResultDistance { get; private set; }
        public double ResultMaxReachedVelocity { get; private set; }
        public int ResultTrajectoryInstanceCase { get; private set; }

        public List<DataPoint> DataJ { get; private set; }
        public List<DataPoint> DataA { get; private set; }
        public List<DataPoint> DataV { get; private set; }
        public List<DataPoint> DataS { get; private set; }
        public List<DataPoint> DataBrakingDistance { get; private set; }

        public P2PCalculatorViewModel()
        {
            Update();
        }

        private void Update()
        {
            DataJ = new List<DataPoint>();
            DataA = new List<DataPoint>();
            DataV = new List<DataPoint>();
            DataS = new List<DataPoint>();
            DataBrakingDistance = new List<DataPoint>();
            ResultDuration = 0;
            ResultTrajectoryInstanceCase = 0;
            ResultDistance = 0;
            ResultMaxReachedVelocity = 0;

            try
            {
                var calc = new SimpleP2PCalculator(TargetDistance, JerkMax, AccelerationMax, VelocityMax);

                for (double t = 0; t <= calc.t7; t += 0.001)
                {
                    calc.GetStatus(t, out var j, out var a, out var v, out var s);
                    DataJ.Add(new DataPoint(t, j));
                    DataA.Add(new DataPoint(t, a));
                    DataV.Add(new DataPoint(t, v));
                    DataS.Add(new DataPoint(t, s));
                    DataBrakingDistance.Add(new DataPoint(t, calc.GetBrakingDistance(t)));
                }

                ResultDuration = calc.t7;
                ResultTrajectoryInstanceCase = calc.TrajectoryInstanceCase;
                ResultDistance = calc.GetTotalDistance();
                ResultMaxReachedVelocity = calc.CalculateMaximumReachedVelocity();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Exception: {ex.Message}\r\n{ex.StackTrace}");
            }

            OnPropertyChanged(nameof(DataJ));
            OnPropertyChanged(nameof(DataA));
            OnPropertyChanged(nameof(DataV));
            OnPropertyChanged(nameof(DataS));
            OnPropertyChanged(nameof(DataBrakingDistance));
            OnPropertyChanged(nameof(ResultDuration));
            OnPropertyChanged(nameof(ResultTrajectoryInstanceCase));
            OnPropertyChanged(nameof(ResultDistance));
            OnPropertyChanged(nameof(ResultMaxReachedVelocity));
        }
    }
}
