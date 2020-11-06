using System;
using Point2Point.Mathematics.SimpleP2P;

namespace Point2Point.Mathematics.ExtendedP2P
{
    public struct MotionParameter
    {
        private double _positiveJerk;
        public double PositiveJerk
        {
            get => _positiveJerk; 
            set => _positiveJerk = value > 0 ? value : throw new ArgumentOutOfRangeException($"PositiveJerk must be > 0");
        }

        private double _negativeJerk;
        public double NegativeJerk
        {
            get => _negativeJerk;
            set => _negativeJerk = value < 0 ? value : throw new ArgumentOutOfRangeException($"NegativeJerk must be < 0");
        }

        private double _maximumAcceleration;
        public double MaximumAcceleration
        {
            get => _maximumAcceleration;
            set => _maximumAcceleration = value > 0 ? value : throw new ArgumentOutOfRangeException($"MaximumAcceleration must be > 0");
        }

        private double _maximumDecceleration; 
        public double MaximumDecceleration
        {
            get => _maximumDecceleration;
            set => _maximumDecceleration = value < 0 ? value : throw new ArgumentOutOfRangeException($"MaximumDecceleration must be < 0");
        }
        
        public MotionParameter(double positiveJerk, double negativeJerk, double maximumAcceleration, double maximumDecceleration)
        {
            _positiveJerk = positiveJerk;
            _negativeJerk = negativeJerk;
            _maximumAcceleration = maximumAcceleration;
            _maximumDecceleration = maximumDecceleration;
        }

        public MotionParameter(SimpleP2PParameters parameters)
        {
            _positiveJerk = parameters.JerkMax;
            _negativeJerk = -parameters.JerkMax;
            _maximumAcceleration = parameters.AccelerationMax;
            _maximumDecceleration = -parameters.AccelerationMax;
        }
    }
}
