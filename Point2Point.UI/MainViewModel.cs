using System;
using System.Collections.Generic;
using System.ComponentModel;
using OxyPlot;
using Point2Point.Ramp;

namespace Point2Point.UI
{
    public class MainViewModel : INotifyPropertyChanged
    {
        private double _targetDistance = 15000;
        public double TargetDistance
        {
            get => _targetDistance;
            set
            {
                _targetDistance = value;
                Update();
            }
        }

        private double _velocityMax = 1500;
        public double VelocityMax
        {
            get => _velocityMax;
            set
            {
                _velocityMax = value;
                Update();
            }
        }

        private double _jerkMax = 2000;
        public double JerkMax
        {
            get => _jerkMax;
            set
            {
                _jerkMax = value;
                Update();
            }
        }

        private double _accelerationMax = 500;
        public double AccelerationMax
        {
            get => _accelerationMax;
            set
            {
                _accelerationMax = value;
                Update();
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

        public event PropertyChangedEventHandler PropertyChanged;

        public MainViewModel()
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
                var calc = new P2PCalculator(TargetDistance, JerkMax, AccelerationMax, VelocityMax);

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

            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(DataJ)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(DataA)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(DataV)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(DataS)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(DataBrakingDistance)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ResultDuration)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ResultTrajectoryInstanceCase)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ResultDistance)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ResultMaxReachedVelocity)));
        }
    }
}
