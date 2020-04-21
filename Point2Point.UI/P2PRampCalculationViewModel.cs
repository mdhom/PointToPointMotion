using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OxyPlot;

namespace Point2Point.UI
{
    public class P2PRampCalculationViewModel : NotifyPropertyChangedBase
    {
        public List<DataPoint> DataJ { get; private set; }
        public List<DataPoint> DataA { get; private set; }
        public List<DataPoint> DataV { get; private set; }
        public List<DataPoint> DataS { get; private set; }

        public P2PRampCalculationViewModel()
        {
            Update();
        }

        private void Update()
        {
            DataJ = new List<DataPoint>();
            DataA = new List<DataPoint>();
            DataV = new List<DataPoint>();
            DataS = new List<DataPoint>();

            try
            {
                var d1 = 0.128452323; // 0.1130835;
                var d2 = 0.0;
                var v0 = 232;
                var jMax = -2000;

                var tMax = 2 * d1 + d2;
                for (double t = 0; t <= tMax; t += 0.001)
                {
                    P2PRampCalculations.GetStatus(t, v0, d1, d2, jMax, out var j, out var a, out var v, out var s);
                    DataJ.Add(new DataPoint(t, j));
                    DataA.Add(new DataPoint(t, a));
                    DataV.Add(new DataPoint(t, v));
                    DataS.Add(new DataPoint(t, s));
                }
                P2PRampCalculations.GetStatus(tMax, v0, d1, d2, jMax, out var jEnd, out var aEnd, out var vEnd, out var sEnd);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Exception: {ex.Message}\r\n{ex.StackTrace}");
            }

            OnPropertyChanged(nameof(DataJ));
            OnPropertyChanged(nameof(DataA));
            OnPropertyChanged(nameof(DataV));
            OnPropertyChanged(nameof(DataS));
        }
    }
}
