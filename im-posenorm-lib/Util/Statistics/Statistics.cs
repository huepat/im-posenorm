namespace HuePat.IMPoseNorm.Util.Statistics {
    public class Statistics {
        public long Counter { get; private set; }
        public double Mean { get; private set; }
        public double Variance { get; private set; }

        public double StandardDeviation {
            get {
                return Variance.Sqrt();
            }
        }

        public Statistics() {

            Mean = 0.0;
            Variance = 0.0;
            Counter = 0;
        }

        public void Update(double value) {

            double oldMean = Mean;

            Mean = oldMean + (value - oldMean) / (Counter + 1);

            if (Counter > 0) {
                Variance = (1.0 - 1.0 / Counter) * Variance + (Counter + 1) * (Mean - oldMean).Squared();
            }

            Counter++;
        }
    }
}