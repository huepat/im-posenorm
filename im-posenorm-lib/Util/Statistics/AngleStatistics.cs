namespace HuePat.IMPoseNorm.Util.Statistics {
    public class AngleStatistics {
        private const double FULL_CIRCLE = 2 * System.Math.PI;

        public long Counter { get; private set; }
        public double Mean { get; private set; }
        public double Variance { get; private set; }

        public double StandardDeviation {
            get {
                return Variance.Sqrt();
            }
        }

        public AngleStatistics() {

            Mean = 0f;
            Variance = 0f;
            Counter = 0;
        }

        public void Update(double value) {

            double oldMean = Mean;

            Mean = oldMean + AngleSubstract(value, oldMean) / (Counter + 1);

            if (Counter > 0) {
                Variance = (1 - 1 / Counter) * Variance + (Counter + 1) 
                    * AngleSubstract(Mean, oldMean).Squared();
            }

            Counter++;
        }

        private double AngleSubstract(
                double angle1, 
                double angle2) {

            double result1 = System.Math.Max(angle1, angle2) - System.Math.Min(angle1, angle2);
            double result2 = result1 - FULL_CIRCLE;

            if (result2.Abs() < result1.Abs()) {
                if (angle1 < angle2) {
                    angle1 = angle1 + FULL_CIRCLE;
                }
                else {
                    angle2 = angle2 + FULL_CIRCLE;
                }
            }

            return angle1 - angle2;
        }
    }
}