using System;

namespace HuePat.IMPoseNorm.Eval {
    static class Random {
        private static readonly object LOCK = new object();

        [ThreadStatic]
        private static System.Random generator;
        private static System.Random globalGenerator = new System.Random(DateTime.Now.Millisecond);

        public static double GetDouble(
                double min, 
                double max) {

            return min + GetGenerator().NextDouble() * (max - min);
        }

        private static System.Random GetGenerator() {

            if (generator == null) {
                lock (LOCK) {
                    generator = new System.Random(
                        globalGenerator.Next());
                }
            }

            return generator;
        }
    }
}