using HuePat.IMPoseNorm.Util.Geometry;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.IMPoseNorm.Util {
    public static class Extensions {
        private const double EPSILON = 10E-5;

        public static double Abs(this double value) {
            return Math.Abs(value);
        }

        public static double Ceil(this double value) {
            return Math.Ceiling(value);
        }

        public static double Floor(this double value) {
            return Math.Floor(value);
        }

        public static double Sqrt(this double value) {
            return Math.Sqrt(value);
        }

        public static double Acos(this double value) {
            return Math.Acos(value);
        }
        public static double Cos(this double value) {
            return Math.Cos(value);
        }

        public static double Sin(this double value) {
            return Math.Sin(value);
        }

        public static double Squared(this double value) {
            return value * value;
        }

        public static double DegreeToRadian(this double degree) {
            return degree * Math.PI / 180.0;
        }

        public static double RadianToDegree(this double radian) {
            return radian * 180.0 / Math.PI;
        }

        public static bool ApproximateEquals(
                this double d1,
                double d2,
                double epsilon = EPSILON) {

            return (d2 - d1).Abs() < epsilon;
        }

        public static double AngleTo(
                this Vector3d v1,
                Vector3d v2) {

            return Vector3d.CalculateAngle(v1, v2);
        }

        public static double AngleTo(
                this Vector3d v1,
                Vector3d v2,
                out Vector3d axis) {

            axis = Vector3d
                .Cross(v1, v2)
                .Normalized();

            return Vector3d.Dot(
                    v1.Normalized(), 
                    v2.Normalized())
                .Acos();
        }

        public static double AngleTo(
                this Vector3d v1,
                Vector3d v2,
                Vector3d axis) {

            return Math.Atan2(
                Vector3d.Dot(
                    axis,
                    Vector3d.Cross(v1, v2)),
                Vector3d.Dot(v1, v2));
        }

        public static Matrix3d RotationTo(
                this Vector3d v1,
                Vector3d v2) {

            double angle = v1.AngleTo(
                v2, 
                out Vector3d axis);

            return axis.GetRotationAround(angle);
        }

        public static Matrix3d GetRotationAround(
                this Vector3d axis, 
                double angle) {

            double c = angle.Cos();
            double s = angle.Sin();
            double t = 1.0 - c;

            Vector3d axisNormalized = axis.Normalized();

            double tmp1 = axisNormalized.X * axisNormalized.Y * t;
            double tmp2 = axisNormalized.Z * s;
            double tmp5 = axisNormalized.Y * axisNormalized.Z * t;
            double tmp6 = axisNormalized.X * s;
            double tmp3 = axisNormalized.X * axisNormalized.Z * t;
            double tmp4 = axisNormalized.Y * s;

            return new Matrix3d(
                c + axisNormalized.X * axisNormalized.X * t, 
                    tmp1 - tmp2, 
                    tmp3 + tmp4,
                tmp1 + tmp2, 
                    c + axisNormalized.Y * axisNormalized.Y * t, 
                    tmp5 - tmp6,
                tmp3 - tmp4, 
                    tmp5 + tmp6, 
                    c + axisNormalized.Z * axisNormalized.Z * t);
        }

        public static Vector3d OrthogonalProject(
                this Vector3d vector,
                Vector3d axis) {

            return vector - Vector3d.Dot(vector, axis) * axis;
        }

        public static Vector3d Multiply(
                this Matrix3d matrix,
                Vector3d vector) {

            return new Vector3d(
                matrix[0, 0] * vector.X + matrix[0, 1] * vector.Y + matrix[0, 2] * vector.Z,
                matrix[1, 0] * vector.X + matrix[1, 1] * vector.Y + matrix[1, 2] * vector.Z,
                matrix[2, 0] * vector.X + matrix[2, 1] * vector.Y + matrix[2, 2] * vector.Z);
        }

        public static Vector3d GetCentroid(
                this IShape shape) {

            return shape
                    .Points
                    .AsParallel()
                    .Select(point => point.Position)
                    .Aggregate((position1, position2) => position1 + position2)
                / shape.Points.Count;
        }

        public static Vector3d WeightedMean(
                this IEnumerable<(Vector3d, double)> valueWeightPairs) {

            double totalWeigth = 0.0;
            Vector3d sum = new Vector3d();

            foreach ((Vector3d, double) valueWeightPair in valueWeightPairs) {
                sum += valueWeightPair.Item1 * valueWeightPair.Item2;
                totalWeigth += valueWeightPair.Item2;
            }

            return sum / totalWeigth;
        }

        public static double WeightedMean(
                this IEnumerable<(double, double)> valueWeightPairs) {

            double sum = 0.0;
            double totalWeigth = 0.0;

            foreach ((double, double) valueWeightPair in valueWeightPairs) {
                sum += valueWeightPair.Item1 * valueWeightPair.Item2;
                totalWeigth += valueWeightPair.Item2;
            }

            return sum / totalWeigth;
        }

        public static Vector3d WeightedMedian(
                this IEnumerable<(Vector3d, double)> valueWeightPairs) {

            return new Vector3d(
                valueWeightPairs
                    .Select(valueWeightPair => (
                        valueWeightPair.Item1.X,
                        valueWeightPair.Item2
                    ))
                    .WeightedMedian(),
                valueWeightPairs
                    .Select(valueWeightPair => (
                        valueWeightPair.Item1.Y,
                        valueWeightPair.Item2
                    ))
                    .WeightedMedian(),
                valueWeightPairs
                    .Select(valueWeightPair => (
                        valueWeightPair.Item1.Z,
                        valueWeightPair.Item2
                    ))
                    .WeightedMedian());
        }

        public static double WeightedMedian(
                this IEnumerable<(double, double)> valueWeightPairs) {

            double weightSum = 0.0;
            double totalweightSum;
            List<(double, double)> sorted = valueWeightPairs
                .OrderBy(valueWeightPair => valueWeightPair.Item1)
                .ToList();

            totalweightSum = sorted.Sum(valueWeightPair => valueWeightPair.Item2);

            foreach ((double, double) valueWeightPair in sorted) {

                weightSum += valueWeightPair.Item2;

                if (weightSum / totalweightSum >= 0.5) {
                    return valueWeightPair.Item1;
                }
            }

            throw new ApplicationException();
        }

        public static void BucketAdd<TKey, TValue>(
                this Dictionary<TKey, List<TValue>> dictionary,
                TKey key,
                TValue value) {

            if (!dictionary.ContainsKey(key)) {
                dictionary.Add(key, new List<TValue>());
            }

            dictionary[key].Add(value);
        }

        public static void BucketAdd<TKey, TValue>(
                this Dictionary<TKey, List<TValue>> dictionary,
                TKey key,
                IEnumerable<TValue> values) {

            if (!dictionary.ContainsKey(key)) {
                dictionary.Add(key, new List<TValue>());
            }

            dictionary[key].AddRange(values);
        }

        public static void BucketAdd<TKey, TValue>(
                this Dictionary<TKey, List<TValue>> dictionary1,
                Dictionary<TKey, List<TValue>> dictionary2) {

            foreach (TKey key in dictionary2.Keys) {

                if (!dictionary1.ContainsKey(key)) {
                    dictionary1.Add(key, new List<TValue>());
                }

                dictionary1[key].AddRange(dictionary2[key]);
            }
        }

        public static List<T> WhereMax<T, U>(
                this IEnumerable<T> objects,
                Func<T, U> valueExtractor) where U : IComparable {

            if (!objects.Any()) {
                return new List<T>();
            }

            bool first = true;
            int testResult;
            List<T> resultObjects = new List<T> { objects.First() };
            U candidateValue = valueExtractor(resultObjects[0]);
            U testValue;

            foreach (T @object in objects) {

                testValue = valueExtractor(@object);
                testResult = testValue.CompareTo(candidateValue);

                if (testResult > 0) {
                    candidateValue = testValue;
                    resultObjects = new List<T> { @object };
                }
                else if (!first && testResult == 0) {
                    resultObjects.Add(@object);
                }

                first = false;
            }

            return resultObjects;
        }

        public static string FormatMilliseconds(this long milliseconds) {
            return ((double)milliseconds).FormatMilliseconds();
        }

        public static string FormatMilliseconds(this double milliseconds) {

            int days, hours, minutes, seconds;
            string s = "";

            days = (int)((double)milliseconds / (24 * 60 * 60 * 1000)).Floor();
            if (days > 0) {
                milliseconds -= days * 24 * 60 * 60 * 1000;
            }

            hours = (int)((double)milliseconds / (60 * 60 * 1000)).Floor();
            if (hours > 0) {
                milliseconds -= hours * 60 * 60 * 1000;
            }

            minutes = (int)((double)milliseconds / (60 * 1000)).Floor();
            if (minutes > 0) {
                milliseconds -= minutes * 60 * 1000;
            }

            seconds = (int)((double)milliseconds / 1000).Floor();
            if (seconds > 0) {
                milliseconds -= seconds * 1000;
            }

            if (days > 0) {
                s += $"{days}d/";
            }
            if (hours > 0) {
                s += $"{hours}h/";
            }
            if (minutes > 0) {
                s += $"{minutes}m/";
            }
            if (seconds > 0) {
                s += $"{seconds}s/";
            }

            s += $"{milliseconds:0}ms";

            return s;
        }
    }
}