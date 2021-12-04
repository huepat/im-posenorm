using HuePat.IMPoseNorm.Util;
using HuePat.IMPoseNorm.Util.Geometry;
using HuePat.IMPoseNorm.Util.IO;
using HuePat.IMPoseNorm.Util.IO.Reading;
using HuePat.IMPoseNorm.Util.IO.Writing;
using HuePat.IMPoseNorm.Util.Statistics;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;

namespace HuePat.IMPoseNorm.Eval {
    public static class Evaluation {
        public enum Type {
            MESH,
            POINT_CLOUD
        };

        public class Config {
            private int sampleCount;
            private IList<Matrix3d> rotations;

            public bool OutputPly { get; set; }
            public bool NormalizeAlignment { get; set; }
            public Type Type { get; private set; }
            public PLYEncoding OutputPlyEncoding { get; set; }
            public int LineBreakByteSize { get; set; }
            public double VerticalAngleRange { private get; set; }
            public string File { get; private set; }
            public string OutputDirectory { get; set; }
            public Vector3d UpAxis { get; private set; }
            public Vector3d HorizontalAxis { get; private set; }
            public Vector3d HorizontalAxis2 { get; private set; }
            public (string, string, string) CoordinateIndentifiers { get; set; }
            public (string, string, string) NormalVectorIndentifiers { get; set; }

            public int SampleCount { 
                get {
                    return sampleCount;
                }
                set {
                    sampleCount = value;
                    rotations = null;
                }
            }

            public IList<Matrix3d> Rotations {
                get {

                    if (rotations != null) {
                        return rotations;
                    }

                    Angles = Enumerable
                        .Range(
                            0, 
                            SampleCount)
                        .Select(i => (
                            Random.GetDouble(
                                0.0, 
                                360.0.DegreeToRadian()),
                            Random.GetDouble(
                                -VerticalAngleRange, 
                                VerticalAngleRange),
                            Random.GetDouble(
                                -VerticalAngleRange, 
                                VerticalAngleRange)
                        ))
                        .ToList();

                    return rotations;
                }
                set {
                    sampleCount = value.Count;
                    rotations = value;
                }
            }

            public IList<(double, double, double)> Angles {
                set {

                    using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                        Log("");
                        Log("INPUT ANGLES:");
                        for (int i = 0; i < value.Count(); i++) {
                            Log($"    {i}: (" +
                                $"{value[i].Item1.RadianToDegree():0.00}°, " +
                                $"{value[i].Item2.RadianToDegree():0.00}°, " +
                                $"{value[i].Item3.RadianToDegree():0.00}°)");
                        }
                        Log("");
                    }

                    sampleCount = value.Count;
                    rotations = value
                        .Select(angleSet => HorizontalAxis2.GetRotationAround(angleSet.Item3)
                            * HorizontalAxis.GetRotationAround(angleSet.Item2)
                            * UpAxis.GetRotationAround(angleSet.Item1))
                        .ToList();
                }
            }

            public Config(
                    double verticalAngleRange,
                    string file,
                    Type type,
                    Vector3d upAxis,
                    Vector3d horizontalAxis) {

                VerticalAngleRange = verticalAngleRange;
                SampleCount = 10;
                LineBreakByteSize = 2;
                File = file;
                Type = type;
                UpAxis = upAxis;
                OutputDirectory = ".";
                OutputPlyEncoding = PLYEncoding.BINARY_LITTLE_ENDIAN;
                HorizontalAxis = horizontalAxis;
                HorizontalAxis2 = Vector3d.Cross(UpAxis, HorizontalAxis);
                CoordinateIndentifiers = ("x", "y", "z");
                NormalVectorIndentifiers = ("nx", "ny", "nz");

                using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                    Log("");
                    Log($"UP AXIS: " +
                        $"({UpAxis.X}, " +
                        $"{UpAxis.Y}, " +
                        $"{UpAxis.Z})");
                    Log($"HORIZONTAL AXIS 1: (" +
                        $"{HorizontalAxis.X}, " +
                        $"{HorizontalAxis.Y}, " +
                        $"{HorizontalAxis.Z})");
                    Log($"HORIZONTAL AXIS 2: (" +
                        $"{HorizontalAxis2.X}, " +
                        $"{HorizontalAxis2.Y}, " +
                        $"{HorizontalAxis2.Z})");
                    Log("");
                }
            }
        }

        public static void Evaluate(Config config) {

            if (!Directory.Exists(config.OutputDirectory)) {
                Directory.CreateDirectory(config.OutputDirectory);
            }

            int i = 0;
            long startTimestamp, time;
            (double, double) referenceAngles;
            (double, double) angleDeviations;
            IShape clone;
            IShape model = Load(config);
            Timer timer = Timer.Instance;
            Matrix3d rotation;
            Statistics timeStatistics = new Statistics();
            (AngleStatistics, AngleStatistics) angleDeviationStatistics = (
                new AngleStatistics(),
                new AngleStatistics()
            );
            IList<Matrix3d> rotations = config.Rotations;

            Log("EVALUATION:");

            if (config.OutputPly) {
                model.Export(
                    $"{config.OutputDirectory}/GroundTruth.ply",
                    config);
            }

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                foreach (Matrix3d referenceRotation in rotations) {

                    clone = model.Clone();
                    clone.Rotate(
                        new Rotation.Config(referenceRotation) {
                            RotateNormals = config.Type == Type.POINT_CLOUD
                        });

                    if (config.OutputPly) {
                        clone.Export(
                            $"{config.OutputDirectory}/In_{i}.ply",
                            config);
                    }

                    startTimestamp = timer.Timestamp;

                    clone.NormalizePose(
                        config.UpAxis,
                        config.HorizontalAxis,
                        config.NormalizeAlignment,
                        out rotation);

                    time = timer.Timestamp - startTimestamp;
                    timeStatistics.Update(time);

                    if (config.OutputPly) {
                        clone.Export(
                            $"{config.OutputDirectory}/Out_{i}.ply",
                            config);
                    }

                    referenceAngles = (
                        config.HorizontalAxis
                            .AngleTo(
                                referenceRotation.Multiply(config.HorizontalAxis))
                            .Abs(),
                        config.UpAxis
                            .AngleTo(
                                referenceRotation.Multiply(config.UpAxis))
                            .Abs()
                    );

                    angleDeviations = (
                        config.HorizontalAxis
                            .AngleTo(
                                rotation.Multiply(
                                    referenceRotation.Multiply(config.HorizontalAxis)))
                            .Abs(),
                        config.UpAxis
                            .AngleTo(
                                rotation.Multiply(
                                    referenceRotation.Multiply(config.UpAxis)))
                            .Abs()
                    );

                    while (angleDeviations.Item1 >= 45.0.DegreeToRadian()) {
                        angleDeviations.Item1 -= 90.0.DegreeToRadian();
                    }

                    angleDeviations.Item1 = angleDeviations.Item1.Abs();
                    angleDeviationStatistics.Item1.Update(angleDeviations.Item1);
                    angleDeviationStatistics.Item2.Update(angleDeviations.Item2);

                    Log($"{i++}: " +
                        $"({referenceAngles.Item1.RadianToDegree():0.00}°, " +
                            $"{referenceAngles.Item2.RadianToDegree():0.00}°) -> " +
                        $"({angleDeviations.Item1.RadianToDegree():0.00}°, " +
                            $"{angleDeviations.Item2.RadianToDegree():0.00}°) [{time.FormatMilliseconds()}]");
                }

                Log("___________________________________________________________");
                Log(
                    $"({angleDeviationStatistics.Item1.Mean.RadianToDegree():0.00}° " +
                        $"(+-{angleDeviationStatistics.Item1.StandardDeviation.RadianToDegree():0.00}°), " +
                    $"{angleDeviationStatistics.Item2.Mean.RadianToDegree():0.00}° " +
                        $"(+-{angleDeviationStatistics.Item2.StandardDeviation.RadianToDegree():0.00}°)) " +
                    $"[{timeStatistics.Mean.FormatMilliseconds()} +- {timeStatistics.StandardDeviation.FormatMilliseconds()}]");
            }
        }

        private static IShape Load(Config config) {

            PLYReader reader = new PLYReader() {
                LineBreakByteSize = config.LineBreakByteSize,
                CoordinateIndentifiers = config.CoordinateIndentifiers,
                NormalVectorIndentifiers = config.NormalVectorIndentifiers
            };

            switch (config.Type) {
                case Type.MESH:
                    return reader.ReadMesh(config.File);
                case Type.POINT_CLOUD:
                    return reader.ReadPointCloud(config.File);
                default:
                    throw new ArgumentException();
            }
        }

        private static void Export(
                this IShape model,
                string file,
                Config config) {

            PLYWriter writer = new PLYWriter() { 
                Encoding = config.OutputPlyEncoding
            };

            switch (model.Type) {
                case ShapeType.POINT_CLOUD:
                    writer.Write(
                        file, 
                        model as PointCloud);
                    break;
                case ShapeType.MESH:
                    writer.Write(
                        file, 
                        model as Mesh);
                    break;
            }
        }

        private static void Log(string line) {

            Trace.WriteLine(line);
            Console.WriteLine(line);
        }
    }
}