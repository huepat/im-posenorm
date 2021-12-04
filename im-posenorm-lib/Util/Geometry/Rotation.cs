using OpenTK.Mathematics;
using System.Collections.Concurrent;
using System.Threading.Tasks;

namespace HuePat.IMPoseNorm.Util.Geometry {
    public static class Rotation {
        public class Config {
            public bool UpdateBBox { get; set; }
            public bool RotateNormals { get; set; }
            public Vector3d? Anchor { get; set; }
            public Matrix3d Matrix { get; private set; }

            public Config(Matrix3d matrix) {
                Matrix = matrix;
            }

            public Config(
                    double angle,
                    Vector3d axis) :
                        this(axis.GetRotationAround(angle)) {
            }
        }

        public static IShape Rotate(
                this IShape shape,
                Config config) {

            Vector3d anchor = config.Anchor.HasValue ?
                config.Anchor.Value :
                shape.GetCentroid();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    shape.Points.Count),
                (partition, loopState) => {

                    for (int j = partition.Item1; j < partition.Item2; j++) {

                        shape.Points[j].Position = config.Matrix.Multiply(shape.Points[j].Position - anchor) + anchor;

                        if (config.RotateNormals) {
                            shape.Normals[j] = config.Matrix.Multiply(shape.Normals[j]);
                        }
                    }

                });

            if (config.UpdateBBox) {
                shape.UpdateBBox();
            }

            return shape;
        }
    }
}