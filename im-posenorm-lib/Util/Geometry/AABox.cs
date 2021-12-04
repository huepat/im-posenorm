using OpenTK.Mathematics;
using System.Collections.Generic;

namespace HuePat.IMPoseNorm.Util.Geometry {
    public class AABox {
        public Vector3d Min { get; protected set; }
        public Vector3d Max { get; protected set; }

        public Vector3d Size {
            get {
                return Max - Min;
            }
        }

        public virtual Mesh Mesh {
            get {

                List<Point> vertices = new List<Point> {
                    new Point(Min),
                    new Point(new Vector3d(Min.X, Max.Y, Min.Z)),
                    new Point(new Vector3d(Min.X, Max.Y, Max.Z)),
                    new Point(new Vector3d(Min.X, Min.Y, Max.Z)),
                    new Point(new Vector3d(Max.X, Min.Y, Min.Z)),
                    new Point(new Vector3d(Max.X, Max.Y, Min.Z)),
                    new Point(Max),
                    new Point(new Vector3d(Max.X, Min.Y, Max.Z))
                };

                Mesh mesh = new Mesh(
                    vertices,
                    new List<Face> {
                        new Face(0, 3, 1, vertices),
                        new Face(1, 3, 2, vertices),
                        new Face(5, 2, 6, vertices),
                        new Face(1, 2, 5, vertices),
                        new Face(2, 3, 6, vertices),
                        new Face(3, 7, 6, vertices),
                        new Face(4, 6, 7, vertices),
                        new Face(4, 5, 6, vertices),
                        new Face(0, 7, 3, vertices),
                        new Face(0, 4, 7, vertices),
                        new Face(0, 1, 4, vertices),
                        new Face(1, 5, 4, vertices)
                    }
                );

                return mesh;
            }
        }

        public AABox(
                Vector3d min, 
                Vector3d max) {

            Min = min;
            Max = max;
        }
    }
}