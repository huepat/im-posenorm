using OpenTK.Mathematics;
using System.Collections.Generic;

namespace HuePat.IMPoseNorm.Util.Geometry {
    public class Face {

        public int VertexIndex1 { get; private set; }
        public int VertexIndex2 { get; private set; }
        public int VertexIndex3 { get; private set; }
        public Point Vertex1 { get; private set; }
        public Point Vertex2 { get; private set; }
        public Point Vertex3 { get; private set; }

        public double Area {
            get {
                double a = Vertex1.DistanceTo(Vertex2);
                double b = Vertex1.DistanceTo(Vertex3);
                double c = Vertex2.DistanceTo(Vertex3);
                double s = (a + b + c) / 2.0;
                return (s * (s - a) * (s - b) * (s - c)).Sqrt();
            }
        }

        public Vector3d Normal {
            get {
                return Vector3d.Normalize(
                    Vector3d.Cross(
                        Vertex2.Position - Vertex1.Position,
                        Vertex3.Position - Vertex1.Position));
            }
        }

        public Face(
                int vertexIndex1,
                int vertexIndex2,
                int vertexIndex3,
                IReadOnlyList<Point> vertices) {

            VertexIndex1 = vertexIndex1;
            VertexIndex2 = vertexIndex2;
            VertexIndex3 = vertexIndex3;

            Vertex1 = vertices[VertexIndex1];
            Vertex2 = vertices[VertexIndex2];
            Vertex3 = vertices[VertexIndex3];
        }

        public Face Clone(
                IReadOnlyList<Point> clonedVertices) {

            return new Face(
                VertexIndex1,
                VertexIndex2,
                VertexIndex3,
                clonedVertices);
        }
    }
}