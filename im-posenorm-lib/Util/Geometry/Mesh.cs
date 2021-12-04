using OpenTK.Mathematics;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.IMPoseNorm.Util.Geometry {
    public class Mesh: IReadOnlyList<Face>, IShape {

        public static Mesh Merge(IList<Mesh> meshes) {

            int offset = 0;
            List<Point> vertices;
            List<Face> faces = new List<Face>();

            vertices = meshes
                .SelectMany(mesh => mesh.Vertices)
                .ToList();

            foreach (Mesh mesh in meshes) {

                faces.AddRange(
                    mesh.Select(face => {
                        return new Face(
                            face.VertexIndex1 + offset,
                            face.VertexIndex2 + offset,
                            face.VertexIndex3 + offset,
                            vertices);
                    }));

                offset += mesh.Vertices.Count;
            }

            return new Mesh(
                vertices,
                faces);
        }

        private readonly List<Face> faces;

        public PointCloud Vertices { get; private set; }

        public int Count {
            get {
                return faces.Count;
            }
        }

        public ShapeType Type {
            get {
                return ShapeType.MESH;
            }
        }

        public AABox BBox {
            get {
                return Vertices.BBox;
            }
        }

        public IReadOnlyList<double> SizeWeights {
            get {
                return this
                    .Select(face => face.Area)
                    .ToList();
            }
        }

        public List<Vector3d> Normals {
            get {
                return this
                    .Select(face => face.Normal)
                    .ToList();
            }
        }

        public IReadOnlyList<Point> Points {
            get {
                return Vertices;
            }
        }

        Mesh IShape.Mesh {
            get {
                return this;
            }
        }

        public Face this[int i] {
            get {
                return faces[i];
            }
        }

        public Mesh(
                List<Point> vertices,
                List<Face> faces) {

            this.faces = faces;
            Vertices = new PointCloud(vertices);
        }

        public IEnumerator<Face> GetEnumerator() {

            return faces
                .AsEnumerable()
                .GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator() {
            return GetEnumerator();
        }

        public void UpdateBBox() {
            Vertices.UpdateBBox();
        }

        public IShape Clone() {

            List<Point> clonedVertices = Vertices
                .Select(vertex => vertex.Clone())
                .ToList();

            return new Mesh(
                clonedVertices,
                faces
                    .Select(face => face.Clone(clonedVertices))
                    .ToList());
        }
    }
}