using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using OpenTK.Mathematics;

namespace HuePat.IMPoseNorm.Util.Geometry {
    public class PointCloud : IReadOnlyList<Point>, IShape {

        public AABox BBox { get; private set; }
        public List<Vector3d> Normals { get; private set; }
        public IReadOnlyList<Point> Points { get; private set; }

        public ShapeType Type {
            get {
                return ShapeType.POINT_CLOUD;
            }
        }

        public int Count {
            get {
                return Points.Count;
            }
        }

        Mesh IShape.Mesh {
            get {
                return new Mesh(
                    this.ToList(),
                    new List<Face>());
            }
        }

        public IReadOnlyList<double> SizeWeights {
            get {
                return Enumerable
                    .Range(0, Count)
                    .Select(i => 1.0)
                    .ToList();
            }      
        }

        public Point this[int i] {
            get {
                return Points[i];
            }
        }

        public PointCloud(List<Point> points) : 
            this(
                points,
                new List<Vector3d>()) {
        }

        public PointCloud(
                List<Point> points,
                List<Vector3d> normals) {

            Points = points;
            Normals = normals;
        }

        public IEnumerator<Point> GetEnumerator() {

            return Points
                .AsEnumerable()
                .GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator() {
            return GetEnumerator();
        }

        public void UpdateBBox() {

            object @lock = new object();
            (Vector3d, Vector3d) bounds = (
                new Vector3d(double.MaxValue),
                new Vector3d(double.MinValue));

            Parallel.ForEach(
                Partitioner.Create(0, Points.Count),
                () => (
                    new Vector3d(double.MaxValue),
                    new Vector3d(double.MinValue)),
                (partition, loopState, localBounds) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {
                        if (Points[i].X < bounds.Item1.X) {
                            bounds.Item1.X = Points[i].X;
                        }
                        if (Points[i].Y < bounds.Item1.Y) {
                            bounds.Item1.Y = Points[i].Y;
                        }
                        if (Points[i].Z < bounds.Item1.Z) {
                            bounds.Item1.Z = Points[i].Z;
                        }
                        if (Points[i].X > bounds.Item2.X) {
                            bounds.Item2.X = Points[i].X;
                        }
                        if (Points[i].Y > bounds.Item2.Y) {
                            bounds.Item2.Y = Points[i].Y;
                        }
                        if (Points[i].Z > bounds.Item2.Z) {
                            bounds.Item2.Z = Points[i].Z;
                        }
                    }

                    return localBounds;

                },
                localBounds => {

                    lock (@lock) {
                        if (localBounds.Item1.X < bounds.Item1.X) {
                            bounds.Item1.X = localBounds.Item1.X;
                        }
                        if (localBounds.Item1.Y < bounds.Item1.Y) {
                            bounds.Item1.Y = localBounds.Item1.Y;
                        }
                        if (localBounds.Item1.Z < bounds.Item1.Z) {
                            bounds.Item1.Z = localBounds.Item1.Z;
                        }
                        if (localBounds.Item2.X > bounds.Item2.X) {
                            bounds.Item2.X = localBounds.Item2.X;
                        }
                        if (localBounds.Item2.Y > bounds.Item2.Y) {
                            bounds.Item2.Y = localBounds.Item2.Y;
                        }
                        if (localBounds.Item2.Z > bounds.Item2.Z) {
                            bounds.Item2.Z = localBounds.Item2.Z;
                        }
                    }

                });

            BBox = new AABox(
                bounds.Item1,
                bounds.Item2);
        }

        IShape IShape.Clone() {

            return new PointCloud(
                Points
                    .Select(point => point.Clone())
                    .ToList(),
                Normals);
        }
    }
}