using OpenTK.Mathematics;

namespace HuePat.IMPoseNorm.Util.Geometry {
    public class Point {
        private Vector3d position;

        public Vector3d Position {
            get { return position; }
            set { position = value; }
        }

        public double X {
            get { return position.X; }
            set { position.X = value; }
        }

        public double Y {
            get { return position.Y; }
            set { position.Y = value; }
        }

        public double Z {
            get { return position.Z; }
            set { position.Z = value; }
        }

        public Point(Vector3d position) {
            this.position = position;
        }

        public Point Clone() {
            return new Point(position);
        }

        public double DistanceTo(Point otherPoint) {
            return (otherPoint.Position - position).Length;
        }
    }
}