using HuePat.IMPoseNorm.Util.Geometry;
using OpenTK.Mathematics;
using System.IO;

namespace HuePat.IMPoseNorm.Util.IO.Writing {
    class AsciiEncoder : IEncoder {
        private StreamWriter writer;

        public AsciiEncoder(string file) {

            writer = new StreamWriter(file, true);
        }

        public void Dispose() {
            writer.Dispose();
        }

        public void Encode(Point point) {

            writer.WriteLine(
                $"{(float)point.X} {(float)point.Y} {(float)point.Z}");
        }

        public void Encode(
                Point point, 
                Vector3d normal) {

            writer.WriteLine(
                $"{(float)point.X} {(float)point.Y} {(float)point.Z} " +
                    $"{(float)normal.X} {(float)normal.Y} {(float)normal.Z}");
        }

        public void Encode(
                Face face,
                int offset) {

            writer.WriteLine(
                $"3 {face.VertexIndex1 + offset} " +
                $"{face.VertexIndex2 + offset} " +
                $"{face.VertexIndex3 + offset}");
        }
    }
}