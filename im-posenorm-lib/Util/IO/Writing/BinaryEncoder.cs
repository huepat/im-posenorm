using HuePat.IMPoseNorm.Util.Geometry;
using OpenTK.Mathematics;
using System.IO;
using System.Text;

namespace HuePat.IMPoseNorm.Util.IO.Writing {
    class BinaryEncoder : IEncoder {

        private BinaryWriter writer;

        public BinaryEncoder(
                string file,
                bool littleEndian) {

            writer = new BinaryWriter(
                File.Open(
                    file,
                    FileMode.Append,
                    FileAccess.Write),
                littleEndian ?
                    Encoding.Unicode :
                    Encoding.BigEndianUnicode);
        }

        public void Dispose() {
            writer.Dispose();
        }

        public void Encode(Vector3d vector) {
            
        }

        public void Encode(Point point) {

            writer.Write((float)point.X);
            writer.Write((float)point.Y);
            writer.Write((float)point.Z);
        }

        public void Encode(
                Point point, 
                Vector3d normal) {

            Encode(point);
            writer.Write((float)normal.X);
            writer.Write((float)normal.Y);
            writer.Write((float)normal.Z);
        }

        public void Encode(
                Face face, 
                int offset) {

            writer.Write((byte)3);
            writer.Write(face.VertexIndex1 + offset);
            writer.Write(face.VertexIndex2 + offset);
            writer.Write(face.VertexIndex3 + offset);
        }
    }
}