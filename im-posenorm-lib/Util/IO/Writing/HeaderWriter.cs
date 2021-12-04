using System.IO;

namespace HuePat.IMPoseNorm.Util.IO.Writing {
    static class HeaderWriter {
        public static void Write(
                string file,
                bool hasNormals,
                PLYEncoding encoding,
                long vertexCount,
                long faceCount) {

            using (StreamWriter writer = new StreamWriter(file)) {

                writer.WriteLine("ply");
                writer.WriteLine($"format {encoding.GetString()} 1.0");
                writer.WriteLine($"element vertex {vertexCount}");
                writer.WriteLine("property float x");
                writer.WriteLine("property float y");
                writer.WriteLine("property float z");

                if (hasNormals) {
                    writer.WriteLine("property float nx");
                    writer.WriteLine("property float ny");
                    writer.WriteLine("property float nz");
                }

                writer.WriteLine($"element face {faceCount}");
                writer.WriteLine("property list uchar int vertex_indices");
                writer.WriteLine("end_header");
            }
        }
    }
}
