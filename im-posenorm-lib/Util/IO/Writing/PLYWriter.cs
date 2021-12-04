using HuePat.IMPoseNorm.Util.Geometry;
using System;

namespace HuePat.IMPoseNorm.Util.IO.Writing {
    public class PLYWriter {
        public PLYEncoding Encoding { private get; set; }

        public PLYWriter() {

            Encoding = PLYEncoding.BINARY_LITTLE_ENDIAN;
        }

        protected IEncoder GetEncoder(string file) {

            switch (Encoding) {
                case PLYEncoding.BINARY_LITTLE_ENDIAN:
                    return new BinaryEncoder(file, true);
                case PLYEncoding.BINARY_BIG_ENDIAN:
                    return new BinaryEncoder(file, false);
                case PLYEncoding.ASCII:
                    return new AsciiEncoder(file);
            }

            throw new ApplicationException();
        }

        public void Write(
                string file,
                PointCloud pointCloud) {

            HeaderWriter.Write(
                file,
                true,
                Encoding,
                pointCloud.Count,
                0);

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                using (IEncoder encoder = GetEncoder(file)) {
                    for (int i = 0; i < pointCloud.Count; i++) {
                        encoder.Encode(
                            pointCloud[i],
                            pointCloud.Normals[i]);
                    }
                }
            }
        }

        public void Write(
                string file,
                Mesh mesh) {

            HeaderWriter.Write(
                file,
                false,
                Encoding,
                mesh.Vertices.Count,
                mesh.Count);

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                using (IEncoder encoder = GetEncoder(file)) {
                    for (int i = 0; i < mesh.Vertices.Count; i++) {
                        encoder.Encode(mesh.Vertices[i]);
                    }
                    for (int i = 0; i < mesh.Count; i++) {
                        encoder.Encode(mesh[i], 0);
                    }
                }
            }
        }
    }
}