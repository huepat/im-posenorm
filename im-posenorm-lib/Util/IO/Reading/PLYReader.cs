using System.IO;
using System;
using System.Collections.Generic;
using OpenTK.Mathematics;
using HuePat.IMPoseNorm.Util.Geometry;

namespace HuePat.IMPoseNorm.Util.IO.Reading {
    public class PLYReader {
        private HeaderParser headerParser;

        public (string, string, string) CoordinateIndentifiers { private get; set; }
        public (string, string, string) NormalVectorIndentifiers { private get; set; }

        public int LineBreakByteSize {
            set {
                headerParser.LineBreakByteSize = value;
            }
        }

        public PLYReader() {

            headerParser = new HeaderParser() {
                LineBreakByteSize = 2
            };
            CoordinateIndentifiers = ("x", "y", "z");
            NormalVectorIndentifiers = ("nx", "ny", "nz");
        }

        public PointCloud ReadPointCloud(string file) {

            List<Point> points;
            List<Vector3d> normals;
            Header header = ReadHeader(file, true);
            IDecoder decoder = GetDecoder(header);

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                decoder.ReadPointsAndNormals(
                    file,
                    header,
                    out points,
                    out normals);
            }

            return new PointCloud(
                points, 
                normals);
        }

        public Mesh ReadMesh(string file) {

            IDecoder decoder;
            Header header;
            Mesh mesh;

            header = ReadHeader(
                file, 
                false);

            decoder = GetDecoder(header);

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                mesh = decoder.ReadMesh(
                    file,
                    header);
            }

            return mesh;
        }

        private Header ReadHeader(
                string file,
                bool hasNormalVector) {

            Header header;

            headerParser.Initialize(
                hasNormalVector,
                CoordinateIndentifiers,
                NormalVectorIndentifiers);

            foreach (string line in File.ReadLines(file)) {

                header = headerParser.ParseHeaderLine(line);

                if (header != null) {
                    return header;
                }
            }

            return null;
        }

        private IDecoder GetDecoder(Header header) {

            switch (header.Encoding) {
                case PLYEncoding.BINARY_LITTLE_ENDIAN:
                    return new BinaryDecoder(true);
                case PLYEncoding.BINARY_BIG_ENDIAN:
                    return new BinaryDecoder(false);
                case PLYEncoding.ASCII:
                    return new AsciiDecoder();
                default:
                    throw new ApplicationException();
            }
        }
    }
}