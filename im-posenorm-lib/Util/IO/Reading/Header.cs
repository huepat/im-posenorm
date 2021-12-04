namespace HuePat.IMPoseNorm.Util.IO.Reading {
    class Header {
        public PLYEncoding Encoding { get; private set; }
        public int HeaderLineCount { get; private set; }
        public int VertexSectionStartPosition { get; private set; }
        public VertexSection VertexSection { get; private set; }
        public FaceSection FaceSection { get; private set; }

        public bool HasFaces {
            get {
                return FaceSection.Count > 0;
            }
        }

        public Header(
                PLYEncoding encoding,
                int headerLineCount, 
                int vertexSectionStartPosition,
                VertexSection vertexSection,
                FaceSection faceSection) {

            Encoding = encoding;
            HeaderLineCount = headerLineCount;
            VertexSectionStartPosition = vertexSectionStartPosition;
            VertexSection = vertexSection;
            FaceSection = faceSection;
        }
    }
}