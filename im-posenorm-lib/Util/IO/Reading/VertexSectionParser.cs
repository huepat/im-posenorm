using System.Collections.Generic;

namespace HuePat.IMPoseNorm.Util.IO.Reading {
    class VertexSectionParser : HeaderSectionParser {

        private bool hasNormalVector;
        private (string, string, string) coordinateIdentifiers;
        private (string, string, string) normalVectorIdentifiers;
        private Dictionary<string, int> coordinateIndices;
        private Dictionary<string, int> normalVectorIndices;

        public override string SectionName {
            get {
                return "vertex";
            }
        }

        public void Initialize(
                bool hasNormalVector,
                (string, string, string) coordinateIdentifiers,
                (string, string, string) normalVectorIdentifiers) {

            this.hasNormalVector = hasNormalVector;
            this.coordinateIdentifiers = coordinateIdentifiers;
            this.normalVectorIdentifiers = normalVectorIdentifiers;

            Initialize();
        }

        public override void Initialize() {

            base.Initialize();

            InitializeCoordinateIndices();
            InitializeNormalVectorIndices();
        }

        public override void ParsePropertyIndex(
                string propertyType, 
                string propertyName) {

            if (coordinateIndices.ContainsKey(propertyName)
                    && (propertyType == FLOAT_TYPE 
                        || propertyType == DOUBLE_TYPE)) {

                coordinateIndices[propertyName] = propertyIndex++;
            }
            else if (normalVectorIndices.ContainsKey(propertyName)
                    && (propertyType == FLOAT_TYPE 
                        || propertyType == DOUBLE_TYPE)) {

                normalVectorIndices[propertyName] = propertyIndex++;
            }
            else {
                base.ParsePropertyIndex(propertyType, propertyName);
            }
        }

        public new VertexSection Create(
                int count, 
                int indexOffset = 0) {

            VertexSection section = new VertexSection(
                base.Create(
                    count, 
                    indexOffset));

            section.CoordinateIndices = new int[] {
                coordinateIndices[coordinateIdentifiers.Item1],
                coordinateIndices[coordinateIdentifiers.Item2],
                coordinateIndices[coordinateIdentifiers.Item3]
            };

            if (hasNormalVector) {
                section.NormalVectorIndices = new int[] {
                    normalVectorIndices[normalVectorIdentifiers.Item1] + indexOffset,
                    normalVectorIndices[normalVectorIdentifiers.Item2] + indexOffset,
                    normalVectorIndices[normalVectorIdentifiers.Item3] + indexOffset
                };
            }

            return section;
        }

        protected override void Check() {

            base.Check();

            foreach (string identifier in coordinateIndices.Keys) {
                if (coordinateIndices[identifier] == NOT_SET_INDEX) {
                    ReportUnsetPropertyIndex("coordinate component", identifier);
                }
            }

            foreach (string identifier in normalVectorIndices.Keys) {
                if (normalVectorIndices[identifier] == NOT_SET_INDEX) {
                    ReportUnsetPropertyIndex("normal component", identifier);
                }
            }
        }

        private void InitializeCoordinateIndices() {

            coordinateIndices = new Dictionary<string, int>();

            coordinateIndices.Add(coordinateIdentifiers.Item1, NOT_SET_INDEX);
            coordinateIndices.Add(coordinateIdentifiers.Item2, NOT_SET_INDEX);
            coordinateIndices.Add(coordinateIdentifiers.Item3, NOT_SET_INDEX);
        }

        private void InitializeNormalVectorIndices() {

            normalVectorIndices = new Dictionary<string, int>();

            if (hasNormalVector) {
                normalVectorIndices.Add(normalVectorIdentifiers.Item1, NOT_SET_INDEX);
                normalVectorIndices.Add(normalVectorIdentifiers.Item2, NOT_SET_INDEX);
                normalVectorIndices.Add(normalVectorIdentifiers.Item3, NOT_SET_INDEX);
            }
        }
    }
}