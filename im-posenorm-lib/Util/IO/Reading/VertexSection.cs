using System.Collections.Generic;

namespace HuePat.IMPoseNorm.Util.IO.Reading {
    class VertexSection : HeaderSection {
        public int[] CoordinateIndices { get; set; }
        public int[] NormalVectorIndices { get; set; }

        public bool HasNormalVector {
            get {
                return NormalVectorIndices != null;
            }
        }

        public override Dictionary<int, PropertyType> PropertyTypes {
            get {

                Dictionary<int, PropertyType> types = base.PropertyTypes;

                types.Add(CoordinateIndices[0], PropertyType.FLOAT);
                types.Add(CoordinateIndices[1], PropertyType.FLOAT);
                types.Add(CoordinateIndices[2], PropertyType.FLOAT);

                if (HasNormalVector) {
                    types.Add(NormalVectorIndices[0], PropertyType.FLOAT);
                    types.Add(NormalVectorIndices[1], PropertyType.FLOAT);
                    types.Add(NormalVectorIndices[2], PropertyType.FLOAT);
                }

                return types;
            }
        }

        public VertexSection(
                HeaderSection headerSection) : 
                    base(headerSection) {
        }
    }
}