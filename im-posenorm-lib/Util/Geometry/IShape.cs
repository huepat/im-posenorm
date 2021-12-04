using OpenTK.Mathematics;
using System.Collections.Generic;

namespace HuePat.IMPoseNorm.Util.Geometry {
    public enum ShapeType {
        POINT_CLOUD,
        MESH
    }

    public interface IShape {
        ShapeType Type { get; }
        AABox BBox { get; }
        Mesh Mesh { get; }
        IReadOnlyList<double> SizeWeights { get; }
        List<Vector3d> Normals { get; }
        IReadOnlyList<Point> Points { get; }


        void UpdateBBox();
        IShape Clone();
    }
}