using HuePat.IMPoseNorm.Util.Geometry;
using OpenTK.Mathematics;
using System;

namespace HuePat.IMPoseNorm.Util.IO.Writing {
    public interface IEncoder : IDisposable {
        void Encode(Point point);
        void Encode(
            Point point,
            Vector3d normal);
        void Encode(
            Face face, 
            int offset);
    }
}