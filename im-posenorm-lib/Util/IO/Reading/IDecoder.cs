using HuePat.IMPoseNorm.Util.Geometry;
using OpenTK.Mathematics;
using System.Collections.Generic;

namespace HuePat.IMPoseNorm.Util.IO.Reading {
    interface IDecoder {
        List<Point> ReadPoints(
            string file, 
            Header header);

        void ReadPointsAndNormals(
            string file,
            Header header,
            out List<Point> points,
            out List<Vector3d> normals);

        Mesh ReadMesh(
            string file,
            Header header);
    }
}