using HuePat.IMPoseNorm.Util.Geometry;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace HuePat.IMPoseNorm.Util.IO.Reading {
    class AsciiDecoder : IDecoder {
        public List<Point> ReadPoints(
                string file, 
                Header header) {

            List<Point> points = new List<Point>();

            ReadLines(
                file,
                header.HeaderLineCount,
                header.HeaderLineCount + header.VertexSection.Count,
                line => {

                    points.Add(
                        new Point(
                            ParseVector3d(
                                SplitValues(line),
                                header.VertexSection.CoordinateIndices)));

                });

            return points;
        }

        public void ReadPointsAndNormals(
                string file, 
                Header header, 
                out List<Point> points, 
                out List<Vector3d> normals) {

            string[] values;
            List<Point> _points = new List<Point>();
            List<Vector3d> _normals = new List<Vector3d>();

            ReadLines(
                file,
                header.HeaderLineCount,
                header.HeaderLineCount + header.VertexSection.Count,
                line => {

                    values = SplitValues(line);

                    _points.Add(
                        new Point(
                            ParseVector3d(
                                values,
                                header.VertexSection.CoordinateIndices)));

                    _normals.Add(
                        ParseVector3d(
                            values,
                            header.VertexSection.NormalVectorIndices));

                });

            points = _points;
            normals = _normals;
        }

        public Mesh ReadMesh(
                string file, 
                Header header) {

            string[] values;
            List<Face> faces = new List<Face>();
            List<Point> vertices = ReadPoints(
                file, 
                header);

            ReadLines(
                file,
                header.HeaderLineCount + header.VertexSection.Count,
                header.HeaderLineCount + header.VertexSection.Count + header.FaceSection.Count,
                line => {

                    values = SplitValues(line);

                    if (int.Parse(values[0]) != 3) {
                        throw new ArgumentException(
                            "FaceParser currently only parses faces with three indices.");
                    }

                    faces.Add(
                        new Face(
                            int.Parse(values[1]),
                            int.Parse(values[2]),
                            int.Parse(values[3]),
                            vertices));
                });

            return new Mesh(
                vertices, 
                faces);
        }

        private static void ReadLines(
                string file,
                int startIndex,
                int stopIndex,
                Action<string> callback) {

            int index = 0;

            foreach (string line in File.ReadLines(file)) {

                if (index >= stopIndex) {
                    return;
                }

                if (index >= startIndex) {
                    callback(line);
                }

                index++;
            }
        }

        private string[] SplitValues(string line) {

            return line
                .Split(' ')
                .Where(v => v != "")
                .ToArray();
        }

        private Vector3d ParseVector3d(
                string[] values, 
                int[] indices) {

            return new Vector3d(
                double.Parse(values[indices[0]]),
                double.Parse(values[indices[1]]),
                double.Parse(values[indices[2]]));
        }
    }
}