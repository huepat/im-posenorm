using HuePat.IMPoseNorm.Util.Geometry;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace HuePat.IMPoseNorm.Util.IO.Reading {
    class BinaryDecoder : IDecoder {
        private bool littleEndian;

        public BinaryDecoder(bool littleEndian) {

            this.littleEndian = littleEndian;
        }

        public List<Point> ReadPoints(
                string file, 
                Header header) {

            return ReadPoints(
                CreateReader(
                    file, 
                    header),
                header);
        }

        public void ReadPointsAndNormals(
                string file, 
                Header header, 
                out List<Point> points, 
                out List<Vector3d> normals) {

            int propertyIndex, maxPropertyIndex;
            BinaryReader reader = CreateReader(
                file, 
                header);
            object[] properties;
            Dictionary<int, PropertyType> propertyTypes = header.VertexSection.PropertyTypes;

            points = new List<Point>();
            normals = new List<Vector3d>();

            maxPropertyIndex = propertyTypes.Keys.Max();
            properties = new object[maxPropertyIndex + 1];

            while (points.Count < header.VertexSection.Count) {

                propertyIndex = 0;

                while (propertyIndex <= maxPropertyIndex) {

                    properties[propertyIndex] = ReadProperty(
                        reader,
                        propertyTypes[propertyIndex]);

                    propertyIndex++;
                }

                points.Add(
                    new Point(
                        ParseVector3d(
                            properties,
                            header.VertexSection.CoordinateIndices)));

                normals.Add(
                    ParseVector3d(
                        properties,
                        header.VertexSection.NormalVectorIndices));
            }
        }

        public Mesh ReadMesh(
                string file, 
                Header header) {

            int propertyIndex, maxPropertyIndex;
            object[] properties;
            BinaryReader reader = CreateReader(
                file, 
                header);
            List<Point> vertices = ReadPoints(
                reader, 
                header);
            List<Face> faces = new List<Face>();
            Dictionary<int, PropertyType> propertyTypes = header.FaceSection.PropertyTypes;

            maxPropertyIndex = propertyTypes.Keys.Max();
            properties = new object[maxPropertyIndex + 1];

            while (faces.Count < header.FaceSection.Count) {

                propertyIndex = 0;

                while (propertyIndex <= maxPropertyIndex) {

                    properties[propertyIndex] = ReadProperty(
                        reader,
                        propertyTypes[propertyIndex]);

                    propertyIndex++;
                }

                faces.Add(
                    ParseFace(
                        properties,
                        header.FaceSection,
                        vertices));
            }

            return new Mesh(
                vertices, 
                faces);
        }

        private BinaryReader CreateReader(
                string file,
                Header header) {

            BinaryReader reader = new BinaryReader(
                File.Open(
                    file, 
                    FileMode.Open, 
                    FileAccess.Read),
                littleEndian ?
                    Encoding.Unicode :
                    Encoding.BigEndianUnicode);

            reader.BaseStream.Position = header.VertexSectionStartPosition;

            return reader;
        }

        private List<Point> ReadPoints(
                BinaryReader reader,
                Header header) {

            int propertyIndex, maxPropertyIndex;
            object[] properties;
            List<Point> points = new List<Point>();
            Dictionary<int, PropertyType> propertyTypes = header.VertexSection.PropertyTypes;

            maxPropertyIndex = propertyTypes.Keys.Max();
            properties = new object[maxPropertyIndex + 1];

            while (points.Count < header.VertexSection.Count) {

                propertyIndex = 0;

                while (propertyIndex <= maxPropertyIndex) {

                    properties[propertyIndex] = ReadProperty(
                        reader, 
                        propertyTypes[propertyIndex]);

                    propertyIndex++;
                }

                points.Add(
                    new Point(
                        ParseVector3d(
                            properties,
                            header.VertexSection.CoordinateIndices)));
            }

            return points;
        }

        private object ReadProperty(
                BinaryReader reader,
                PropertyType propertyType) {

            switch (propertyType) {
                case PropertyType.BYTE:
                    return reader.ReadByte();
                case PropertyType.INTEGER:
                    return reader.ReadInt32();
                case PropertyType.LONG:
                    return reader.ReadInt64();
                case PropertyType.FLOAT:
                    return reader.ReadSingle();
                case PropertyType.DOUBLE:
                    return reader.ReadDouble();
                case PropertyType.VECTOR3D:
                case PropertyType.COLOR:
                case PropertyType.BOOL:
                default:
                    break;
            }

            throw new ArgumentException();
        }

        private Face ParseFace(
                object[] properties,
                FaceSection faceSection,
                List<Point> vertices) {

            byte faceVertexCount = ParseByte(
                properties, 
                faceSection.VertexCountIndex);

            if (faceVertexCount != 3) {
                throw new ArgumentException(
                    "BinaryDecorder currently only parses faces with three indices.");
            }

            return new Face(
                ParseInteger(
                    properties, 
                    faceSection.VertexIndexIndices[0]),
                ParseInteger(
                    properties, 
                    faceSection.VertexIndexIndices[1]),
                ParseInteger(
                    properties, 
                    faceSection.VertexIndexIndices[2]),
                vertices
            );
        }

        private byte ParseByte(
                object[] properties,
                int index) {

            return (byte)properties[index];
        }

        private int ParseInteger(
                object[] properties, 
                int index) {

            return (int)properties[index];
        }

        private Vector3d ParseVector3d(
                object[] properties,
                int[] indices) {

            return new Vector3d(
                (float)properties[indices[0]],
                (float)properties[indices[1]],
                (float)properties[indices[2]]);
        }
    }
}