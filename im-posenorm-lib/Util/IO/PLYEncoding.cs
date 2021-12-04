using System;

namespace HuePat.IMPoseNorm.Util.IO {
    enum PropertyType {
        BOOL,
        BYTE,
        INTEGER,
        LONG,
        FLOAT,
        DOUBLE,
        VECTOR3D,
        COLOR
    }

    public enum PLYEncoding {
        BINARY_LITTLE_ENDIAN,
        BINARY_BIG_ENDIAN,
        ASCII
    }

    public static class Extensions {
        public static string GetString(this PLYEncoding encoding) {

            switch (encoding) {
                case PLYEncoding.BINARY_LITTLE_ENDIAN:
                    return "binary_little_endian";
                case PLYEncoding.BINARY_BIG_ENDIAN:
                    return "binary_big_endian";
                case PLYEncoding.ASCII:
                    return "ascii";
            }

            throw new ArgumentException();
        }
    }
}