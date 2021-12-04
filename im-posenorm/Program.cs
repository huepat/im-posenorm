using HuePat.IMPoseNorm.Eval;
using HuePat.IMPoseNorm.Util;
using HuePat.IMPoseNorm.Util.Geometry;
using HuePat.IMPoseNorm.Util.IO.Writing;
using OpenTK.Mathematics;

namespace HuePat.IMPoseNorm {
    public static class Program {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // IM-PoseNorm: Pose Normalization for Indoor Mapping Datasets
        //-----------------------------------------------------------
        //
        // This solution contains the code for the publication: 
        //
        //      Hübner, P.; Weinmann, M.; Wursthorn, S. & Hinz, S. (2021).
        //      Pose Normalization of Indoor Mapping Datasets Partially Compliant with the Manhattan World Assumption.
        //      Remote Sensing, 2021, 13(23), 4765
        //      https://www.mdpi.com/2072-4292/13/23/4765
        //
        // This Console Project contains a simple executable example to demonstrate, how IM-PoseNorm is used.
        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        public static void Main(string[] args) {

            // Create a mesh (simple demo scenario of two boxes):
            Mesh mesh = Mesh.Merge(new Mesh[] {
                new AABox(
                    new Vector3d(0.0, 0.0, 0.0),
                    new Vector3d(5.0, 10.0, 2.5)).Mesh,
                new AABox(
                    new Vector3d(5.0, 0.0, 0.0),
                    new Vector3d(15.0, 5.0, 2.5)).Mesh
            });

            // Note: You can load triangle meshes with the PLYReader (for more information, see the ReadMe of the repository):
            // mesh = new PLYReader().ReadMesh("path/to/MeshFile.ply");

            // rotate the demo mesh by 19° around the vertical z-axis, an by 13 degree around the horizontal x-axis
            Vector3d verticalAxis = new Vector3d(0.0, 0.0, 1.0);
            Vector3d horizontalAxis = new Vector3d(1.0, 0.0, 0.0);
            Matrix3d rotation = horizontalAxis.GetRotationAround(13.0.DegreeToRadian())
                * verticalAxis.GetRotationAround(19.0.DegreeToRadian());
            mesh.Rotate(
                new Rotation.Config(rotation));

            // Export the created demo 'house' to a PLY file
            // (you can find the created output in 'path/to/voxir/voxir/bin/Debug/net5.0' or 'path/to/voxir/voxir/bin/Release/net5.0')
            new PLYWriter().Write(
                "./demo_mesh.ply",
                mesh);

            // Apply pose normalization.
            // The 'horizontal' faces of the mesh are now perpendicular again to the vertical axis and the 'vertical' faces are perpendicular to one of the horizontal axes.
            mesh.NormalizePose(
                verticalAxis,
                horizontalAxis);

            // Export the normalized mesh
            new PLYWriter().Write(
                "./demo_mesh_normalized.ply",
                mesh);

            // Run the evaluation procedure:
            //     The normalized mesh gets rotated 20 times by an arbitrary random rotation around the vertical axis
            //          and by random rotations of up to 30° around the horizontal axes.
            //     For each rotated sample, pose normalization is applied and evaluated as described in the paper.
            Evaluation.Evaluate(
                new Evaluation.Config(
                        30.0.DegreeToRadian(),
                        "./demo_mesh_normalized.ply",
                        Evaluation.Type.MESH,
                        verticalAxis,
                        horizontalAxis) { 

                    SampleCount = 20,

                    // Note: instead of evaluating random rotations, rotations can also be specified explicitely as:
                    //Angles = new (double, double, double)[] { 
                    //    (verticalAngle_1, HorizontalAngle1_1, HorizontalAngle1_2),
                    //    (verticalAngle_2, HorizontalAngle1_2, HorizontalAngle1_2),
                    //    ...
                    //}
            });
        }
    }
}