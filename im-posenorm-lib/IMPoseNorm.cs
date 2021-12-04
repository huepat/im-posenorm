using HuePat.IMPoseNorm.Util;
using HuePat.IMPoseNorm.Util.Geometry;
using HuePat.IMPoseNorm.Util.Statistics;
using OpenTK.Mathematics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.IMPoseNorm {
    public static class IMPoseNorm {

        private const double VERTICAL_PEAK_RATIO = 0.75;
        private const double HORIZONTAL_PEAK_RATIO = 0.75;
        private const double UNAMBIGUOUS_ALIGN_SIZE_FRACTION = 0.1;
        private static readonly double DEGREE_45 = 45.0.DegreeToRadian();
        private static readonly double DEGREE_90 = 90.0.DegreeToRadian();
        private static readonly double DEGREE_180 = 180.0.DegreeToRadian();
        private static readonly double VERTICAL_RESOLUTION = 1.0.DegreeToRadian();
        private static readonly double HORIZONTAL_RESOLUTION = 1.0.DegreeToRadian();
        private static readonly double VERTICAL_ALIGNMENT_ANGLE_RADIUS = 40.0.DegreeToRadian();
        private static readonly double VERTICAL_AXIS_REFINEMENT_ANGLE_RADIUS = 5.0.DegreeToRadian();
        private static readonly double HORIZONTAL_ANGLE_REFINEMENT_ANGLE_RADIUS = 5.0.DegreeToRadian();
        private static readonly double HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MIN_THRESHOLD = 45.0.DegreeToRadian();
        private static readonly double HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MAX_THRESHOLD = 135.0.DegreeToRadian();

        public static void NormalizePose(
                this IShape shape,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                bool horizontallyUnambiguatePose = false) {

            shape.NormalizePose(
                upAxis,
                horizontalAxis,
                horizontallyUnambiguatePose,
                out _);
        }

        public static void NormalizePose(
                this IShape shape,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                bool horizontallyUnambiguatePose,
                out Matrix3d rotation) {

            Vector3d centroid = shape.GetCentroid();
            IReadOnlyList<double> weights = shape.SizeWeights;

            rotation = shape.NormalizePoseVertically(
                centroid,
                upAxis,
                horizontalAxis,
                weights);

            rotation = shape.NormalizePoseHorizontally(
                    centroid,
                    upAxis,
                    horizontalAxis,
                    weights)
                * rotation;

            if (horizontallyUnambiguatePose) {
                shape.HorizontallyUnambiguatePose(
                    centroid,
                    upAxis,
                    horizontalAxis,
                    weights);
            }
        }

        private static Matrix3d NormalizePoseVertically(
                this IShape shape,
                Vector3d centroid,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                IReadOnlyList<double> weights) {

            Matrix3d rotation;
            double[,] verticalGrid;
            List<Vector3d> normals = shape.Normals;
            Dictionary<(int, int), List<(Vector3d, double)>> weightedNormalsPerVerticalGridCell;

            verticalGrid = CreateVerticalGrid(
                upAxis,
                horizontalAxis,
                weights,
                normals,
                out weightedNormalsPerVerticalGridCell);

            RemoveMinorNormalClustersPerGridCell(
                verticalGrid,
                weightedNormalsPerVerticalGridCell);

            rotation = GetNormalizedUpAxis(
                    upAxis,
                    verticalGrid,
                    weights,
                    normals,
                    weightedNormalsPerVerticalGridCell)
                .RotationTo(upAxis);

            shape.Rotate(
                new Rotation.Config(rotation) {
                    Anchor = centroid,
                    RotateNormals = shape.Type == ShapeType.POINT_CLOUD
                });

            return rotation;
        }

        private static double[,] CreateVerticalGrid(
                Vector3d upAxis,
                Vector3d horizontalAxis,
                IReadOnlyList<double> weights,
                IReadOnlyList<Vector3d> normals,
                out Dictionary<(int, int), List<(Vector3d, double)>> weightedNormalsPerVerticalGridCell) {

            object @lock = new object();
            Vector3d horizontalAxis2 = Vector3d.Cross(
                upAxis, 
                horizontalAxis);
            double[,] verticalGrid = new double[
                (int)(DEGREE_90 / VERTICAL_RESOLUTION).Ceil(),
                (int)(VERTICAL_ALIGNMENT_ANGLE_RADIUS / VERTICAL_RESOLUTION).Ceil()];
            Dictionary<(int, int), List<(Vector3d, double)>> _weightedNormalsPerVerticalGridCell
                = new Dictionary<(int, int), List<(Vector3d, double)>>();

            Parallel.ForEach(
                Partitioner.Create(0, normals.Count),
                () => (
                    new double[
                        verticalGrid.GetLength(0),
                        verticalGrid.GetLength(1)],
                    new Dictionary<(int, int), List<(Vector3d, double)>>()),
                (partition, loopState, localState) => {

                    for (int j = partition.Item1; j < partition.Item2; j++) {
                        UpdateVerticalGrid(
                            weights[j],
                            localState.Item1,
                            normals[j],
                            upAxis,
                            horizontalAxis,
                            horizontalAxis2,
                            localState.Item2);
                    }

                    return localState;
                },
                localState => {

                    int a, i;

                    lock (@lock) {

                        _weightedNormalsPerVerticalGridCell.BucketAdd(localState.Item2);

                        for (a = 0; a < verticalGrid.GetLength(0); a++) {
                            for (i = 0; i < verticalGrid.GetLength(1); i++) {
                                verticalGrid[a, i] += localState.Item1[a, i];
                            }
                        }
                    }

                });

            weightedNormalsPerVerticalGridCell = _weightedNormalsPerVerticalGridCell;

            ProcessGridPole(
                verticalGrid,
                weightedNormalsPerVerticalGridCell);

            return verticalGrid;
        }

        private static void UpdateVerticalGrid(
                double weight,
                double[,] verticalGrid,
                Vector3d normal,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                Vector3d horizontalAxis2,
                Dictionary<(int, int), List<(Vector3d, double)>> weightedNormalsPerVerticalGridCell) {

            int a, i;
            double azimuth, inclination;

            if (double.IsNaN(normal.X)
                    || double.IsNaN(normal.Y)
                    || double.IsNaN(normal.Z)
                    || double.IsNaN(weight)) {
                return;
            }

            inclination = DEGREE_90 - (Vector3d.Dot(normal, upAxis).Acos() - DEGREE_90).Abs();

            if (double.IsNaN(inclination)
                    || inclination.Abs() > VERTICAL_ALIGNMENT_ANGLE_RADIUS) {
                return;
            }

            azimuth = (System.Math.Atan2(
                    Vector3d.Dot(normal, horizontalAxis),
                    Vector3d.Dot(normal, horizontalAxis2)).Abs()
                - DEGREE_90).Abs();

            a = (int)(azimuth / VERTICAL_RESOLUTION);
            i = (int)(inclination / VERTICAL_RESOLUTION);

            if (a == verticalGrid.GetLength(0)) {
                a = 0;
            }

            verticalGrid[a, i] += weight;
            weightedNormalsPerVerticalGridCell.BucketAdd(
                (a, i),
                (normal, weight));
        }

        private static void ProcessGridPole(
                double[,] verticalGrid,
                Dictionary<(int, int), List<(Vector3d, double)>> weightedNormalsPerVerticalGridCell) {

            for (int a = 1; a < verticalGrid.GetLength(0); a++) {

                verticalGrid[0, 0] += verticalGrid[a, 0];
                verticalGrid[a, 0] = 0;

                if (weightedNormalsPerVerticalGridCell.ContainsKey((a, 0))) {

                    weightedNormalsPerVerticalGridCell.BucketAdd(
                        (0, 0),
                        weightedNormalsPerVerticalGridCell[(a, 0)]);

                    weightedNormalsPerVerticalGridCell.Remove((a, 0));
                }
            }
        }

        private static void RemoveMinorNormalClustersPerGridCell(
                double[,] verticalGrid,
                Dictionary<(int, int), List<(Vector3d, double)>> weightedNormalsPerVerticalGridCell) {

            List<(int, int)> verticalGridCellKeys = weightedNormalsPerVerticalGridCell.Keys.ToList();

            Parallel.For(
                0,
                verticalGridCellKeys.Count,
                j => {

                    ;
                    List<VectorStatistics> clusterCenters = new List<VectorStatistics>();
                    List<(Vector3d, double)> mainClusterNormals;
                    List<List<(Vector3d, double)>> clusters = new List<List<(Vector3d, double)>>();
                    List<List<(Vector3d, double)>> finalClusters;

                    ClusterNormalsPerVerticalGridCell(
                        verticalGridCellKeys[j],
                        clusterCenters,
                        clusters, 
                        weightedNormalsPerVerticalGridCell);

                    finalClusters = MergeClustersWithAntipodalDirections(
                        clusterCenters,
                        clusters);

                    if (finalClusters.Count > 1) {

                        mainClusterNormals = finalClusters
                            .WhereMax(cluster => cluster.Count)
                            .SelectMany(cluster => cluster)
                            .ToList();

                        verticalGrid[
                                verticalGridCellKeys[j].Item1,
                                verticalGridCellKeys[j].Item2]
                            = mainClusterNormals.Sum(weightedNormal => weightedNormal.Item2);

                        weightedNormalsPerVerticalGridCell[verticalGridCellKeys[j]] = mainClusterNormals;
                    }
                });
        }

        private static void ClusterNormalsPerVerticalGridCell(
                (int, int) verticalGridCellKey,
                List<VectorStatistics> clusterCenters,
                List<List<(Vector3d, double)>> clusters,
                Dictionary<(int, int), List<(Vector3d, double)>> weightedNormalsPerVerticalGridCell) {

            bool found;
            int k;
            double angle;

            foreach ((Vector3d, double) weightedNormal in weightedNormalsPerVerticalGridCell[verticalGridCellKey]) {

                found = false;

                for (k = 0; k < clusterCenters.Count; k++) {

                    angle = weightedNormal.Item1.AngleTo(clusterCenters[k].Mean);

                    if (angle < 2 * VERTICAL_RESOLUTION) {
                        found = true;
                        clusters[k].Add(weightedNormal);
                        clusterCenters[k].Update(weightedNormal.Item1);
                        break;
                    }
                }

                if (!found) {

                    clusters.Add(
                        new List<(Vector3d, double)> {
                            weightedNormal
                        });

                    clusterCenters.Add(new VectorStatistics());

                    clusterCenters[clusterCenters.Count - 1]
                        .Update(weightedNormal.Item1);
                }
            }
        }

        private static List<List<(Vector3d, double)>> MergeClustersWithAntipodalDirections(
                List<VectorStatistics> clusterCenters,
                List<List<(Vector3d, double)>> clusters) {

            bool found;
            int k, l;
            List<Vector3d> finalClusterCenters = new List<Vector3d>();
            List<List<(Vector3d, double)>> finalClusters = new List<List<(Vector3d, double)>>();

            for (k = 0; k < clusters.Count; k++) {

                found = false;

                for (l = 0; l < finalClusters.Count; l++) {
                    if ((clusterCenters[k]
                                .Mean
                                .AngleTo(finalClusterCenters[l]) - DEGREE_180)
                            .Abs() < 2 * VERTICAL_RESOLUTION) {
                        found = true;
                        finalClusters[l].AddRange(clusters[k]);
                        break;
                    }
                }

                if (!found) {
                    finalClusters.Add(clusters[k]);
                    finalClusterCenters.Add(clusterCenters[k].Mean);
                }
            }

            return finalClusters;
        }

        private static Vector3d GetNormalizedUpAxis(
                Vector3d upAxis,
                double[,] verticalGrid,
                IReadOnlyList<double> weights,
                IReadOnlyList<Vector3d> normals,
                Dictionary<(int, int), List<(Vector3d, double)>> weightedNormalsPerVerticalGridCell) {

            Vector3d normalizedUpAxis;
            List<double> clusterSums = new List<double>();
            List<List<(int, int)>> clusters = new List<List<(int, int)>>();

            ClusterVerticalGridCells(
                verticalGrid,
                out clusterSums,
                out clusters);

            normalizedUpAxis = clusters[
                    Enumerable
                        .Range(0, clusters.Count)
                        .WhereMax(j => clusterSums[j])
                        .First()]
                .Where(gridPosition => weightedNormalsPerVerticalGridCell.ContainsKey(gridPosition))
                .SelectMany(gridPosition => weightedNormalsPerVerticalGridCell[gridPosition])
                .Select(weightedNormal => (
                    Vector3d.Dot(upAxis, weightedNormal.Item1) < 0.0 ? -weightedNormal.Item1 : weightedNormal.Item1,
                    weightedNormal.Item2
                ))
                .WeightedMean()
                .Normalized();

            return RefineNormalizedUpAxis(
                upAxis,
                normalizedUpAxis,
                weights,
                normals);
        }

        

        private static void ClusterVerticalGridCells(
                double[,] verticalGrid,
                out List<double> clusterSums,
                out List<List<(int, int)>> clusters) {

            int a, i;
            (int, int) candidate;
            double threshold;
            double clusterSum;
            bool[,] alreadyClustered = new bool[
                verticalGrid.GetLength(0),
                verticalGrid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            clusterSums = new List<double>();
            clusters = new List<List<(int, int)>>();

            threshold = GetVerticalGridCellWeightSumThreshold(verticalGrid);

            for (a = 0; a < verticalGrid.GetLength(0); a++) {
                for (i = 0; i < verticalGrid.GetLength(1); i++) {

                    if (alreadyClustered[a, i]
                            || verticalGrid[a, i] < threshold) {
                        continue;
                    }

                    clusterSum = 0.0;
                    List<(int, int)> cluster = new List<(int, int)>();
                    candidates.Enqueue((a, i));

                    do {

                        candidate = candidates.Dequeue();

                        if (alreadyClustered[
                                candidate.Item1,
                                candidate.Item2]) {
                            continue;
                        }

                        alreadyClustered[
                            candidate.Item1,
                            candidate.Item2] = true;

                        cluster.Add(candidate);
                        clusterSum += verticalGrid[
                            candidate.Item1,
                            candidate.Item2];

                        CheckVerticalGridCellNeighbours(
                            threshold,
                            candidate,
                            alreadyClustered,
                            verticalGrid,
                            candidates,
                            cluster);
                        
                    } while (candidates.Count > 0);

                    clusters.Add(cluster);
                    clusterSums.Add(clusterSum);
                }
            }
        }

        private static double GetVerticalGridCellWeightSumThreshold(
                double[,] verticalGrid) {

            int a, i;
            double threshold = double.MinValue;

            for (a = 0; a < verticalGrid.GetLength(0); a++) {
                for (i = 0; i < verticalGrid.GetLength(1); i++) {
                    if (verticalGrid[a, i] > threshold) {
                        threshold = verticalGrid[a, i];
                    }
                }
            }

            threshold *= VERTICAL_PEAK_RATIO;

            return threshold;
        }

        private static void CheckVerticalGridCellNeighbours(
                double threshold,
                (int, int) candidate,
                bool[,] alreadyClustered,
                double[,] verticalGrid,
                Queue<(int, int)> candidates,
                List<(int, int)> cluster) {

            int da, a, di, i;

            foreach ((int, int) position in GetVerticalGridCellsToCheckNeighbours(
                    candidate,
                    alreadyClustered,
                    verticalGrid,
                    cluster)) {

                for (da = -1; da <= 1; da++) {
                    for (di = -1; di <= 1; di++) {

                        if (da == 0 && di == 0) {
                            continue;
                        }

                        a = position.Item1 + da;
                        i = position.Item2 + di;
                        if (i < 0
                                || i >= verticalGrid.GetLength(1)) {
                            continue;
                        }
                        if (a == -1) {
                            a = verticalGrid.GetLength(0) - 1;
                        }
                        if (a == verticalGrid.GetLength(0)) {
                            a = 0;
                        }
                        if (!alreadyClustered[a, i]
                                && verticalGrid[a, i] >= threshold) {
                            candidates.Enqueue((a, i));
                        }
                    }
                }
            }
        }

        private static List<(int, int)> GetVerticalGridCellsToCheckNeighbours(
                (int, int) gridCell,
                bool[,] alreadyClustered,
                double[,] verticalGrid,
                List<(int, int)> cluster) {

            int a;
            List<(int, int)>  gridCellsToCheckNeighbours = new List<(int, int)> {
                gridCell
            };

            if (gridCell.Item1 == 0
                    && gridCell.Item2 == 0) {

                for (a = 1; a < verticalGrid.GetLength(1); a++) {
                    alreadyClustered[a, 0] = true;
                    cluster.Add((a, 0));
                    gridCellsToCheckNeighbours.Add((a, 0));
                }
            }

            return gridCellsToCheckNeighbours;
        }

        private static Vector3d RefineNormalizedUpAxis(
                Vector3d upAxis,
                Vector3d normalizedUpAxis,
                IReadOnlyList<double> weights,
                IReadOnlyList<Vector3d> normals) {

            return Enumerable
                .Range(0, normals.Count)
                .AsParallel()
                .Where(j => {

                    if (double.IsNaN(weights[j])) {
                        return false;
                    }

                    double angle = normals[j]
                        .AngleTo(normalizedUpAxis)
                        .Abs();

                    return angle < VERTICAL_AXIS_REFINEMENT_ANGLE_RADIUS
                        || (angle - DEGREE_180).Abs() < VERTICAL_AXIS_REFINEMENT_ANGLE_RADIUS;

                })
                .Select(j => (
                    Vector3d.Dot(
                            upAxis,
                            normals[j]) < 0.0 ?
                        -normals[j] :
                        normals[j],
                    weights[j]
                ))
                .WeightedMedian()
                .Normalized();
        }

        private static Matrix3d NormalizePoseHorizontally(
                this IShape shape,
                Vector3d centroid,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                IReadOnlyList<double> weights) {

            double angle;
            double[] horizontalGrid;
            Matrix3d rotation;
            List<(double, double)> weightedAngles;

            weightedAngles = shape.GetWeightedAnglesToHorizontalAxis(
                upAxis,
                horizontalAxis,
                weights);

            horizontalGrid = CreateHorizontalGrid(weightedAngles);

            angle = GetAngleOfRotationAroundVerticalAxis(
                horizontalGrid,
                weightedAngles);

            rotation = upAxis.GetRotationAround(angle);

            shape.Rotate(
                new Rotation.Config(rotation) {
                    UpdateBBox = true,
                    Anchor = centroid,
                    RotateNormals = shape.Type == ShapeType.POINT_CLOUD
                });

            return rotation;
        }

        private static List<(double, double)> GetWeightedAnglesToHorizontalAxis(
                this IShape shape,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                IReadOnlyList<double> weights) {

            List<Vector3d> normals = shape.Normals;

            return Enumerable
                .Range(0, normals.Count)
                .AsParallel()
                .Where(j => {

                    if (double.IsNaN(normals[j].X)
                            || double.IsNaN(normals[j].Y)
                            || double.IsNaN(normals[j].Z)
                            || double.IsNaN(weights[j])) {
                        return false;
                    }

                    double angle = normals[j]
                        .AngleTo(upAxis)
                        .Abs();

                    return angle >= HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MIN_THRESHOLD
                        && angle <= HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MAX_THRESHOLD;

                })
                .Select(j => {

                    double angle = normals[j]
                         .OrthogonalProject(upAxis)
                         .AngleTo(
                            horizontalAxis, 
                            upAxis);

                    if (angle < 0.0) {
                        angle += DEGREE_180;
                    }

                    if (angle > DEGREE_90) {
                        angle -= DEGREE_90;
                    }

                    return (angle, weights[j]);

                })
                .ToList();
        }

        private static double[] CreateHorizontalGrid(
                List<(double, double)> weightedAngles) {

            object @lock = new object();
            double[] horizontalGrid = new double[
                (int)(DEGREE_90 / HORIZONTAL_RESOLUTION).Ceil()];

            Parallel.ForEach(
                Partitioner.Create(0, weightedAngles.Count),
                () => new double[horizontalGrid.Length],
                (partition, loopState, localGrid) => {

                    int a;

                    for (int j = partition.Item1; j < partition.Item2; j++) {

                        a = (int)(weightedAngles[j].Item1 / HORIZONTAL_RESOLUTION);

                        if (a == localGrid.Length) {
                            a = 0;
                        }

                        localGrid[a] += weightedAngles[j].Item2;
                    }

                    return localGrid;

                },
                localGrid => {

                    lock (@lock) {
                        for (int j = 0; j < horizontalGrid.Length; j++) {
                            horizontalGrid[j] += localGrid[j];
                        }
                    }

                });

            return horizontalGrid;
        }

        private static double GetAngleOfRotationAroundVerticalAxis(
                double[] horizontalGrid,
                List<(double, double)> weightedAngles) {

            double angle;
            List<double> clusterSums;
            List<List<int>> clusters;

            ClusterHorizontalGridCells(
                horizontalGrid,
                out clusters,
                out clusterSums);

            angle = GetAngleOfRotationAroundVerticalAxis(
                horizontalGrid,
                clusterSums,
                clusters);

            angle = RefineAngleOfRotationAroundVerticalAxis(
                angle,
                weightedAngles);

            return angle;
        }

        private static void ClusterHorizontalGridCells(
                double[] horizontalGrid,
                out List<List<int>> clusters,
                out List<double> clusterSums) {

            int da, a, a2;
            int candidate;
            double clusterSum;
            double threshold = HORIZONTAL_PEAK_RATIO * horizontalGrid.Max();
            bool[] alreadyClustered = new bool[horizontalGrid.Length];
            Queue<int> candidates = new Queue<int>();

            clusterSums = new List<double>();
            clusters = new List<List<int>>();

            for (a = 0; a < horizontalGrid.Length; a++) {

                if (alreadyClustered[a]
                        || horizontalGrid[a] < threshold) {
                    continue;
                }

                clusterSum = 0.0;
                List<int> cluster = new List<int>();
                candidates.Enqueue(a);

                do {

                    candidate = candidates.Dequeue();
                    if (alreadyClustered[candidate]) {
                        continue;
                    }

                    alreadyClustered[candidate] = true;
                    cluster.Add(candidate);
                    clusterSum += horizontalGrid[candidate];

                    for (da = -1; da <= 1; da += 2) {

                        a2 = candidate + da;

                        if (a2 == -1) {
                            a2 = horizontalGrid.Length - 1;
                        }

                        if (a2 == horizontalGrid.Length) {
                            a2 = 0;
                        }

                        if (!alreadyClustered[a2]
                                && horizontalGrid[a2] >= threshold) {
                            candidates.Enqueue(a2);
                        }
                    }

                } while (candidates.Count > 0);

                clusters.Add(cluster);
                clusterSums.Add(clusterSum);
            }
        }

        private static double GetAngleOfRotationAroundVerticalAxis(
                double[] horizontalGrid,
                List<double> clusterSums,
                List<List<int>> clusters) {

            int maxSumClusterIndex;
            double diff;
            double firstAngle;

            maxSumClusterIndex = Enumerable
                .Range(0, clusters.Count)
                .WhereMax(j => clusterSums[j])
                .First();

            firstAngle = clusters[maxSumClusterIndex].First() * HORIZONTAL_RESOLUTION;

            return clusters[maxSumClusterIndex]
                .Select(a => {

                    diff = a * HORIZONTAL_RESOLUTION - firstAngle;

                    if (diff.Abs() > DEGREE_45) {
                        if (diff < 0.0) {
                            diff += DEGREE_90;
                        }
                        else {
                            diff -= DEGREE_90;
                        }
                    }

                    return (
                        firstAngle + diff, 
                        horizontalGrid[a]
                    );

                })
                .WeightedMean();
        }

        private static double RefineAngleOfRotationAroundVerticalAxis(
                double angle,
                List<(double, double)> weightedAngles) {

            return weightedAngles
                .AsParallel()
                .Where(a => {

                    double diff = (angle - a.Item1).Abs();

                    if (diff > DEGREE_45) {
                        diff -= DEGREE_90;
                    }

                    return diff.Abs() < HORIZONTAL_ANGLE_REFINEMENT_ANGLE_RADIUS;

                })
                .Select(a => {

                    double diff = a.Item1 - angle;

                    if (diff.Abs() > DEGREE_45) {
                        if (diff < 0) {
                            diff += DEGREE_90;
                        }
                        else {
                            diff -= DEGREE_90;
                        }
                    }

                    return (angle + diff, a.Item2);

                })
                .WeightedMedian();
        }

        private static void HorizontallyUnambiguatePose(
                this IShape shape,
                Vector3d centroid,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                IReadOnlyList<double> weights) {

            double size;
            
            if (!IsUnambiguable(
                    upAxis,
                    horizontalAxis)) {

                Console.WriteLine("Unambiguation of poses is currently only supported for axis being {(0,0,+-1), (0,+-1,0), (+-1,0,0)}.");
                return;
            }

            size = Vector3d.Dot(
                shape.BBox.Size,
                horizontalAxis);

            shape.OrientHorizontalAxisAlongLongerSideOfBBox(
                size,
                centroid,
                upAxis,
                horizontalAxis);

            shape.OrientPositiveHorizontalAxisToMoreDenseProximalSectionOfBBox(
                size,
                centroid,
                upAxis,
                horizontalAxis,
                weights);
        }

        private static bool IsUnambiguable(
                Vector3d upAxis,
                Vector3d horizontalAxis) {

            return ((horizontalAxis.X.Abs().ApproximateEquals(1.0)
                        && horizontalAxis.Y.Abs().ApproximateEquals(0.0)
                        && horizontalAxis.Z.Abs().ApproximateEquals(0.0))
                    || (horizontalAxis.X.Abs().ApproximateEquals(0.0)
                        && horizontalAxis.Y.Abs().ApproximateEquals(1.0)
                        && horizontalAxis.Z.Abs().ApproximateEquals(0.0))
                    || (horizontalAxis.X.Abs().ApproximateEquals(0.0)
                        && horizontalAxis.Y.Abs().ApproximateEquals(0.0)
                        && horizontalAxis.Z.Abs().ApproximateEquals(1.0)))
                || !((upAxis.X.Abs().ApproximateEquals(1.0)
                        && upAxis.Y.Abs().ApproximateEquals(0.0)
                        && upAxis.Z.Abs().ApproximateEquals(0.0))
                    || (upAxis.X.Abs().ApproximateEquals(0.0)
                        && upAxis.Y.Abs().ApproximateEquals(1.0)
                        && upAxis.Z.Abs().ApproximateEquals(0.0))
                    || (upAxis.X.Abs().ApproximateEquals(0.0)
                        && upAxis.Y.Abs().ApproximateEquals(0.0)
                        && upAxis.Z.Abs().ApproximateEquals(1.0)));
        }

        private static void OrientHorizontalAxisAlongLongerSideOfBBox(
                this IShape shape,
                double size,
                Vector3d centroid,
                Vector3d upAxis,
                Vector3d horizontalAxis) {

            if (size < Vector3d.Dot(
                    shape.BBox.Size,
                    Vector3d.Cross(upAxis, horizontalAxis))) {

                shape.Rotate(
                    new Rotation.Config(DEGREE_90, upAxis) {
                        UpdateBBox = true,
                        Anchor = centroid,
                        RotateNormals = shape.Type == ShapeType.POINT_CLOUD
                    });
            }
        }

        private static void OrientPositiveHorizontalAxisToMoreDenseProximalSectionOfBBox(
                this IShape shape,
                double size,
                Vector3d centroid,
                Vector3d upAxis,
                Vector3d horizontalAxis,
                IReadOnlyList<double> weights) {

            (double, double) densities = shape.GetDensityInProximalSectionsOfBBox(
                size,
                horizontalAxis,
                weights);

            if (densities.Item1 > densities.Item2) {
                shape.Rotate(
                    new Rotation.Config(DEGREE_180, upAxis) {
                        UpdateBBox = true,
                        Anchor = centroid,
                        RotateNormals = shape.Type == ShapeType.POINT_CLOUD
                    });
            }
        }

        private static (double, double) GetDensityInProximalSectionsOfBBox(
                this IShape shape,
                double size,
                Vector3d horizontalAxis,
                IReadOnlyList<double> weights) {

            object @lock = new object();
            (double, double) density = (0.0, 0.0);
            (double, double) bounds = (
                Vector3d.Dot(shape.BBox.Min, horizontalAxis) + UNAMBIGUOUS_ALIGN_SIZE_FRACTION * size,
                Vector3d.Dot(shape.BBox.Max, horizontalAxis) - UNAMBIGUOUS_ALIGN_SIZE_FRACTION * size);

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    shape.Points.Count),
                () => (0.0, 0.0),
                (partition, loopState, localDensity) => {

                    double positionAlongHorizontalAxis;

                    for (int i = partition.Item1; i < partition.Item2; i++) {

                        positionAlongHorizontalAxis = Vector3d.Dot(
                            shape.Points[i].Position,
                            horizontalAxis);

                        if (positionAlongHorizontalAxis <= bounds.Item1) {
                            localDensity.Item1 += weights[i];
                        }
                        else if (positionAlongHorizontalAxis >= bounds.Item2) {
                            localDensity.Item2 += weights[i];
                        }
                    }

                    return localDensity;

                },
                localDensity => {

                    lock (@lock) {
                        density.Item1 += localDensity.Item1;
                        density.Item2 += localDensity.Item2;
                    }

                });

            return density;
        }
    }
}