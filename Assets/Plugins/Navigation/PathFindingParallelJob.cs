using Priority_Queue;
using Providers.Grid;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Vella.Common.Navigation;

namespace Plugins.Navigation
{
   [BurstCompile]
    public struct PathFindingParallelJob : IJobParallelFor
    {
        public struct PathFindingJobResult
        {
            public PathStatus PathStatus;
        }

        public NativePriorityQueue OpenQueue;
        public NativeArray3D<GridNode> Grid;
        public NativeArray<NativeAreaDefinition> Areas;
        public float4x4 TransformMatrix;
        public GridPoint StartPoint;
        public GridPoint EndPoint;
        [ReadOnly]
        public NativeList<GridNode> Path;
        [ReadOnly]
        public NativeList<Vector3> WorldPath;
        public ulong AllowedFlags;
        public int MaxPoints;

        [NativeDisableUnsafePtrRestriction]
        public unsafe PathFindingJobResult* ResultPtr;


        public unsafe void Execute(int index)
        {
            ref var result = ref ResultPtr;
            ref var start = ref Grid.AsRef(StartPoint.X, StartPoint.Y, StartPoint.Z);
            ref var end = ref Grid.AsRef(EndPoint.X, EndPoint.Y, EndPoint.Z);

            OpenQueue.Enqueue(start.GridPoint, 0);

            start.OpenId = index;

            //var eX = end.NavigableCenter.x;
            //var eY = end.NavigableCenter.y;
            //var eZ = end.NavigableCenter.z;

            var closest = start;

            var maxX = Grid.GetLength(0) - 1;
            var maxY = Grid.GetLength(1) - 1;
            var maxZ = Grid.GetLength(2) - 1;

            while (OpenQueue.Count > 0)
            {
                var g = OpenQueue.Dequeue();
                ref var current = ref Grid.AsRef(g.X, g.Y, g.Z);

                current.OpenId = index;
                current.ClosedId = index;

                if (current.GridPoint == EndPoint)
                {
                    result->PathStatus = PathStatus.Complete;
                    RetracePath(ref start, ref current);
                    return;
                }

                //var cX = current.NavigableCenter.x;
                //var cY = current.NavigableCenter.y;
                //var cZ = current.NavigableCenter.z;

                var currentPoint = current.GridPoint;

                var xMin = currentPoint.X - 1;
                var yMin = currentPoint.Y - 1;
                var zMin = currentPoint.Z - 1;
                var xMax = currentPoint.X + 1;
                var yMax = currentPoint.Y + 1;
                var zMax = currentPoint.Z + 1;
                if (xMin < 0) xMin = 0;
                if (yMin < 0) yMin = 0;
                if (zMin < 0) zMin = 0;
                if (xMax > maxX) xMax = maxX;
                if (yMax > maxY) yMax = maxY;
                if (zMax > maxZ) zMax = maxZ;

                for (var x = xMin; x <= xMax; x++)
                {
                    for (var y = yMin; y <= yMax; y++)
                    {
                        for (var z = zMin; z <= zMax; z++)
                        {
                            if (x == currentPoint.X && y == currentPoint.Y && z == currentPoint.Z)
                                continue;

                            ref var neighbor = ref Grid.AsRef(x, y, z);

                            if ((neighbor.Flags & AllowedFlags) == 0)
                                continue;

                            if (neighbor.ClosedId == index)
                                continue;

                            //if (Math.Abs(neighbor.Y-current.Y) > 0.8f)
                            //    continue;

                            var nX = neighbor.NavigableCenter.x;
                            var nY = neighbor.NavigableCenter.y;
                            var nZ = neighbor.NavigableCenter.z;

                            var distance = math.distance(current.NavigableCenter, neighbor.NavigableCenter); //GetDistance(cX, cY, cZ, nX, nY, nZ);

                            float areaModifier = 0;
                            for (var j = 0; j < Areas.Length; j++)
                            {
                                areaModifier += (current.Flags & Areas[j].FlagValue) != 0 ? Areas[j].Weight * distance : 0;
                            }

                            var newCostToNeighbour = current.GScore + distance - areaModifier;
                            var isOpen = neighbor.OpenId == index;

                            if (newCostToNeighbour < neighbor.GScore || !isOpen)
                            {
                                neighbor.GScore = newCostToNeighbour;
                                var h = math.distance(neighbor.NavigableCenter, end.NavigableCenter); //GetDistance(nX, nY, nZ, eX, eY, eZ);

                                neighbor.HScore = h;
                                neighbor.FScore = newCostToNeighbour + h;
                                neighbor.ParentPoint = current.GridPoint;

                                if (closest.HScore <= 0 || h < closest.HScore)
                                {
                                    closest = neighbor;
                                }

                                if (!isOpen)
                                {
                                    OpenQueue.Enqueue(neighbor.GridPoint, neighbor.FScore);
                                    neighbor.OpenId = index;
                                }
                            }
                        }
                    }
                }
            }

            if (closest == start) return;
            result->PathStatus = PathStatus.Partial;
            RetracePath(ref start, ref closest);
        }

        private float GetDistance(float aX, float aY, float aZ, float bX, float bY, float bZ)
        {
            var xD = aX - bX;
            var yD = aY - bY;
            var zD = aZ - bZ;
            return (xD < 0 ? -xD : xD) + (yD < 0 ? -yD : yD) + (zD < 0 ? -zD : zD);
        }

        private void RetracePath(ref GridNode startNode, ref GridNode endNode)
        {
            ref var currentNode = ref endNode;
            var i = 0;

            while (currentNode.GridPoint != startNode.GridPoint && i < MaxPoints)
            {
                Path.Add(currentNode);
                currentNode = ref Grid.AsRef(currentNode.ParentPoint.X, currentNode.ParentPoint.Y, currentNode.ParentPoint.Z);
                i++;
            }

            Path.Add(startNode);
            Reverse(ref Path, 0, Path.Length);

            if (TransformMatrix.Equals(float4x4.zero)) return;
            for (var j = 0; j < Path.Length; j++)
            {
                WorldPath.Add(math.transform(TransformMatrix, Path[j].NavigableCenter));
            }
        }

        private static void Reverse<T>(ref NativeList<T> array, int index, int length) where T : struct
        {
            if (!array.IsCreated || array.Length <= 0)
                return;

            var maxIndex = array.Length - 1;
            var i = index;
            var j = index + length - 1;

            if (i > maxIndex || j > maxIndex)
                return;

            while (i < j)
            {
                var temp = array[i];
                array[i] = array[j];
                array[j] = temp;
                i++;
                j--;
            }
        }


    }
}