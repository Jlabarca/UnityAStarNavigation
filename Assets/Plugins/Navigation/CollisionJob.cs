using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace Plugins.Navigation
{
    [BurstCompile]
    public struct CollisionJob : IJobParallelFor
    {
        public NativeArray<Vector3> position;
        public NativeArray<Vector3> velocity;
        public NativeArray<float> collider;

        public void Execute(int id)
        {
            if (velocity[id] == Vector3.zero) return;
            var myPosition = position[id];
            var myCollider = collider[id];
            //var myVelocity = velocity[id];
            for (var i = 0; i < position.Length; i++)
            {
                if (i == id || !Intersect(myPosition, myCollider, position[i], collider[i])) continue;
                //velocity[i] = (myVelocity+velocity[i])/2;
                //velocity[i] = Vector3.zero;
                //position[i] = new Vector3(myPosition.x, 4, myPosition.z);
            }
        }

        private static bool Intersect(Vector3 position1, float colliderRadius1, Vector3 position2, float colliderRadius2, float threshold=0.1f){
            var distance = position1 - position2;
            var diff = colliderRadius1 + colliderRadius2 - distance.magnitude;
            return diff >= threshold;
        }
    }
}