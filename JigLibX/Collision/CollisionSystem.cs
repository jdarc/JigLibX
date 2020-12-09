using System.Collections.Generic;
using Microsoft.Xna.Framework;
using JigLibX.Physics;
using JigLibX.Geometry;
using System.Collections.ObjectModel;
using JigLibX.Collision.Detection;

namespace JigLibX.Collision
{
    public abstract class CollisionSystem
    {
        private readonly Dictionary<int, DetectFunctor> detectionFunctors = new Dictionary<int, DetectFunctor>();

        private static readonly CollDetectBoxBox boxBoxCollDetector = new CollDetectBoxBox();
        private static readonly CollDetectBoxStaticMesh boxStaticMeshCollDetector = new CollDetectBoxStaticMesh();
        private static readonly CollDetectCapsuleBox capsuleBoxCollDetector = new CollDetectCapsuleBox();
        private static readonly CollDetectCapsuleCapsule capsuleCapsuleCollDetector = new CollDetectCapsuleCapsule();
        private static readonly CollDetectSphereCapsule sphereCapsuleCollDetector = new CollDetectSphereCapsule();
        private static readonly CollDetectSphereBox sphereBoxCollDetector = new CollDetectSphereBox();
        private static readonly CollDetectSphereSphere sphereSphereCollDetector = new CollDetectSphereSphere();
        private static readonly CollDetectBoxHeightmap boxHeightmapCollDetector = new CollDetectBoxHeightmap();
        private static readonly CollDetectSphereHeightmap sphereHeightmapCollDetector = new CollDetectSphereHeightmap();
        private static readonly CollDetectCapsuleHeightmap capsuleHeightmapCollDetector = new CollDetectCapsuleHeightmap();
        private static readonly CollDetectSphereStaticMesh sphereStaticMeshCollDetector = new CollDetectSphereStaticMesh();
        private static readonly CollDetectCapsuleStaticMesh capsuleStaticMeshCollDetector = new CollDetectCapsuleStaticMesh();
        private static readonly CollDetectBoxPlane boxPlaneCollDetector = new CollDetectBoxPlane();
        private static readonly CollDetectSpherePlane spherePlaneCollDetector = new CollDetectSpherePlane();
        private static readonly CollDetectCapsulePlane capsulePlaneCollDetector = new CollDetectCapsulePlane();

        public CollisionSystem()
        {
            RegisterCollDetectFunctor(boxBoxCollDetector);
            RegisterCollDetectFunctor(boxStaticMeshCollDetector);
            RegisterCollDetectFunctor(capsuleBoxCollDetector);
            RegisterCollDetectFunctor(capsuleCapsuleCollDetector);
            RegisterCollDetectFunctor(sphereBoxCollDetector);
            RegisterCollDetectFunctor(sphereSphereCollDetector);
            RegisterCollDetectFunctor(sphereCapsuleCollDetector);
            RegisterCollDetectFunctor(boxHeightmapCollDetector);
            RegisterCollDetectFunctor(sphereHeightmapCollDetector);
            RegisterCollDetectFunctor(capsuleHeightmapCollDetector);
            RegisterCollDetectFunctor(sphereStaticMeshCollDetector);
            RegisterCollDetectFunctor(capsuleStaticMeshCollDetector);
            RegisterCollDetectFunctor(boxPlaneCollDetector);
            RegisterCollDetectFunctor(spherePlaneCollDetector);
            RegisterCollDetectFunctor(capsulePlaneCollDetector);
        }

        public abstract void AddCollisionSkin(CollisionSkin collisionSkin);

        public abstract bool RemoveCollisionSkin(CollisionSkin collisionSkin);

        public abstract ReadOnlyCollection<CollisionSkin> CollisionSkins { get; }

        public abstract void CollisionSkinMoved(CollisionSkin skin);

        public abstract void DetectCollisions(Body body, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance);

        public abstract void DetectAllCollisions(List<Body> bodies, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance);

        public void RegisterCollDetectFunctor(DetectFunctor f)
        {
            var key01 = (f.Type0 << 16) | f.Type1;
            var key10 = (f.Type1 << 16) | f.Type0;

            if (!detectionFunctors.ContainsKey(key01)) detectionFunctors.Add(key01, f);

            if (!detectionFunctors.ContainsKey(key10)) detectionFunctors.Add(key10, f);
        }

        public DetectFunctor GetCollDetectFunctor(int type0, int type1)
        {
            var key01 = (type0 << 16) | type1;
            if (detectionFunctors.TryGetValue(key01, out var functor))
                return functor;
            else
                return null;
        }

        public abstract bool SegmentIntersect(out float fracOut, out CollisionSkin skinOut, out Vector3 posOut, out Vector3 normalOut, Segment seg, CollisionSkinPredicate1 collisionPredicate);

        public bool UseSweepTests { get; set; }

        public MaterialTable MaterialTable { get; } = new MaterialTable();
    }
}