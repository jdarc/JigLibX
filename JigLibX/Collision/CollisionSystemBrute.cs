using System.Collections.Generic;
using JigLibX.Physics;
using JigLibX.Geometry;
using Microsoft.Xna.Framework;
using System.Collections.ObjectModel;
using JigLibX.Collision.Detection;
using JigLibX.Geometry.Primitives;

namespace JigLibX.Collision
{
    public class CollisionSystemBrute : CollisionSystem
    {
        private List<CollisionSkin> skins = new List<CollisionSkin>();

        public CollisionSystemBrute()
        {
        }

        private static bool CheckCollidables(CollisionSkin skin0, CollisionSkin skin1)
        {
            var nonColl0 = skin0.NonCollidables;
            var nonColl1 = skin1.NonCollidables;


            if (nonColl0.Count == 0 && nonColl1.Count == 0) return true;

            for (var i0 = nonColl0.Count; i0-- != 0;)
                if (nonColl0[i0] == skin1)
                    return false;

            for (var i1 = nonColl1.Count; i1-- != 0;)
                if (nonColl1[i1] == skin0)
                    return false;

            return true;
        }

        public override ReadOnlyCollection<CollisionSkin> CollisionSkins => skins.AsReadOnly();

        public override void AddCollisionSkin(CollisionSkin skin)
        {
            if (skins.Contains(skin))
                System.Diagnostics.Debug.WriteLine("Warning: tried to add skin to CollisionSystemBrute but it's already registered");
            else
                skins.Add(skin);

            skin.CollisionSystem = this;
        }

        public override bool RemoveCollisionSkin(CollisionSkin skin)
        {
            if (!skins.Contains(skin)) return false;
            skins.Remove(skin);
            return true;
        }

        public override void CollisionSkinMoved(CollisionSkin skin)
        {
        }

        public override void DetectCollisions(Body body, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance)
        {
            if (!body.IsActive) return;

            var info = new CollDetectInfo();
            info.Skin0 = body.CollisionSkin;
            if (info.Skin0 == null) return;

            var bodyPrimitves = info.Skin0.NumPrimitives;
            var numSkins = skins.Count;

            for (var skin = 0; skin < numSkins; ++skin)
            {
                info.Skin1 = skins[skin];
                if (info.Skin0 != info.Skin1 && CheckCollidables(info.Skin0, info.Skin1))
                {
                    var primitives = info.Skin1.NumPrimitives;

                    for (info.IndexPrim0 = 0; info.IndexPrim0 < bodyPrimitves; ++info.IndexPrim0)
                    for (info.IndexPrim1 = 0; info.IndexPrim1 < primitives; ++info.IndexPrim1)
                    {
                        var f = GetCollDetectFunctor(info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0).Type, info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1).Type);

                        f?.CollDetect(info, collTolerance, collisionFunctor);
                    }
                }
            }
        }

        public override void DetectAllCollisions(List<Body> bodies, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance)
        {
            var numSkins = skins.Count;
            var numBodies = bodies.Count;

            var info = new CollDetectInfo();

            for (var ibody = 0; ibody < numBodies; ++ibody)
            {
                var body = bodies[ibody];
                if (!body.IsActive) continue;

                info.Skin0 = body.CollisionSkin;
                if (info.Skin0 == null) continue;

                for (var skin = 0; skin < numSkins; ++skin)
                {
                    info.Skin1 = skins[skin];
                    if (info.Skin0 == info.Skin1) continue;


                    if (info.Skin1 == null) continue;

                    var skinSleeping = true;

                    if (info.Skin1.Owner != null && info.Skin1.Owner.IsActive) skinSleeping = false;

                    if (skinSleeping == false && info.Skin1.ID < info.Skin0.ID) continue;

                    if (collisionPredicate != null && collisionPredicate.ConsiderSkinPair(info.Skin0, info.Skin1) == false) continue;


                    if (BoundingBoxHelper.OverlapTest(ref info.Skin0.WorldBoundingBox, ref info.Skin1.WorldBoundingBox, collTolerance))
                        if (CheckCollidables(info.Skin0, info.Skin1))
                        {
                            var bodyPrimitives = info.Skin0.NumPrimitives;
                            var primitves = info.Skin1.NumPrimitives;

                            for (info.IndexPrim0 = 0; info.IndexPrim0 < bodyPrimitives; ++info.IndexPrim0)
                            for (info.IndexPrim1 = 0; info.IndexPrim1 < primitves; ++info.IndexPrim1)
                            {
                                var f = GetCollDetectFunctor(info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0).Type, info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1).Type);
                                f?.CollDetect(info, collTolerance, collisionFunctor);
                            }
                        }
                }
            }
        }

        public override bool SegmentIntersect(out float fracOut, out CollisionSkin skinOut, out Vector3 posOut, out Vector3 normalOut, Segment seg, CollisionSkinPredicate1 collisionPredicate)
        {
            var numSkins = skins.Count;
            var segBox = BoundingBoxHelper.InitialBox;
            BoundingBoxHelper.AddSegment(seg, ref segBox);


            fracOut = float.MaxValue;
            skinOut = null;
            posOut = normalOut = Vector3.Zero;


            for (var iskin = 0; iskin < numSkins; ++iskin)
            {
                var skin = skins[iskin];
                if (collisionPredicate == null || collisionPredicate.ConsiderSkin(skin))

                    if (BoundingBoxHelper.OverlapTest(ref skin.WorldBoundingBox, ref segBox))
                        if (skin.SegmentIntersect(out var frac, out var pos, out var normal, seg))
                            if (frac < fracOut)
                            {
                                posOut = pos;
                                normalOut = normal;
                                skinOut = skin;
                                fracOut = frac;
                            }
            }

            if (fracOut > 1.0f) return false;
            fracOut = MathHelper.Clamp(fracOut, 0.0f, 1.0f);
            return true;
        }
    }
}