using System.Collections.Generic;
using System.Collections.ObjectModel;
using JigLibX.Collision.Detection;
using Microsoft.Xna.Framework;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;

namespace JigLibX.Collision
{
    public class CollisionSystemSAP : CollisionSystem, IComparer<CollisionSkin>
    {
        private List<CollisionSkin> skins_ = new List<CollisionSkin>();
        private bool dirty_;
        private List<CollisionSkin> active_ = new List<CollisionSkin>();
        private List<Primitive> testing_ = new List<Primitive>();
        private List<Primitive> second_ = new List<Primitive>();

        public float LargestX { get; private set; }

        public CollisionSystemSAP()
        {
        }

        public override void AddCollisionSkin(CollisionSkin collisionSkin)
        {
            collisionSkin.CollisionSystem = this;
            skins_.Add(collisionSkin);
            dirty_ = true;
            var dx = collisionSkin.WorldBoundingBox.Max.X - collisionSkin.WorldBoundingBox.Min.X;
            if (dx > LargestX) LargestX = dx;
        }

        public override bool RemoveCollisionSkin(CollisionSkin collisionSkin)
        {
            var ix = skins_.IndexOf(collisionSkin);
            if (ix >= skins_.Count || ix < 0) return false;
            skins_.RemoveAt(ix);
            return true;
        }

        public override ReadOnlyCollection<CollisionSkin> CollisionSkins => skins_.AsReadOnly();

        public override void CollisionSkinMoved(CollisionSkin skin)
        {
            dirty_ = true;
        }

        private void Extract(Vector3 min, Vector3 max, List<CollisionSkin> skins)
        {
            if (skins_.Count == 0) return;
            MaybeSort();
            var i = bsearch(min.X - LargestX);
            var xMax = max.X;
            while (i < skins_.Count && skins_[i].WorldBoundingBox.Min.X < xMax)
            {
                if (skins_[i].WorldBoundingBox.Max.X > min.X) skins.Add(skins_[i]);
                ++i;
            }
        }

        private int bsearch(float x)
        {
            var top = skins_.Count;
            var bot = 0;
            while (top > bot)
            {
                var mid = (top + bot) >> 1;
                if (skins_[mid].WorldBoundingBox.Min.X >= x)
                {
#if DEBUG
                    System.Diagnostics.Debug.Assert(top > mid);
#endif
                    top = mid;
                }
                else
                {
#if DEBUG
                    System.Diagnostics.Debug.Assert(bot <= mid);
#endif
                    bot = mid + 1;
                }
            }

#if DEBUG
            System.Diagnostics.Debug.Assert(top >= 0 && top <= skins_.Count);
            System.Diagnostics.Debug.Assert(top == 0 || skins_[top - 1].WorldBoundingBox.Min.X < x);
            System.Diagnostics.Debug.Assert(top == skins_.Count || skins_[top].WorldBoundingBox.Min.X >= x);
#endif

            return top;
        }

        public override void DetectCollisions(Physics.Body body, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance)
        {
            if (!body.IsActive) return;

            var info = new CollDetectInfo();
            info.Skin0 = body.CollisionSkin;
            if (info.Skin0 == null) return;

            active_.Clear();
            testing_.Clear();
            Extract(info.Skin0.WorldBoundingBox.Min, info.Skin0.WorldBoundingBox.Max, active_);

            for (int j = 0, m = info.Skin0.NumPrimitives; j != m; ++j) testing_.Add(info.Skin0.GetPrimitiveNewWorld(j));

            var nBodyPrims = testing_.Count;

            for (int i = 0, n = active_.Count; i != n; ++i)
            {
                info.Skin1 = active_[i];
                if (info.Skin0 != info.Skin1 && (collisionPredicate == null || collisionPredicate.ConsiderSkinPair(info.Skin0, info.Skin1)))
                {
                    var nPrim1 = info.Skin1.NumPrimitives;
                    second_.Clear();
                    for (var k = 0; k != nPrim1; ++k) second_.Add(info.Skin1.GetPrimitiveNewWorld(k));
                    for (info.IndexPrim0 = 0; info.IndexPrim0 != nBodyPrims; ++info.IndexPrim0)
                    for (info.IndexPrim1 = 0; info.IndexPrim1 != nPrim1; ++info.IndexPrim1)
                    {
                        var f = GetCollDetectFunctor(info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0).Type, info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1).Type);
                        f?.CollDetect(info, collTolerance, collisionFunctor);
                    }
                }
            }
        }

        private SkinTester skinTester_ = new SkinTester();

        public override void DetectAllCollisions(List<Physics.Body> bodies, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance)
        {
            skinTester_.Set(this, collisionFunctor, collisionPredicate, collTolerance);

            MaybeSort();


            var nSkins = skins_.Count;
            active_.Clear();


            unsafe
            {
                for (var i = 0; i != nSkins; ++i) AddToActive(skins_[i], skinTester_);
            }
        }

        private class SkinTester : CollisionSkinPredicate2
        {
            private CollisionFunctor collisionFunctor_;
            private CollisionSkinPredicate2 collisionPredicate_;
            private float collTolerance_;
            private CollDetectInfo info_;
            private CollisionSystem sys_;

            internal SkinTester()
            {
            }

            internal void Set(CollisionSystem sys, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance)
            {
                sys_ = sys;
                collisionFunctor_ = collisionFunctor;
                collisionPredicate_ = collisionPredicate;
                if (collisionPredicate_ == null) collisionPredicate_ = this;
                collTolerance_ = collTolerance;
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

            internal unsafe void TestSkin(CollisionSkin b, CollisionSkin s)
            {
#if DEBUG
                System.Diagnostics.Debug.Assert(b.Owner != null);
                System.Diagnostics.Debug.Assert(b.Owner.IsActive);
#endif
                if (!collisionPredicate_.ConsiderSkinPair(b, s)) return;

                info_.Skin0 = b;
                info_.Skin1 = s;
                var nSkin0 = info_.Skin0.NumPrimitives;
                var nSkin1 = info_.Skin1.NumPrimitives;


                DetectFunctor detectFunctor;
                for (info_.IndexPrim0 = 0; info_.IndexPrim0 != nSkin0; ++info_.IndexPrim0)
                for (info_.IndexPrim1 = 0; info_.IndexPrim1 != nSkin1; ++info_.IndexPrim1)
                    if (CheckCollidables(info_.Skin0, info_.Skin1))
                    {
                        detectFunctor = sys_.GetCollDetectFunctor(info_.Skin0.GetPrimitiveNewWorld(info_.IndexPrim0).Type, info_.Skin1.GetPrimitiveNewWorld(info_.IndexPrim1).Type);

                        detectFunctor?.CollDetect(info_, collTolerance_, collisionFunctor_);
                    }
            }

            public override bool ConsiderSkinPair(CollisionSkin skin0, CollisionSkin skin1)
            {
                return true;
            }
        }

        private void AddToActive(CollisionSkin cs, SkinTester st)
        {
            var n = active_.Count;
            var xMin = cs.WorldBoundingBox.Min.X;
            var active = cs.Owner != null && cs.Owner.IsActive;

            unsafe
            {
                CollisionSkin asi;
                for (var i = 0; i != n;)
                {
                    asi = active_[i];
                    if (asi.WorldBoundingBox.Max.X < xMin)
                    {
                        --n;
                        active_.RemoveAt(i);
                    }
                    else
                    {
                        if (active)
                        {
                            if (!(cs.WorldBoundingBox.Min.Z >= asi.WorldBoundingBox.Max.Z || cs.WorldBoundingBox.Max.Z <= asi.WorldBoundingBox.Min.Z || cs.WorldBoundingBox.Min.Y >= asi.WorldBoundingBox.Max.Y || cs.WorldBoundingBox.Max.Y <= asi.WorldBoundingBox.Min.Y || cs.WorldBoundingBox.Max.X <= asi.WorldBoundingBox.Min.X)) st.TestSkin(cs, asi);
                        }
                        else if (active_[i].Owner != null && asi.Owner.IsActive && !(cs.WorldBoundingBox.Min.Z >= asi.WorldBoundingBox.Max.Z || cs.WorldBoundingBox.Max.Z <= asi.WorldBoundingBox.Min.Z || cs.WorldBoundingBox.Min.Y >= asi.WorldBoundingBox.Max.Y || cs.WorldBoundingBox.Max.Y <= asi.WorldBoundingBox.Min.Y || cs.WorldBoundingBox.Max.X <= asi.WorldBoundingBox.Min.X))
                        {
                            st.TestSkin(asi, cs);
                        }

                        ++i;
                    }
                }
            }

            active_.Add(cs);
        }

        public override bool SegmentIntersect(out float fracOut, out CollisionSkin skinOut, out Vector3 posOut, out Vector3 normalOut, Segment seg, CollisionSkinPredicate1 collisionPredicate)
        {
            fracOut = float.MaxValue;
            skinOut = null;
            posOut = normalOut = Vector3.Zero;

            var segmentBeginning = seg.Origin;
            var segmentEnd = seg.Origin + seg.Delta;

            var min = Vector3.Min(segmentBeginning, segmentEnd);
            var max = Vector3.Max(segmentBeginning, segmentEnd);

            active_.Clear();

            var box = new BoundingBox(min, max);
            Extract(min, max, active_);

            var distanceSquared = float.MaxValue;
            var nActive = active_.Count;
            for (var i = 0; i != nActive; ++i)
            {
                var skin = active_[i];
                if (collisionPredicate == null || collisionPredicate.ConsiderSkin(skin))
                    if (BoundingBoxHelper.OverlapTest(ref box, ref skin.WorldBoundingBox))
                        if (skin.SegmentIntersect(out var frac, out var pos, out var normal, seg))
                            if (frac >= 0)
                            {
                                var newDistanceSquared = Vector3.DistanceSquared(segmentBeginning, pos);
                                if (newDistanceSquared < distanceSquared)
                                {
                                    distanceSquared = newDistanceSquared;

                                    fracOut = frac;
                                    skinOut = skin;
                                    posOut = pos;
                                    normalOut = normal;
                                }
                            }
            }

            return fracOut <= 1;
        }

        private void MaybeSort()
        {
            if (dirty_)
            {
                skins_.Sort(this);
                dirty_ = false;
            }
        }

        public int Compare(CollisionSkin x, CollisionSkin y)
        {
            var f = x.WorldBoundingBox.Min.X - y.WorldBoundingBox.Min.X;
            return f < 0 ? -1 : f > 0 ? 1 : 0;
        }
    }
}