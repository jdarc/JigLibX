using System.Collections.Generic;
using Microsoft.Xna.Framework;
using JigLibX.Physics;
using JigLibX.Geometry;
using JigLibX.Math;
using System.Collections.ObjectModel;
using JigLibX.Geometry.Primitives;

namespace JigLibX.Collision
{
    public class CollisionSystemGrid : CollisionSystem
    {
        private List<GridEntry> gridEntries;
        private List<CollisionSkin> skins = new List<CollisionSkin>();
        private List<AABox> gridBoxes;
        private List<GridEntry> tempGridLists;

        private GridEntry overflowEntries;

        private int nx, ny, nz;
        private float dx, dy, dz;
        private float sizeX, sizeY, sizeZ;

        private float minDelta;

        private Stack<GridEntry> freeGrids;

        public CollisionSystemGrid(int nx, int ny, int nz, float dx, float dy, float dz)
        {
            this.nx = nx;
            this.ny = ny;
            this.nz = nz;
            this.dx = dx;
            this.dy = dy;
            this.dz = dz;

            sizeX = nx * dx;
            sizeY = ny * dy;
            sizeZ = nz * dz;
            minDelta = System.Math.Min(System.Math.Min(dx, dy), dz);

            var numEntries = nx * ny * nz * 2;
            gridEntries = new List<GridEntry>(numEntries);
            gridBoxes = new List<AABox>(numEntries);

            tempGridLists = new List<GridEntry>(numEntries);

            freeGrids = new Stack<GridEntry>(numEntries);
            for (var i = 0; i < numEntries; ++i)
            {
                var entry = new GridEntry();
                entry.GridIndex = -2;
                freeGrids.Push(entry);
            }

            for (var i = 0; i < nx * ny * nz; ++i)
            {
                var gridEntry = freeGrids.Pop();
                gridEntry.GridIndex = i;
                gridEntries.Add(gridEntry);
                gridBoxes.Add(null);
            }

            overflowEntries = freeGrids.Pop();
            overflowEntries.GridIndex = -1;

            for (var iX = 0; iX < nx; ++iX)
            for (var iY = 0; iY < ny; ++iY)
            for (var iZ = 0; iZ < nz; ++iZ)
            {
                var box = new AABox();
                box.AddPoint(new Vector3(iX * dx, iY * dy, iZ + dz));
                box.AddPoint(new Vector3(iX * dx + dx, iY * dy + dy, iZ * dz + dz));

                var index = CalcIndex(iX, iY, iZ);
                gridBoxes[index] = box;
            }
        }

        public override ReadOnlyCollection<CollisionSkin> CollisionSkins => skins.AsReadOnly();

        private int CalcIndex(int i, int j, int k) => i % nx + nx * (j % ny) + (nx + ny) * (k % nz);

        private void CalcGridForSkin(out int i, out int j, out int k, CollisionSkin skin)
        {
            var sides = skin.WorldBoundingBox.Max - skin.WorldBoundingBox.Min;
            if (sides.X > dx || sides.Y > dy || sides.Z > dz)
            {
                System.Diagnostics.Debug.WriteLine($"CollisionSkin too big for gridding system - putting it into overflow list.");
                i = j = k = -1;
                return;
            }

            var min = skin.WorldBoundingBox.Min;

            min.X = JiggleMath.Wrap(min.X, 0.0f, sizeX);
            min.Y = JiggleMath.Wrap(min.Y, 0.0f, sizeY);
            min.Z = JiggleMath.Wrap(min.Z, 0.0f, sizeZ);

            i = (int) (min.X / dx) % nx;
            j = (int) (min.Y / dy) % ny;
            k = (int) (min.Z / dz) % nz;
        }

        public void CalcGridForSkin(out int i, out int j, out int k, out float fi, out float fj, out float fk, CollisionSkin skin)
        {
            var sides = skin.WorldBoundingBox.Max - skin.WorldBoundingBox.Min;
            if (sides.X > dx || sides.Y > dy || sides.Z > dz)
            {
                System.Diagnostics.Debug.WriteLine("CollisionSkin too big for gridding system - putting it into overflow list.");

                i = j = k = -1;
                fi = fj = fk = 0.0f;
                return;
            }

            var min = skin.WorldBoundingBox.Min;

            min.X = JiggleMath.Wrap(min.X, 0.0f, sizeX);
            min.Y = JiggleMath.Wrap(min.Y, 0.0f, sizeY);
            min.Z = JiggleMath.Wrap(min.Z, 0.0f, sizeZ);

            fi = min.X / dx;
            fj = min.Y / dy;
            fk = min.Z / dz;

            i = (int) fi;
            j = (int) fj;
            k = (int) fk;

            if (i < 0)
            {
                i = 0;
                fi = 0.0f;
            }
            else if (i >= nx)
            {
                i = 0;
                fi = 0.0f;
            }
            else
            {
                fi -= i;
            }

            if (j < 0)
            {
                j = 0;
                fj = 0.0f;
            }
            else if (j >= ny)
            {
                j = 0;
                fj = 0.0f;
            }
            else
            {
                fj -= j;
            }

            if (k < 0)
            {
                k = 0;
                fk = 0.0f;
            }
            else if (k >= nz)
            {
                k = 0;
                fk = 0.0f;
            }
            else
            {
                fk -= k;
            }
        }

        private int CalcGridIndexForSkin(CollisionSkin skin)
        {
            CalcGridForSkin(out var i, out var j, out var k, skin);
            if (i == -1) return -1;
            return CalcIndex(i, j, k);
        }

        public override void AddCollisionSkin(CollisionSkin skin)
        {
            if (skins.Contains(skin))
                System.Diagnostics.Debug.WriteLine("Warning: tried to add skin to CollisionSkinGrid but it's already registered");
            else
                skins.Add(skin);

            skin.CollisionSystem = this;

            if (freeGrids.Count == 0) freeGrids.Push(new GridEntry());


            var entry = freeGrids.Pop();
            skin.ExternalData = entry;
            entry.Skin = skin;

            GridEntry.InsertGridEntryAfter(entry, overflowEntries);
            CollisionSkinMoved(skin);
        }

        public override bool RemoveCollisionSkin(CollisionSkin skin)
        {
            var entry = (GridEntry) skin.ExternalData;

            if (entry != null)
            {
                entry.Skin = null;
                freeGrids.Push(entry);
                GridEntry.RemoveGridEntry(entry);
                skin.ExternalData = null;
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("Warning - skin being deleted without a grid entry");
            }

            if (!skins.Contains(skin)) return false;
            skins.Remove(skin);
            return true;
        }

        public override void CollisionSkinMoved(CollisionSkin skin)
        {
            var entry = (GridEntry) skin.ExternalData;
            if (entry == null)
            {
                System.Diagnostics.Debug.WriteLine("Warning skin has grid entry null!");
                return;
            }

            var gridIndex = CalcGridIndexForSkin(skin);


            if (gridIndex == entry.GridIndex) return;

            GridEntry start;
            if (gridIndex >= 0)
                start = gridEntries[gridIndex];
            else
                start = overflowEntries;

            GridEntry.RemoveGridEntry(entry);
            GridEntry.InsertGridEntryAfter(entry, start);
        }

        private void GetListsToCheck(List<GridEntry> entries, CollisionSkin skin)
        {
            entries.Clear();

            var entry = (GridEntry) skin.ExternalData;
            if (entry == null)
            {
                System.Diagnostics.Debug.WriteLine("Warning skin has grid entry null!");

                return;
            }


            CalcGridForSkin(out var i, out var j, out var k, out var fi, out var fj, out var fk, skin);

            if (i == -1)
            {
                for (i = 0; i < gridEntries.Count; ++i)
                    if (gridEntries[i].Next != null)
                        entries.Add(gridEntries[i]);

                entries.Add(overflowEntries);
                return;
            }


            entries.Add(overflowEntries);

            var delta = skin.WorldBoundingBox.Max - skin.WorldBoundingBox.Min;
            int maxI = 1, maxJ = 1, maxK = 1;
            if (fi + delta.X / dx < 1.0f) maxI = 0;
            if (fj + delta.Y / dy < 1.0f) maxJ = 0;
            if (fk + delta.Z / dz < 1.0f) maxK = 0;


            for (var di = -1; di <= maxI; ++di)
            for (var dj = -1; dj <= maxJ; ++dj)
            for (var dk = -1; dk <= maxK; ++dk)
            {
                var thisIndex = CalcIndex(nx + i + di, ny + j + dj, nz + k + dk);
                var start = gridEntries[thisIndex];
                if (start.Next != null) entries.Add(start);
            }
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

        public override void DetectCollisions(Body body, CollisionFunctor collisionFunctor, CollisionSkinPredicate2 collisionPredicate, float collTolerance)
        {
            if (!body.IsActive) return;

            var info = new CollDetectInfo();

            info.Skin0 = body.CollisionSkin;
            if (info.Skin0 == null) return;

            var bodyPrimitives = info.Skin0.NumPrimitives;
            var numSkins = skins.Count;

            for (var skin = 0; skin < numSkins; ++skin)
            {
                info.Skin1 = skins[skin];
                if (info.Skin0 != info.Skin1 && CheckCollidables(info.Skin0, info.Skin1))
                {
                    var primitives = info.Skin1.NumPrimitives;

                    for (info.IndexPrim0 = 0; info.IndexPrim0 < bodyPrimitives; ++info.IndexPrim0)
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
            var numBodies = bodies.Count;

            var info = new CollDetectInfo();

            for (var iBody = 0; iBody < numBodies; ++iBody)
            {
                var body = bodies[iBody];
                if (!body.IsActive) continue;

                info.Skin0 = body.CollisionSkin;
                if (info.Skin0 == null) continue;

                tempGridLists.Clear();
                GetListsToCheck(tempGridLists, info.Skin0);

                for (var iList = tempGridLists.Count; iList-- != 0;)
                {
                    var entry = tempGridLists[iList];
                    for (entry = entry.Next; entry != null; entry = entry.Next)
                    {
                        info.Skin1 = entry.Skin;
                        if (info.Skin1 == info.Skin0) continue;


                        if (info.Skin1 == null) continue;

                        var skinSleeping = true;

                        if (info.Skin1.Owner != null && info.Skin1.Owner.IsActive) skinSleeping = false;


                        if (skinSleeping == false && info.Skin1.ID < info.Skin0.ID) continue;

                        if (collisionPredicate != null && !collisionPredicate.ConsiderSkinPair(info.Skin0, info.Skin1))
                            continue;


                        if (BoundingBoxHelper.OverlapTest(ref info.Skin1.WorldBoundingBox, ref info.Skin0.WorldBoundingBox, collTolerance))
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