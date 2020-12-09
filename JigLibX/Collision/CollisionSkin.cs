using System;
using System.Collections.Generic;
using JigLibX.Physics;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision
{
    public class CollisionSkin
    {
        public BoundingBox WorldBoundingBox;

        private List<Primitive> primitivesOldWorld = new List<Primitive>();
        private List<Primitive> primitivesNewWorld = new List<Primitive>();
        private List<Primitive> primitivesLocal = new List<Primitive>();
        private List<int> materialIDs = new List<int>();

        private List<MaterialProperties> materialProperties = new List<MaterialProperties>();

        private Transform transformOld = Transform.Identity;
        private Transform transformNew = Transform.Identity;

        private static int idCounter;

        internal object ExternalData;
        internal int ID;

        public event CollisionCallbackFn callbackFn;

        public CollisionSkin()
        {
            ID = idCounter++;
            Owner = null;

            CollisionSystem = null;
        }

        public CollisionSkin(Body owner)
        {
            ID = idCounter++;
            Owner = owner;

            CollisionSystem = null;
        }

        public bool OnCollisionEvent(CollisionSkin skin0, CollisionSkin skin1)
        {
            if (callbackFn != null)
                return callbackFn(skin0, skin1);
            else
                return true;
        }

        public Body Owner { get; set; }

        private int AddPrimitive(Primitive prim, int matID, MaterialProperties matProps)
        {
            var newPrim = prim.Clone();

            if (newPrim == null) throw new ArgumentException("Not able to clone primitive!");

            materialIDs.Add(matID);
            materialProperties.Add(matProps);

            primitivesOldWorld.Add(prim.Clone());
            primitivesNewWorld.Add(prim.Clone());
            primitivesLocal.Add(newPrim);

            UpdateWorldBoundingBox();

            return materialIDs.Count - 1;
        }

        public int AddPrimitive(Primitive prim, int matID)
        {
            if (matID == (int) MaterialTable.MaterialID.UserDefined)
                throw new ArgumentException("matID can't be set to 'UserDefined'");

            return AddPrimitive(prim, matID, MaterialProperties.Unset);
        }

        public int AddPrimitive(Primitive prim, MaterialProperties matProps)
        {
            return AddPrimitive(prim, (int) MaterialTable.MaterialID.UserDefined, matProps);
        }

        public void RemoveAllPrimitives()
        {
            primitivesOldWorld.Clear();
            primitivesNewWorld.Clear();
            primitivesLocal.Clear();
            materialIDs.Clear();
            materialProperties.Clear();
        }

        public int NumPrimitives => primitivesLocal.Count;

        public Primitive GetPrimitiveLocal(int prim)
        {
            return primitivesLocal[prim];
        }

        public Primitive GetPrimitiveOldWorld(int prim)
        {
            return primitivesOldWorld[prim];
        }

        public Primitive GetPrimitiveNewWorld(int prim)
        {
            return primitivesNewWorld[prim];
        }

        public int GetMaterialID(int prim)
        {
            return materialIDs[prim];
        }

        public MaterialProperties GetMaterialProperties(int prim)
        {
            return materialProperties[prim];
        }

        public void SetMaterialProperties(int prim, MaterialProperties matProperties)
        {
            materialProperties[prim] = matProperties;
            materialIDs[prim] = (int) MaterialTable.MaterialID.UserDefined;
        }

        public float GetVolume()
        {
            var result = 0.0f;
            for (var prim = primitivesLocal.Count; prim-- != 0;) result += primitivesLocal[prim].GetVolume();
            return result;
        }

        public float GetSurfaceArea()
        {
            var result = 0.0f;
            for (var prim = primitivesLocal.Count; prim-- != 0;) result += primitivesLocal[prim].GetSurfaceArea();
            return result;
        }

        public void SetNewTransform(ref Transform transform)
        {
            transformNew = transform;
            Transform t;

            for (var prim = primitivesNewWorld.Count; prim-- != 0;)
            {
                t = primitivesLocal[prim].Transform;
                primitivesNewWorld[prim].Transform = transform * t;
            }

            UpdateWorldBoundingBox();

            CollisionSystem?.CollisionSkinMoved(this);
        }

        public void SetOldTransform(ref Transform transform)
        {
            transformOld = transform;
            Transform t;

            for (var prim = primitivesNewWorld.Count; prim-- != 0;)
            {
                t = primitivesLocal[prim].Transform;
                primitivesOldWorld[prim].Transform = transform * t;
            }

            UpdateWorldBoundingBox();

            CollisionSystem?.CollisionSkinMoved(this);
        }

        public void SetTransform(ref Transform transformOld, ref Transform transformNew)
        {
            this.transformOld = transformOld;
            this.transformNew = transformNew;

            for (var prim = primitivesNewWorld.Count; prim-- != 0;)
            {
                primitivesOldWorld[prim].Transform = transformOld * primitivesLocal[prim].Transform;
                primitivesNewWorld[prim].Transform = transformNew * primitivesLocal[prim].Transform;
            }

            UpdateWorldBoundingBox();

            CollisionSystem?.CollisionSkinMoved(this);
        }

        public void ApplyLocalTransform(Transform transform)
        {
            Transform t;
            for (var prim = primitivesNewWorld.Count; prim-- != 0;)
            {
                t = primitivesLocal[prim].Transform;
                primitivesLocal[prim].Transform = transform * t;
            }

            SetTransform(ref transformOld, ref transformNew);
        }

        public Vector3 OldPosition => transformOld.Position;

        public Vector3 NewPosition => transformNew.Position;

        public Matrix OldOrient => transformOld.Orientation;

        public Matrix NewOrient => transformNew.Orientation;

        public Transform OldTransform => transformOld;

        public Transform NewTransform => transformNew;

        public void UpdateWorldBoundingBox()
        {
            var temp = BoundingBoxHelper.InitialBox;

            for (var iold = primitivesOldWorld.Count; iold-- != 0;)
                BoundingBoxHelper.AddPrimitive(primitivesOldWorld[iold], ref temp);

            if (CollisionSystem != null && CollisionSystem.UseSweepTests)
                for (var inew = primitivesNewWorld.Count; inew-- != 0;)
                    BoundingBoxHelper.AddPrimitive(primitivesNewWorld[inew], ref temp);
            WorldBoundingBox = BoundingBoxHelper.InitialBox;
            BoundingBoxHelper.AddBBox(temp, ref WorldBoundingBox);
        }

        public List<CollisionInfo> Collisions { get; } = new List<CollisionInfo>(16);

        public List<CollisionSkin> NonCollidables { get; } = new List<CollisionSkin>();

        public CollisionSystem CollisionSystem { set; get; }

        public bool SegmentIntersect(out float frac, out Vector3 pos, out Vector3 normal, Segment seg)
        {
            var segEnd = seg.GetEnd();
            frac = float.MaxValue;

            var thisSegLenRelToOrig = 1.0f;
            var segCopy = seg;

            pos = normal = Vector3.Zero;

            for (var prim = primitivesNewWorld.Count; prim-- != 0;)
            {
                var newPosition = pos;

                if (primitivesNewWorld[prim].SegmentIntersect(out var thisFrac, out newPosition, out normal, segCopy))
                {
                    pos = newPosition;
                    frac = thisFrac * thisSegLenRelToOrig;
                    segCopy.Delta *= thisFrac;
                    thisSegLenRelToOrig *= frac;
                }
            }


            return frac <= 1.0f;
        }

        public void GetMassProperties(PrimitiveProperties primitiveProperties, out float mass, out Vector3 centerOfMass, out Matrix inertiaTensor, out Matrix inertiaTensorCoM)
        {
            mass = 0.0f;
            centerOfMass = Vector3.Zero;
            inertiaTensor = new Matrix();

            var totalWeighting = 0.0f;

            if (primitiveProperties.MassType == PrimitiveProperties.MassTypeEnum.Mass)
                for (var prim = primitivesLocal.Count; prim-- != 0;)
                    if (primitiveProperties.MassDistribution == PrimitiveProperties.MassDistributionEnum.Solid)
                        totalWeighting += primitivesLocal[prim].GetVolume();
                    else
                        totalWeighting += primitivesLocal[prim].GetSurfaceArea();

            for (var prim = primitivesLocal.Count; prim-- != 0;)
            {
                var primProperties = primitiveProperties;

                if (primitiveProperties.MassType == PrimitiveProperties.MassTypeEnum.Mass)
                {
                    var weighting = 0.0f;
                    if (primitiveProperties.MassDistribution == PrimitiveProperties.MassDistributionEnum.Solid)
                        weighting = primitivesLocal[prim].GetVolume();
                    else
                        weighting = primitivesLocal[prim].GetSurfaceArea();
                    primProperties.MassOrDensity *= weighting / totalWeighting;
                }

                primitivesLocal[prim].GetMassProperties(primProperties, out var m, out var com, out var it);

                mass += m;
                centerOfMass += m * com;
                inertiaTensor += it;
            }

            inertiaTensorCoM = Matrix.Identity;

            if (mass > 0.0f)
            {
                centerOfMass /= mass;


                inertiaTensorCoM.M11 = inertiaTensor.M11 - mass * (centerOfMass.Y * centerOfMass.Y + centerOfMass.Z * centerOfMass.Z);
                inertiaTensorCoM.M22 = inertiaTensor.M22 - mass * (centerOfMass.Z * centerOfMass.Z + centerOfMass.X * centerOfMass.X);
                inertiaTensorCoM.M33 = inertiaTensor.M33 - mass * (centerOfMass.X * centerOfMass.X + centerOfMass.Y * centerOfMass.Y);


                inertiaTensorCoM.M12 = inertiaTensorCoM.M21 = inertiaTensor.M12 + mass * centerOfMass.X * centerOfMass.Y;
                inertiaTensorCoM.M23 = inertiaTensorCoM.M32 = inertiaTensor.M23 + mass * centerOfMass.Y * centerOfMass.Z;
                inertiaTensorCoM.M31 = inertiaTensorCoM.M13 = inertiaTensor.M31 + mass * centerOfMass.Z * centerOfMass.X;
            }

            if (primitiveProperties.MassType == PrimitiveProperties.MassTypeEnum.Mass)
                mass = primitiveProperties.MassOrDensity;
        }

        public void GetMassProperties(PrimitiveProperties[] primitiveProperties, out float mass, out Vector3 centerOfMass, out Matrix inertiaTensor, out Matrix inertiaTensorCoM)
        {
            mass = 0.0f;
            centerOfMass = Vector3.Zero;
            inertiaTensor = Matrix.Identity;
            inertiaTensorCoM = Matrix.Identity;

            for (var prim = primitivesLocal.Count; prim-- != 0;)
            {
                primitivesLocal[prim].GetMassProperties(primitiveProperties[prim], out var m, out var com, out var it);

                mass += m;
                centerOfMass += m * com;
                inertiaTensor += it;
            }

            if (mass > 0.0f)
            {
                centerOfMass /= mass;


                inertiaTensorCoM.M11 = inertiaTensor.M11 - mass * (centerOfMass.Y * centerOfMass.Y + centerOfMass.Z * centerOfMass.Z);
                inertiaTensorCoM.M22 = inertiaTensor.M22 - mass * (centerOfMass.Z * centerOfMass.Z + centerOfMass.X * centerOfMass.X);
                inertiaTensorCoM.M33 = inertiaTensor.M33 - mass * (centerOfMass.X * centerOfMass.X + centerOfMass.Y * centerOfMass.Y);


                inertiaTensorCoM.M12 = inertiaTensorCoM.M21 = inertiaTensor.M12 + mass * centerOfMass.X * centerOfMass.Y;
                inertiaTensorCoM.M23 = inertiaTensorCoM.M32 = inertiaTensor.M23 + mass * centerOfMass.Y * centerOfMass.Z;
                inertiaTensorCoM.M31 = inertiaTensorCoM.M13 = inertiaTensor.M31 + mass * centerOfMass.Z * centerOfMass.X;
            }
        }
    }
}