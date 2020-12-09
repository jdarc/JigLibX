using System.Collections.Generic;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision
{
    public class CollisionInfo
    {
        public const int MaxCollisionPoints = 10;

        public MaterialPairProperties MatPairProperties;

        public CollDetectInfo SkinInfo;

        public const int InitialCollisionInfoStack = 64;

        public const int InitialCollisionPointInfoStack = 4096;

        private static Stack<CollisionInfo> freeInfos = new Stack<CollisionInfo>(InitialCollisionInfoStack);
        private static Stack<CollPointInfo> freePtInfos = new Stack<CollPointInfo>(InitialCollisionPointInfoStack);

        private CollisionInfo()
        {
        }

        static CollisionInfo()
        {
            for (var i = 0; i < InitialCollisionInfoStack; ++i) freeInfos.Push(new CollisionInfo());
            for (var i = 0; i < InitialCollisionPointInfoStack; ++i) freePtInfos.Push(new CollPointInfo());
        }

        public bool Satisfied { get; set; }

        public Vector3 DirToBody0 { get; set; }

        public readonly CollPointInfo[] PointInfo = new CollPointInfo[MaxCollisionPoints];

        public int NumCollPts;

        private unsafe void Init(CollDetectInfo info, Vector3 dirToBody0, SmallCollPointInfo* pointInfos, int numPointInfos)
        {
            SkinInfo = info;
            DirToBody0 = dirToBody0;

            var ID0 = info.Skin0.GetMaterialID(info.IndexPrim0);
            var ID1 = info.Skin1.GetMaterialID(info.IndexPrim1);

            var matTable = info.Skin0.CollisionSystem.MaterialTable;

            if (ID0 == (int) MaterialTable.MaterialID.UserDefined || ID1 == (int) MaterialTable.MaterialID.UserDefined)
            {
                MaterialProperties prop0;

                prop0 = ID0 == (int) MaterialTable.MaterialID.UserDefined ? info.Skin0.GetMaterialProperties(info.IndexPrim0) : matTable.GetMaterialProperties(ID0);
                var prop1 = ID1 == (int) MaterialTable.MaterialID.UserDefined ? info.Skin1.GetMaterialProperties(info.IndexPrim1) : matTable.GetMaterialProperties(ID1);

                MatPairProperties.Restitution = prop0.Elasticity * prop1.Elasticity;
                MatPairProperties.StaticFriction = prop0.StaticRoughness * prop1.StaticRoughness;
                MatPairProperties.DynamicFriction = prop0.DynamicRoughness * prop1.DynamicRoughness;
            }
            else
            {
                MatPairProperties = matTable.GetPairProperties(ID0, ID1);
            }

            numPointInfos = numPointInfos > MaxCollisionPoints ? MaxCollisionPoints : numPointInfos;

            NumCollPts = 0;
            for (var i = 0; i < numPointInfos; ++i)
            {
                if (freePtInfos.Count == 0) freePtInfos.Push(new CollPointInfo());
                PointInfo[NumCollPts] = freePtInfos.Pop();
                PointInfo[NumCollPts++].Init(ref pointInfos[i]);
            }
        }

        private void Destroy()
        {
            for (var i = 0; i < NumCollPts; ++i) freePtInfos.Push(PointInfo[i]);
            SkinInfo.Skin0 = null;
            SkinInfo.Skin1 = null;
        }

        public static unsafe CollisionInfo GetCollisionInfo(CollDetectInfo info, Vector3 dirToBody0, SmallCollPointInfo* pointInfos, int numCollPts)
        {
            if (freeInfos.Count == 0) freeInfos.Push(new CollisionInfo());

            var collInfo = freeInfos.Pop();
            collInfo.Init(info, dirToBody0, pointInfos, numCollPts);

            return collInfo;
        }

        public static void FreeCollisionInfo(CollisionInfo info)
        {
            info.Destroy();
            freeInfos.Push(info);
        }
    }
}