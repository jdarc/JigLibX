using System.Collections.Generic;

namespace JigLibX.Collision
{
    public class MaterialTable
    {
        public enum MaterialID
        {
            Unset,

            UserDefined,

            NotBouncySmooth,

            NotBouncyNormal,

            NotBouncyRough,

            NormalSmooth,

            NormalNormal,

            NormalRough,

            BouncySmooth,

            BouncyNormal,

            BouncyRough,

            NumMaterialTypes
        }

        private Dictionary<int, MaterialProperties> materials = new Dictionary<int, MaterialProperties>();

        private Dictionary<int, MaterialPairProperties> materialPairs = new Dictionary<int, MaterialPairProperties>();

        public MaterialTable()
        {
            Reset();
        }

        public void Reset()
        {
            Clear();

            var normalBouncy = 0.3f;
            var normalRoughS = 0.5f;
            var normalRoughD = 0.3f;
            var roughRoughS = 0.5f;
            var roughRoughD = 0.3f;

            SetMaterialProperties((int) MaterialID.Unset, new MaterialProperties(0.0f, 0.0f, 0.0f));
            SetMaterialProperties((int) MaterialID.NotBouncySmooth, new MaterialProperties(0.0f, 0.0f, 0.0f));
            SetMaterialProperties((int) MaterialID.NotBouncyNormal, new MaterialProperties(0.0f, normalRoughS, normalRoughD));
            SetMaterialProperties((int) MaterialID.NotBouncyRough, new MaterialProperties(0.0f, roughRoughD, roughRoughD));
            SetMaterialProperties((int) MaterialID.NormalSmooth, new MaterialProperties(normalBouncy, 0.0f, 1.0f));
            SetMaterialProperties((int) MaterialID.NormalNormal, new MaterialProperties(normalBouncy, normalRoughS, normalRoughD));
            SetMaterialProperties((int) MaterialID.NormalRough, new MaterialProperties(normalBouncy, roughRoughS, roughRoughD));
            SetMaterialProperties((int) MaterialID.BouncySmooth, new MaterialProperties(1.0f, 0.0f, 0.0f));
            SetMaterialProperties((int) MaterialID.BouncyNormal, new MaterialProperties(1.0f, normalRoughS, normalRoughD));
            SetMaterialProperties((int) MaterialID.BouncyRough, new MaterialProperties(1.0f, roughRoughS, roughRoughD));
        }

        public void Clear()
        {
            materials.Clear();
            materialPairs.Clear();
        }

        public void SetMaterialProperties(int id, MaterialProperties properties)
        {
            materials[id] = properties;

            foreach (var it in materials)
            {
                var otherID = it.Key;
                var mat = it.Value;

                var key01 = (otherID << 16) | id;
                var key10 = (id << 16) | otherID;
                materialPairs[key01] = materialPairs[key10] = new MaterialPairProperties(properties.Elasticity * mat.Elasticity, properties.StaticRoughness * mat.StaticRoughness, properties.DynamicRoughness * mat.DynamicRoughness);
            }
        }

        public MaterialProperties GetMaterialProperties(int id)
        {
            return materials[id];
        }

        public MaterialPairProperties GetPairProperties(int id1, int id2)
        {
            var key = (id1 << 16) | id2;
            return materialPairs[key];
        }

        public void SetMaterialPairProperties(int id1, int id2, MaterialPairProperties pairProperties)
        {
            var key01 = (id1 << 16) | id2;
            var key10 = (id2 << 16) | id1;
            materialPairs[key01] = materialPairs[key10] = pairProperties;
        }
    }
}