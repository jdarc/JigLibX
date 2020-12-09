namespace JigLibX.Collision
{
    public struct MaterialProperties
    {
        public float Elasticity;

        public float StaticRoughness;

        public float DynamicRoughness;

        public MaterialProperties(float e, float sr, float dr)
        {
            Elasticity = e;
            StaticRoughness = sr;
            DynamicRoughness = dr;
        }

        public static MaterialProperties Unset => new MaterialProperties();
    }
}