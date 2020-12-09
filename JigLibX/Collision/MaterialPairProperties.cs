namespace JigLibX.Collision
{
    public struct MaterialPairProperties
    {
        public float Restitution;

        public float StaticFriction;

        public float DynamicFriction;

        public MaterialPairProperties(float r, float sf, float df)
        {
            Restitution = r;
            DynamicFriction = df;
            StaticFriction = sf;
        }
    }
}