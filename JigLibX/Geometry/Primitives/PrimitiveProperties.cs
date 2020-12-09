namespace JigLibX.Geometry.Primitives
{
    public struct PrimitiveProperties
    {
        public enum MassDistributionEnum
        {
            Solid,

            Shell
        }

        public enum MassTypeEnum
        {
            Mass,

            Density
        }

        public MassTypeEnum MassType;

        public MassDistributionEnum MassDistribution;

        public float MassOrDensity;

        public PrimitiveProperties(MassDistributionEnum massDistribution, MassTypeEnum massType, float massOrDensity)
        {
            MassDistribution = massDistribution;
            MassOrDensity = massOrDensity;
            MassType = massType;
        }
    }
}