using Microsoft.Xna.Framework;

namespace JigLibX.Collision
{
    public struct SmallCollPointInfo
    {
        public float InitialPenetration;

        public Vector3 R0;

        public Vector3 R1;

        public SmallCollPointInfo(ref Vector3 R0, ref Vector3 R1, float initialPenetration)
        {
            this.R0 = R0;
            this.R1 = R1;
            InitialPenetration = initialPenetration;
        }

        public SmallCollPointInfo(Vector3 R0, Vector3 R1, float initialPenetration)
        {
            this.R0 = R0;
            this.R1 = R1;
            InitialPenetration = initialPenetration;
        }
    }
}