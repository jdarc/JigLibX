using Microsoft.Xna.Framework;

namespace JigLibX.Collision
{
    public class CollPointInfo
    {
        public SmallCollPointInfo Info;

        public float MinSeparationVel;

        public float Denominator;

        public float AccumulatedNormalImpulse;

        public Vector3 AccumulatedFrictionImpulse;

        public float AccumulatedNormalImpulseAux;

        public Vector3 Position;

        public void Init(ref SmallCollPointInfo info)
        {
            Info = info;
            Denominator = 0.0f;
            AccumulatedNormalImpulse = 0.0f;
            AccumulatedNormalImpulseAux = 0.0f;
            AccumulatedFrictionImpulse = Vector3.Zero;
            Position = Vector3.Zero;
            MinSeparationVel = 0.0f;
        }
    }
}