using Microsoft.Xna.Framework;

namespace JigLibX.Math
{
    public struct TransformRate
    {
        public Vector3 Velocity;

        public Vector3 AngularVelocity;

        public TransformRate(Vector3 velocity, Vector3 angularVelocity)
        {
            Velocity = velocity;
            AngularVelocity = angularVelocity;
        }

        public static TransformRate Zero => new TransformRate();

        public static TransformRate Add(TransformRate rate1, TransformRate rate2)
        {
            var result = new TransformRate();
            Vector3.Add(ref rate1.Velocity, ref rate2.Velocity, out result.Velocity);
            Vector3.Add(ref rate1.AngularVelocity, ref rate2.AngularVelocity, out result.AngularVelocity);
            return result;
        }

        public static void Add(ref TransformRate rate1, ref TransformRate rate2, out TransformRate result)
        {
            Vector3.Add(ref rate1.Velocity, ref rate2.Velocity, out result.Velocity);
            Vector3.Add(ref rate1.AngularVelocity, ref rate2.AngularVelocity, out result.AngularVelocity);
        }
    }
}