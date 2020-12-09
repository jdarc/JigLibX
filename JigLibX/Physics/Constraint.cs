using Microsoft.Xna.Framework;

namespace JigLibX.Physics
{
    public abstract class Constraint
    {
        public void EnableConstraint()
        {
            if (PhysicsSystem.CurrentPhysicsSystem == null) return;
            if (IsConstraintEnabled) return;

            IsConstraintEnabled = true;
            PhysicsSystem.CurrentPhysicsSystem.AddConstraint(this);
        }

        public void DisableConstraint()
        {
            if (PhysicsSystem.CurrentPhysicsSystem == null) return;
            if (!IsConstraintEnabled) return;

            IsConstraintEnabled = false;
            PhysicsSystem.CurrentPhysicsSystem.RemoveConstraint(this);
        }

        public bool IsConstraintEnabled { get; private set; }

        public abstract void PreApply(float dt);

        public abstract bool Apply(float dt);

        public abstract void Destroy();

        public bool Satisfied { get; set; }

        public static void SmoothCD(ref Vector3 val, ref Vector3 valRate, float timeDelta, Vector3 to, float smoothTime)
        {
            if (smoothTime > 0.0f)
            {
                var omega = 2.0f / smoothTime;
                var x = omega * timeDelta;
                var exp = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);

                Vector3.Subtract(ref val, ref to, out var change);

                Vector3.Multiply(ref change, omega, out var temp);
                Vector3.Add(ref valRate, ref temp, out temp);
                Vector3.Multiply(ref temp, timeDelta, out temp);

                Vector3.Multiply(ref temp, omega, out var v1);
                Vector3.Subtract(ref valRate, ref v1, out v1);
                Vector3.Multiply(ref v1, exp, out valRate);

                Vector3.Add(ref change, ref temp, out val);
                Vector3.Multiply(ref val, exp, out val);
                Vector3.Add(ref val, ref to, out val);
            }
            else if (timeDelta > 0.0f)
            {
                Vector3.Subtract(ref to, ref val, out valRate);
                Vector3.Divide(ref valRate, timeDelta, out valRate);

                val = to;
            }
            else
            {
                val = to;
                valRate.X = valRate.Y = valRate.Z = 0.0f;
            }
        }
    }
}