using Microsoft.Xna.Framework;
using JigLibX.Math;

namespace JigLibX.Physics
{
    public class ConstraintPoint : Constraint
    {
        private const float mMaxVelMag = 20.0f;
        private const float minVelForProcessing = 0.01f;

        private Vector3 body0Pos;
        private Body body0;
        private Vector3 body1Pos;
        private Body body1;
        private float allowedDistance;
        private float timescale;

        private Vector3 worldPos;
        private Vector3 R0;
        private Vector3 R1;
        private Vector3 vrExtra;

        public ConstraintPoint()
        {
        }

        public ConstraintPoint(Body body0, Vector3 body0Pos, Body body1, Vector3 body1Pos, float allowedDistance, float timescale)
        {
            Initialise(body0, body0Pos, body1, body1Pos, allowedDistance, timescale);
        }

        public void Initialise(Body body0, Vector3 body0Pos, Body body1, Vector3 body1Pos, float allowedDistance, float timescale)
        {
            this.body0Pos = body0Pos;
            this.body1Pos = body1Pos;
            this.body0 = body0;
            this.body1 = body1;

            this.allowedDistance = allowedDistance;
            this.timescale = timescale;

            if (timescale < JiggleMath.Epsilon) timescale = JiggleMath.Epsilon;

            body0?.AddConstraint(this);
            body1?.AddConstraint(this);
        }

        public override void PreApply(float dt)
        {
            Satisfied = false;

            Vector3.TransformNormal(ref body0Pos, ref body0.Transform.Orientation, out R0);

            Vector3.TransformNormal(ref body1Pos, ref body1.Transform.Orientation, out R1);

            Vector3.Add(ref body0.Transform.Position, ref R0, out var worldPos0);

            Vector3.Add(ref body1.Transform.Position, ref R1, out var worldPos1);

            Vector3.Add(ref worldPos0, ref worldPos1, out worldPos);
            Vector3.Multiply(ref worldPos, 0.5f, out worldPos);


            Vector3.Subtract(ref worldPos0, ref worldPos1, out var deviation);

            var deviationAmount = deviation.Length();

            if (deviationAmount > allowedDistance)
                Vector3.Multiply(ref deviation, (deviationAmount - allowedDistance) / (deviationAmount * System.Math.Max(timescale, dt)), out vrExtra);
            else
                vrExtra = Vector3.Zero;
        }

        public override bool Apply(float dt)
        {
            Satisfied = true;

            var body0FrozenPre = !body0.IsActive;
            var body1FrozenPre = !body1.IsActive;


            Vector3.Cross(ref body0.TransformRate.AngularVelocity, ref R0, out var currentVel0);
            Vector3.Add(ref currentVel0, ref body0.TransformRate.Velocity, out currentVel0);

            Vector3.Cross(ref body1.TransformRate.AngularVelocity, ref R1, out var currentVel1);
            Vector3.Add(ref currentVel1, ref body1.TransformRate.Velocity, out currentVel1);


            Vector3.Add(ref vrExtra, ref currentVel0, out var Vr);
            Vector3.Subtract(ref Vr, ref currentVel1, out Vr);

            var normalVel = Vr.Length();

            if (normalVel < minVelForProcessing) return false;


            if (normalVel > mMaxVelMag)
            {
                Vector3.Multiply(ref Vr, mMaxVelMag / normalVel, out Vr);
                normalVel = mMaxVelMag;
            }

            Vector3.Divide(ref Vr, normalVel, out var N);

            var numerator = -normalVel;

            Vector3.Cross(ref R0, ref N, out var v1);
            Vector3.TransformNormal(ref v1, ref body0.worldInvInertia, out v1);
            Vector3.Cross(ref v1, ref R0, out v1);
            Vector3.Dot(ref N, ref v1, out var f1);
            Vector3.Cross(ref R1, ref N, out v1);
            Vector3.TransformNormal(ref v1, ref body1.worldInvInertia, out v1);
            Vector3.Cross(ref v1, ref R1, out v1);
            Vector3.Dot(ref N, ref v1, out var f2);

            var denominator = body0.InverseMass + body1.InverseMass + f1 + f2;

            if (denominator < JiggleMath.Epsilon) return false;

            Vector3.Multiply(ref N, numerator / denominator, out var normalImpulse);

            if (!body0.Immovable) body0.ApplyWorldImpulse(normalImpulse, worldPos);

            if (!body1.Immovable) body1.ApplyWorldImpulse(-normalImpulse, worldPos);

            body0.SetConstraintsAndCollisionsUnsatisfied();
            body1.SetConstraintsAndCollisionsUnsatisfied();

            Satisfied = true;

            return true;
        }

        public override void Destroy()
        {
            body0 = null;
            body1 = null;

            DisableConstraint();
        }
    }
}