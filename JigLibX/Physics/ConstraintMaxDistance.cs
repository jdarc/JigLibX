using Microsoft.Xna.Framework;
using JigLibX.Math;

namespace JigLibX.Physics
{
    public class ConstraintMaxDistance : Constraint
    {
        private const float maxVelMag = 20.0f;
        private const float minVelForProcessing = 0.01f;

        private Body body0;
        private Body body1;
        private Vector3 body0Pos;
        private Vector3 body1Pos;
        private float mMaxDistance;

        private Vector3 R0;
        private Vector3 R1;
        private Vector3 worldPos;
        private Vector3 currentRelPos0;

        public ConstraintMaxDistance()
        {
        }

        public ConstraintMaxDistance(Body body0, Vector3 body0Pos, Body body1, Vector3 body1Pos, float maxDistance)
        {
            Initialise(body0, body0Pos, body1, body1Pos, maxDistance);
        }

        public void Initialise(Body body0, Vector3 body0Pos, Body body1, Vector3 body1Pos, float maxDistance)
        {
            this.body0Pos = body0Pos;
            this.body1Pos = body1Pos;
            this.body0 = body0;
            this.body1 = body1;
            mMaxDistance = maxDistance;

            if (body0 != null) this.body0.AddConstraint(this);
            if (body1 != null) this.body1.AddConstraint(this);
        }

        public override void PreApply(float dt)
        {
            Satisfied = false;

            var body0Transform = body0.Transform;
            var body1Transform = body1.Transform;
            Vector3.TransformNormal(ref body0Pos, ref body0Transform.Orientation, out R0);
            Vector3.TransformNormal(ref body1Pos, ref body1Transform.Orientation, out R1);
            Vector3.Add(ref body0Transform.Position, ref R0, out var worldPos0);
            Vector3.Add(ref body1Transform.Position, ref R1, out var worldPos1);
            Vector3.Add(ref worldPos0, ref worldPos1, out worldPos);
            Vector3.Multiply(ref worldPos, 0.5f, out worldPos);
            Vector3.Subtract(ref worldPos0, ref worldPos1, out currentRelPos0);
        }

        public override bool Apply(float dt)
        {
            Satisfied = true;

            var body0FrozenPre = !body0.IsActive;
            var body1FrozenPre = !body1.IsActive;

            if (body0FrozenPre && body1FrozenPre) return false;

            var currentVel0 = body0.AngularVelocity;
            Vector3.Cross(ref currentVel0, ref R0, out currentVel0);
            var body0TransformRate = body0.TransformRate;
            Vector3.Add(ref body0TransformRate.Velocity, ref currentVel0, out currentVel0);

            var currentVel1 = body1.AngularVelocity;
            Vector3.Cross(ref currentVel1, ref R1, out currentVel1);
            var body1TransformRate = body1.TransformRate;
            Vector3.Add(ref body1TransformRate.Velocity, ref currentVel1, out currentVel1);


            Vector3.Subtract(ref currentVel0, ref currentVel1, out var predRelPos0);
            Vector3.Multiply(ref predRelPos0, dt, out predRelPos0);
            Vector3.Add(ref predRelPos0, ref currentRelPos0, out predRelPos0);


            var clampedRelPos0 = predRelPos0;

            var clampedRelPos0Mag = clampedRelPos0.Length();

            if (clampedRelPos0Mag <= JiggleMath.Epsilon) return false;

            if (clampedRelPos0Mag > mMaxDistance)
                Vector3.Multiply(ref clampedRelPos0, mMaxDistance / clampedRelPos0Mag, out clampedRelPos0);


            Vector3.Subtract(ref clampedRelPos0, ref currentRelPos0, out var desiredRelVel0);
            Vector3.Divide(ref desiredRelVel0, MathHelper.Max(dt, JiggleMath.Epsilon), out desiredRelVel0);


            Vector3.Subtract(ref currentVel0, ref currentVel1, out var Vr);
            Vector3.Subtract(ref Vr, ref desiredRelVel0, out Vr);

            var normalVel = Vr.Length();


            if (normalVel > maxVelMag)
            {
                Vector3.Multiply(ref Vr, maxVelMag / normalVel, out Vr);
                normalVel = maxVelMag;
            }
            else if (normalVel < minVelForProcessing)
            {
                return false;
            }

            Vector3.Divide(ref Vr, normalVel, out var N);

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

            var normalImpulse = -normalVel / denominator;

            Vector3.Multiply(ref N, normalImpulse, out var imp);

            if (!body0.Immovable) body0.ApplyWorldImpulse(ref imp, ref worldPos);

            Vector3.Multiply(ref N, -normalImpulse, out imp);

            if (!body1.Immovable) body1.ApplyWorldImpulse(ref imp, ref worldPos);

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