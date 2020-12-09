using System.Collections.Generic;
using Microsoft.Xna.Framework;
using JigLibX.Collision;
using JigLibX.Math;

namespace JigLibX.Physics
{
    public class ConstraintWorldPoint : Constraint
    {
        private const float minVelForProcessing = 0.001f;

        private Vector3 pointOnBody;
        private Vector3 worldPosition;

        public ConstraintWorldPoint()
        {
            Initialise(null, Vector3.Zero, Vector3.Zero);
        }

        public ConstraintWorldPoint(Body body, Vector3 pointOnBody, Vector3 worldPosition)
        {
            Initialise(body, pointOnBody, worldPosition);
        }

        public void Initialise(Body body, Vector3 pointOnBody, Vector3 worldPosition)
        {
            Body = body;
            this.pointOnBody = pointOnBody;
            this.worldPosition = worldPosition;

            body?.AddConstraint(this);
        }

        public override void PreApply(float dt)
        {
            Satisfied = false;
        }

        public override bool Apply(float dt)
        {
            Satisfied = true;


            Vector3.TransformNormal(ref pointOnBody, ref Body.Transform.Orientation, out var worldPos);
            Vector3.Add(ref worldPos, ref Body.Transform.Position, out worldPos);

            Vector3.Subtract(ref worldPos, ref Body.Transform.Position, out var R);

            Vector3.Cross(ref Body.TransformRate.AngularVelocity, ref R, out var currentVel);
            Vector3.Add(ref currentVel, ref Body.TransformRate.Velocity, out currentVel);


            Vector3 desiredVel;

            var allowedDeviation = 0.01f;
            var timescale = 4.0f * dt;

            Vector3.Subtract(ref worldPos, ref worldPosition, out var deviation);

            var deviationDistance = deviation.Length();

            if (deviationDistance > allowedDeviation)
            {
                Vector3.Divide(ref deviation, deviationDistance, out var deviationDir);

                Vector3.Multiply(ref deviationDir, (allowedDeviation - deviationDistance) / timescale, out desiredVel);
            }
            else
            {
                desiredVel = Vector3.Zero;
            }


            if (Body.CollisionSkin != null)
            {
                var collisions = Body.CollisionSkin.Collisions;

                var num = collisions.Count;

                for (var i = 0; i < num; i++)
                {
                    var collInfo = collisions[i];

                    if (collInfo.SkinInfo.Skin1.Owner == null)
                    {
                        var dir = collInfo.DirToBody0;

                        Vector3.Dot(ref desiredVel, ref dir, out var dot);

                        if (dot < 0.0f) desiredVel -= dot * dir;
                    }
                }
            }


            Vector3.Subtract(ref currentVel, ref desiredVel, out var N);

            var normalVel = N.Length();

            if (normalVel < minVelForProcessing) return false;

            Vector3.Divide(ref N, normalVel, out N);

            Vector3.Cross(ref R, ref N, out var v1);
            Vector3.TransformNormal(ref v1, ref Body.worldInvInertia, out v1);
            Vector3.Cross(ref v1, ref R, out v1);
            Vector3.Dot(ref N, ref v1, out var f1);

            var denominator = Body.InverseMass + f1;

            if (denominator < JiggleMath.Epsilon) return false;

            var normalImpulse = -normalVel / denominator;

            Body.ApplyWorldImpulse(normalImpulse * N, worldPos);

            Body.SetConstraintsAndCollisionsUnsatisfied();
            Satisfied = true;

            return true;
        }

        public override void Destroy()
        {
            Body?.RemoveConstraint(this);

            Body = null;
            DisableConstraint();
        }

        public Vector3 WorldPosition
        {
            set => worldPosition = value;
            get => worldPosition;
        }

        public Body Body { get; private set; }
    }
}