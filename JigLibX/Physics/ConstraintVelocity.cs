using System.Collections.Generic;
using Microsoft.Xna.Framework;
using JigLibX.Collision;

namespace JigLibX.Physics
{
    public class ConstraintVelocity : Constraint
    {
        public enum ReferenceFrame
        {
            World,

            Body
        }

        private Body body;
        private ReferenceFrame frame;
        private Vector3 vel;
        private Vector3 angVel;
        private Vector3 velRate;
        private Vector3 angVelRate;
        private Vector3 targetVel;
        private Vector3 targetAngVel;
        private bool doVel;
        private bool doAngVel;

        public ConstraintVelocity(Body body, ReferenceFrame frame, Vector3? vel, Vector3? angVel)
        {
            Initialise(body, frame, vel, angVel);
        }

        public ConstraintVelocity()
        {
            Initialise(null, ReferenceFrame.World, Vector3.Zero, Vector3.Zero);
        }

        public void Initialise(Body body, ReferenceFrame frame, Vector3? vel, Vector3? angVel)
        {
            this.body = body;
            this.frame = frame;

            doVel = vel != null;
            doAngVel = angVel != null;

            if (doVel) targetVel = (Vector3) vel;
            if (doAngVel) targetAngVel = (Vector3) angVel;

            if (body != null)
            {
                vel = body.Velocity;
                angVel = body.AngularVelocity;
            }
            else
            {
                vel = Vector3.Zero;
                angVel = Vector3.Zero;
            }

            velRate = Vector3.Zero;
            angVelRate = Vector3.Zero;

            body?.AddConstraint(this);
        }

        public override void PreApply(float dt)
        {
            Satisfied = false;


            var smoothTime = 0.2f;

            if (doVel) SmoothCD(ref vel, ref velRate, dt, targetVel, smoothTime);

            if (doAngVel) SmoothCD(ref angVel, ref angVelRate, dt, targetAngVel, smoothTime);


            if (doVel && body.CollisionSkin != null && body.CollisionSkin.Collisions.Count != 0)
            {
                var collisions = body.CollisionSkin.Collisions;
                var num = collisions.Count;

                for (var i = 0; i < num; i++)
                {
                    var collInfo = collisions[i];

                    if (collInfo.SkinInfo.Skin1.Owner == null)
                    {
                        var dir = collInfo.DirToBody0;


                        Vector3.Dot(ref vel, ref dir, out var dot);

                        if (dot < 0.0f)
                        {
                            Vector3.Multiply(ref dir, dot, out var v1);
                            Vector3.Subtract(ref vel, ref v1, out vel);
                        }
                    }
                }
            }
        }

        public override bool Apply(float dt)
        {
            Satisfied = true;

            if (body == null) return false;

            var frac = 0.001f;

            if (frame == ReferenceFrame.Body)
            {
                if (doVel)
                {
                    Vector3.TransformNormal(ref vel, ref body.Transform.Orientation, out var velBodyFrame);

                    Vector3.Multiply(ref body.TransformRate.Velocity, 1.0f - frac, out var v1);
                    Vector3.Multiply(ref velBodyFrame, frac, out body.TransformRate.Velocity);
                    Vector3.Add(ref body.TransformRate.Velocity, ref v1, out body.TransformRate.Velocity);
                }

                if (doAngVel)
                {
                    Vector3.TransformNormal(ref angVel, ref body.Transform.Orientation, out var angVelBodyFrame);

                    Vector3.Multiply(ref body.TransformRate.AngularVelocity, 1.0f - frac, out var v1);
                    Vector3.Multiply(ref angVelBodyFrame, frac, out body.TransformRate.AngularVelocity);
                    Vector3.Add(ref body.TransformRate.AngularVelocity, ref v1, out body.TransformRate.AngularVelocity);
                }
            }
            else
            {
                if (doVel)
                {
                    Vector3.Multiply(ref body.TransformRate.Velocity, 1.0f - frac, out body.TransformRate.Velocity);
                    Vector3.Multiply(ref vel, frac, out var v1);
                    Vector3.Add(ref body.TransformRate.Velocity, ref v1, out body.TransformRate.Velocity);
                }

                if (doAngVel)
                {
                    Vector3.Multiply(ref body.TransformRate.AngularVelocity, 1.0f - frac, out body.TransformRate.AngularVelocity);
                    Vector3.Multiply(ref angVel, frac, out var v1);
                    Vector3.Add(ref body.TransformRate.AngularVelocity, ref v1, out body.TransformRate.AngularVelocity);
                }
            }


            body.SetConstraintsAndCollisionsUnsatisfied();
            Satisfied = true;

            return true;
        }

        public override void Destroy()
        {
            body?.RemoveConstraint(this);

            body = null;
            DisableConstraint();
        }
    }
}