using Microsoft.Xna.Framework;
using JigLibX.Math;

namespace JigLibX.Physics
{
    public class HingeJoint : Joint
    {
        private Vector3 hingeAxis;
        private Vector3 hingePosRel0;
        private Body body0;
        private Body body1;
        private bool usingLimit;
        private float damping;
        private float extraTorque = 0;

        private ConstraintPoint mMidPointConstraint;
        private ConstraintMaxDistance[] mSidePointConstraints;
        private ConstraintMaxDistance mMaxDistanceConstraint;

        public HingeJoint()
        {
        }

        public void Initialise(Body body0, Body body1, Vector3 hingeAxis, Vector3 hingePosRel0, float hingeHalfWidth, float hingeFwdAngle, float hingeBckAngle, float sidewaysSlack, float damping)
        {
            this.body0 = body0;
            this.body1 = body1;
            this.hingeAxis = hingeAxis;
            this.hingePosRel0 = hingePosRel0;
            usingLimit = false;
            this.damping = damping;


            this.hingeAxis.Normalize();

            var hingePosRel1 = body0.Position + hingePosRel0 - body1.Position;


            var relPos0a = hingePosRel0 + hingeHalfWidth * hingeAxis;
            var relPos0b = hingePosRel0 - hingeHalfWidth * hingeAxis;

            var relPos1a = hingePosRel1 + hingeHalfWidth * hingeAxis;
            var relPos1b = hingePosRel1 - hingeHalfWidth * hingeAxis;

            var timescale = 1.0f / 20.0f;
            var allowedDistanceMid = 0.005f;
            var allowedDistanceSide = sidewaysSlack * hingeHalfWidth;

            mSidePointConstraints = new ConstraintMaxDistance[2];

            mSidePointConstraints[0] = new ConstraintMaxDistance();
            mSidePointConstraints[1] = new ConstraintMaxDistance();

            mSidePointConstraints[0].Initialise(body0, relPos0a, body1, relPos1a, allowedDistanceSide);
            mSidePointConstraints[1].Initialise(body0, relPos0b, body1, relPos1b, allowedDistanceSide);

            mMidPointConstraint = new ConstraintPoint();
            mMidPointConstraint.Initialise(body0, hingePosRel0, body1, hingePosRel1, allowedDistanceMid, timescale);

            if (hingeFwdAngle <= 150)
            {
                var perpDir = Vector3.Up;

                if (Vector3.Dot(perpDir, hingeAxis) > 0.1f) perpDir = Vector3.Right;


                var sideAxis = Vector3.Cross(hingeAxis, perpDir);
                perpDir = Vector3.Cross(sideAxis, hingeAxis);
                perpDir.Normalize();


                var len = 10.0f * hingeHalfWidth;


                var hingeRelAnchorPos0 = perpDir * len;


                var angleToMiddle = 0.5f * (hingeFwdAngle - hingeBckAngle);
                var hingeRelAnchorPos1 = Vector3.TransformNormal(hingeRelAnchorPos0, Matrix.CreateFromAxisAngle(hingeAxis, MathHelper.ToRadians(-angleToMiddle)));


                var hingeHalfAngle = 0.5f * (hingeFwdAngle + hingeBckAngle);
                var allowedDistance = len * 2.0f * (float) System.Math.Sin(MathHelper.ToRadians(hingeHalfAngle * 0.5f));

                var hingePos = body1.Position + hingePosRel0;
                var relPos0c = hingePos + hingeRelAnchorPos0 - body0.Position;
                var relPos1c = hingePos + hingeRelAnchorPos1 - body1.Position;

                mMaxDistanceConstraint = new ConstraintMaxDistance();

                mMaxDistanceConstraint.Initialise(body0, relPos0c, body1, relPos1c, allowedDistance);

                usingLimit = true;
            }

            if (damping <= 0.0f)
                damping = -1.0f;
            else
                damping = MathHelper.Clamp(damping, 0, 1);
        }

        public void EnableHinge()
        {
            if (HingeEnabled) return;

            if (body0 != null)
            {
                mMidPointConstraint.EnableConstraint();
                mSidePointConstraints[0].EnableConstraint();
                mSidePointConstraints[1].EnableConstraint();

                if (usingLimit && !IsBroken) mMaxDistanceConstraint.EnableConstraint();

                EnableController();
            }

            HingeEnabled = true;
        }

        public void DisableHinge()
        {
            if (!HingeEnabled) return;

            if (body0 != null)
            {
                mMidPointConstraint.DisableConstraint();
                mSidePointConstraints[0].DisableConstraint();
                mSidePointConstraints[1].DisableConstraint();

                if (usingLimit && !IsBroken) mMaxDistanceConstraint.DisableConstraint();

                DisableController();
            }

            HingeEnabled = false;
        }

        public void Break()
        {
            if (IsBroken) return;

            if (usingLimit) mMaxDistanceConstraint.DisableConstraint();

            IsBroken = true;
        }

        public void Mend()
        {
            if (!IsBroken) return;

            if (usingLimit) mMaxDistanceConstraint.EnableConstraint();

            IsBroken = false;
        }

        public override void UpdateController(float dt)
        {
            if (body0 == null || body1 == null) return;


            if (damping > 0.0f)
            {
                var hingeAxis = body1.AngularVelocity - body0.AngularVelocity;

                JiggleMath.NormalizeSafe(ref hingeAxis);

                Vector3.Dot(ref body0.TransformRate.AngularVelocity, ref hingeAxis, out var angRot1);
                Vector3.Dot(ref body1.TransformRate.AngularVelocity, ref hingeAxis, out var angRot2);

                var avAngRot = 0.5f * (angRot1 + angRot2);

                var frac = 1.0f - damping;
                var newAngRot1 = avAngRot + (angRot1 - avAngRot) * frac;
                var newAngRot2 = avAngRot + (angRot2 - avAngRot) * frac;

                Vector3.Multiply(ref hingeAxis, newAngRot1 - angRot1, out var newAngVel1);
                Vector3.Add(ref newAngVel1, ref body0.TransformRate.AngularVelocity, out newAngVel1);

                Vector3.Multiply(ref hingeAxis, newAngRot2 - angRot2, out var newAngVel2);
                Vector3.Add(ref newAngVel2, ref body1.TransformRate.AngularVelocity, out newAngVel2);

                body0.AngularVelocity = newAngVel1;
                body1.AngularVelocity = newAngVel2;
            }


            if (extraTorque != 0.0f)
            {
                Vector3.TransformNormal(ref hingeAxis, ref body0.Transform.Orientation, out var torque1);
                Vector3.Multiply(ref torque1, extraTorque, out torque1);

                body0.AddWorldTorque(torque1);
                body1.AddWorldTorque(-torque1);
            }
        }

        public bool HingeEnabled { get; private set; }

        public bool IsBroken { get; private set; }
    }
}