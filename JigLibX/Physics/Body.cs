using System.Collections.Generic;
using Microsoft.Xna.Framework;
using JigLibX.Collision;
using JigLibX.Math;

namespace JigLibX.Physics
{
    public class Body
    {
        private enum Activity
        {
            Active,
            Inactive
        }

        private bool bodyEnabled;

        internal bool FoundIsland = false;

        private bool immovable;

        public Transform Transform;
        public TransformRate TransformRate;
        private TransformRate transformRateAux;

        public Transform oldTransform;
        public TransformRate oldTransformRate;

        private float invMass;

        private Transform storedTransform;
        private TransformRate storedTransformRate;

        private Matrix invOrientation;
        private float mass;
        private bool origImmovable;

        private Matrix bodyInertia;
        private Matrix worldInertia;
        private Matrix bodyInvInertia;
        internal Matrix worldInvInertia;

        private Vector3 force;
        private Vector3 torque;

        private Activity activity;
        private float inactiveTime;

        private Vector3 lastPositionForDeactivation;

        private Matrix lastOrientationForDeactivation;

        private float deactivationTime;

        private float sqVelocityActivityThreshold;

        private float sqAngVelocityActivityThreshold;

        private Vector3 storedPositionForActivation;

        private readonly List<Body> bodiesToBeActivatedOnMovement;

        private bool allowFreezing;

        private readonly List<Constraint> constraints = new List<Constraint>();

        private const float VelMax = 100.0f;
        private const float AngVelMax = 50.0f;

        public object ExternalData;

        private static int idCounter;
        internal int ID;

        public Body()
        {
            ID = idCounter++;

            bodiesToBeActivatedOnMovement = new List<Body>();
            bodyEnabled = false;
            CollisionSkin = null;

            Mass = 1.0f;
            BodyInertia = Matrix.Identity;

            Transform = new Transform();
            Transform.Position = Vector3.Zero;
            Transform.Orientation = Matrix.Identity;

            immovable = false;
            origImmovable = false;
            DoShockProcessing = true;

            force = Vector3.Zero;
            torque = Vector3.Zero;

            VelChanged = true;

            activity = Activity.Active;
            inactiveTime = 0.0f;
            deactivationTime = 1.0f;
            SetActivityThreshold(0.5f, 20.0f);
            allowFreezing = true;
            lastPositionForDeactivation = Transform.Position;
            lastOrientationForDeactivation = Transform.Orientation;

            CopyCurrentStateToOld();
        }

        public Vector3 Position
        {
            get => Transform.Position;
            set => Transform.Position = value;
        }

        public Vector3 OldPosition => oldTransform.Position;

        public Matrix Orientation
        {
            get => Transform.Orientation;
            set => Transform.Orientation = value;
        }

        public Matrix OldOrientation => oldTransform.Orientation;

        public Vector3 Velocity
        {
            get => TransformRate.Velocity;
            set => TransformRate.Velocity = value;
        }

        public Vector3 VelocityAux
        {
            get => transformRateAux.Velocity;
            set => transformRateAux.Velocity = value;
        }

        public Vector3 OldVelocity => oldTransformRate.Velocity;

        public Vector3 AngularVelocity
        {
            get => TransformRate.AngularVelocity;
            set => TransformRate.AngularVelocity = value;
        }

        public Vector3 AngularVelocityAux
        {
            get => transformRateAux.AngularVelocity;
            set => transformRateAux.AngularVelocity = value;
        }

        public Vector3 OldAngVel => oldTransformRate.AngularVelocity;

        public Vector3 Force
        {
            get => force;
            set => force = value;
        }

        public Vector3 Torque
        {
            get => torque;
            set => torque = value;
        }

        public float Mass
        {
            get => mass;
            set
            {
                mass = value;
                invMass = 1.0f / mass;
                ClearForces();
                AddGravityToExternalForce();
            }
        }

        public float InverseMass
        {
            get => invMass;
            set
            {
                invMass = value;
                mass = 1.0f / value;
            }
        }

        public bool ApplyGravity { get; set; } = true;

        public void RemoveConstraint(Constraint constraint)
        {
            if (constraints.Contains(constraint)) constraints.Remove(constraint);
        }

        public void AddConstraint(Constraint constraint)
        {
            if (!constraints.Contains(constraint)) constraints.Add(constraint);
        }

        public void SetBodyInertia(float xx, float yy, float zz)
        {
            bodyInertia = Matrix.Identity;
            bodyInertia.M11 = xx;
            bodyInertia.M22 = yy;
            bodyInertia.M33 = zz;

            bodyInvInertia = Matrix.Identity;
            bodyInvInertia.M11 = JiggleMath.SafeInvScalar(xx);
            bodyInvInertia.M22 = JiggleMath.SafeInvScalar(yy);
            bodyInvInertia.M33 = JiggleMath.SafeInvScalar(zz);
        }

        public void SetBodyInvInertia(float xx, float yy, float zz)
        {
            bodyInvInertia = Matrix.Identity;
            bodyInvInertia.M11 = xx;
            bodyInvInertia.M22 = yy;
            bodyInvInertia.M33 = zz;

            bodyInertia = Matrix.Identity;
            bodyInertia.M11 = JiggleMath.SafeInvScalar(xx);
            bodyInertia.M22 = JiggleMath.SafeInvScalar(yy);
            bodyInertia.M33 = JiggleMath.SafeInvScalar(zz);
        }

        public virtual void PostPhysics(float dt)
        {
        }

        public virtual void PrePhysics(float dt)
        {
        }

        public virtual void EnableBody()
        {
            if (bodyEnabled) return;
            if (PhysicsSystem.CurrentPhysicsSystem == null) return;

            bodyEnabled = true;
            PhysicsSystem.CurrentPhysicsSystem.AddBody(this);
        }

        public virtual void DisableBody()
        {
            if (!bodyEnabled) return;
            if (PhysicsSystem.CurrentPhysicsSystem == null) return;

            bodyEnabled = false;
            PhysicsSystem.CurrentPhysicsSystem.RemoveBody(this);
        }

        public virtual bool IsBodyEnabled => bodyEnabled;

        public CollisionSkin CollisionSkin { get; set; }

        public void MoveTo(Vector3 pos, Matrix orientation)
        {
            if (bodyEnabled && !IsActive) SetActive();

            Position = pos;
            Orientation = orientation;
            Velocity = Vector3.Zero;
            AngularVelocity = Vector3.Zero;
            CopyCurrentStateToOld();

            CollisionSkin?.SetTransform(ref oldTransform, ref Transform);
        }

        public void UpdateVelocity(float dt)
        {
            if (immovable || !IsActive) return;

            Vector3.Multiply(ref force, dt * invMass, out var vel);
            Vector3.Add(ref TransformRate.Velocity, ref vel, out TransformRate.Velocity);

            Vector3.Multiply(ref torque, dt, out var angVel);
            Vector3.TransformNormal(ref angVel, ref worldInvInertia, out angVel);
            Vector3.Add(ref TransformRate.AngularVelocity, ref angVel, out TransformRate.AngularVelocity);


            if (CollisionSkin != null && CollisionSkin.Collisions.Count >= 1)
                Vector3.Multiply(ref TransformRate.AngularVelocity, 0.99f, out TransformRate.AngularVelocity);
        }

        public void UpdatePosition(float dt)
        {
            if (immovable || !IsActive) return;

            Vector3.TransformNormal(ref TransformRate.AngularVelocity, ref worldInertia, out var angMomBefore);

            Transform.ApplyTransformRate(TransformRate, dt);

            Matrix.Transpose(ref Transform.Orientation, out invOrientation);


            Matrix.Multiply(ref invOrientation, ref bodyInvInertia, out worldInvInertia);
            Matrix.Multiply(ref worldInvInertia, ref Transform.Orientation, out worldInvInertia);

            Matrix.Multiply(ref invOrientation, ref bodyInertia, out worldInertia);
            Matrix.Multiply(ref worldInertia, ref Transform.Orientation, out worldInertia);


            Vector3.TransformNormal(ref angMomBefore, ref worldInvInertia, out TransformRate.AngularVelocity);

            CollisionSkin?.SetTransform(ref oldTransform, ref Transform);
        }

        public void UpdatePositionWithAux(float dt)
        {
            if (immovable || !IsActive)
            {
                transformRateAux = TransformRate.Zero;
                return;
            }

            var physics = PhysicsSystem.CurrentPhysicsSystem;
            var ga = physics.MainGravityAxis;

            if (ga != -1)
            {
                var ga2 = (ga + 1) % 3;
                if (ga2 == 0)
                {
                    transformRateAux.Velocity.X *= 0.1f;
                    transformRateAux.Velocity.Y *= 0.1f;
                }
                else if (ga2 == 1)
                {
                    transformRateAux.Velocity.Y *= 0.1f;
                    transformRateAux.Velocity.Z *= 0.1f;
                }
                else if (ga2 == 2)
                {
                    transformRateAux.Velocity.Z *= 0.1f;
                    transformRateAux.Velocity.X *= 0.1f;
                }
            }

            Vector3.TransformNormal(ref TransformRate.AngularVelocity, ref worldInertia, out var angMomBefore);

            TransformRate.Add(ref TransformRate, ref transformRateAux, out var rate);

            Transform.ApplyTransformRate(ref rate, dt);

            transformRateAux.AngularVelocity.X = 0.0f;
            transformRateAux.AngularVelocity.Y = 0.0f;
            transformRateAux.AngularVelocity.Z = 0.0f;
            transformRateAux.Velocity.X = 0.0f;
            transformRateAux.Velocity.Y = 0.0f;
            transformRateAux.Velocity.Z = 0.0f;

            Matrix.Transpose(ref Transform.Orientation, out invOrientation);


            Matrix.Multiply(ref invOrientation, ref bodyInvInertia, out worldInvInertia);
            Matrix.Multiply(ref worldInvInertia, ref Transform.Orientation, out worldInvInertia);


            Matrix.Multiply(ref invOrientation, ref bodyInertia, out worldInertia);
            Matrix.Multiply(ref worldInertia, ref Transform.Orientation, out worldInertia);


            Vector3.TransformNormal(ref angMomBefore, ref worldInvInertia, out TransformRate.AngularVelocity);

            CollisionSkin?.SetTransform(ref oldTransform, ref Transform);
        }

        public void InternalSetImmovable()
        {
            origImmovable = immovable;
            immovable = true;
        }

        internal void InternalRestoreImmovable() => immovable = origImmovable;

        public bool VelChanged { get; private set; }

        public void ClearVelChanged() => VelChanged = false;

        public Matrix BodyInertia
        {
            get => bodyInertia;
            set
            {
                bodyInertia = value;
                Matrix.Invert(ref value, out bodyInvInertia);
            }
        }

        public Matrix BodyInvInertia
        {
            get => bodyInvInertia;
            set
            {
                bodyInvInertia = value;
                Matrix.Invert(ref value, out bodyInertia);
            }
        }

        public Matrix WorldInertia => worldInertia;

        public Matrix WorldInvInertia => worldInvInertia;

        public bool Immovable
        {
            get => immovable;
            set
            {
                immovable = value;
                origImmovable = immovable;
            }
        }

        public bool IsActive => activity == Activity.Active;

        public void SetActive()
        {
            if (activity == Activity.Active) return;
            inactiveTime = 0.0f;
            activity = Activity.Active;
        }

        public void SetInactive()
        {
            if (allowFreezing && PhysicsSystem.CurrentPhysicsSystem.IsFreezingEnabled)
            {
                inactiveTime = deactivationTime;
                activity = Activity.Inactive;
            }
        }

        public bool GetShouldBeActive()
        {
            if (inactiveTime >= deactivationTime)
                return false;
            else
                return true;
        }

        public void DampForDeactivation()
        {
            var frac = inactiveTime / deactivationTime;


            var r = 0.5f;

            if (frac < r) return;

            var scale = 1.0f - (frac - r) / (1.0f - r);
            scale = MathHelper.Clamp(scale, 0.0f, 1.0f);


            var frac0 = TransformRate.Velocity.LengthSquared() * scale * scale / sqVelocityActivityThreshold;
            var frac1 = TransformRate.AngularVelocity.LengthSquared() * scale * scale / sqAngVelocityActivityThreshold;

            if (frac0 > 1.0f) Vector3.Multiply(ref TransformRate.Velocity, scale, out TransformRate.Velocity);
            if (frac1 > 1.0f)
                Vector3.Multiply(ref TransformRate.AngularVelocity, scale, out TransformRate.AngularVelocity);
        }

        public void UpdateDeactivation(float dt)
        {
            if (TransformRate.Velocity.LengthSquared() > sqVelocityActivityThreshold || TransformRate.AngularVelocity.LengthSquared() > sqAngVelocityActivityThreshold)
                inactiveTime = 0.0f;
            else
                inactiveTime += dt;
        }

        public bool GetShouldBeActiveAux()
        {
            return transformRateAux.Velocity.LengthSquared() > sqVelocityActivityThreshold || transformRateAux.AngularVelocity.LengthSquared() > sqAngVelocityActivityThreshold;
        }

        public void SetDeactivationTime(float seconds)
        {
            deactivationTime = seconds;
        }

        public void SetActivityThreshold(float vel, float angVel)
        {
            sqVelocityActivityThreshold = vel * vel;
            sqAngVelocityActivityThreshold = MathHelper.ToRadians(angVel) * MathHelper.ToRadians(angVel);
        }

        public bool AllowFreezing
        {
            get => allowFreezing;
            set
            {
                allowFreezing = value;

                if (!value) SetActive();
            }
        }

        public bool DoShockProcessing { get; set; }

        public void LimitVel()
        {
            if (!(TransformRate.Velocity.X < -VelMax) && !(TransformRate.Velocity.X > VelMax) && !(TransformRate.Velocity.Y < -VelMax) && !(TransformRate.Velocity.Y > VelMax) && !(TransformRate.Velocity.Z < -VelMax) && !(TransformRate.Velocity.Z > VelMax)) return;
            float vMin;
            float vMax;
                
            if (TransformRate.Velocity.X < TransformRate.Velocity.Y)
                vMin = TransformRate.Velocity.X < TransformRate.Velocity.Z ? TransformRate.Velocity.X : TransformRate.Velocity.Z;
            else
                vMin = TransformRate.Velocity.Y < TransformRate.Velocity.Z ? TransformRate.Velocity.Y : TransformRate.Velocity.Z;


            if (TransformRate.Velocity.X > TransformRate.Velocity.Y)
                vMax = TransformRate.Velocity.X > TransformRate.Velocity.Z ? TransformRate.Velocity.X : TransformRate.Velocity.Z;
            else
                vMax = TransformRate.Velocity.Y > TransformRate.Velocity.Z ? TransformRate.Velocity.Y : TransformRate.Velocity.Z;


            var dMin = vMin < -VelMax ? vMin + VelMax : 0f;
            var dMax = vMax > VelMax ? vMax - VelMax : 0f;
            dMin = dMin < 0 ? -dMin : dMin;

            dMax = dMax < 0 ? -dMax : dMax;


            var scaleFactor = dMin > dMax ? -VelMax / vMin : VelMax / vMax;


            TransformRate.Velocity.X *= scaleFactor;
            TransformRate.Velocity.Y *= scaleFactor;
            TransformRate.Velocity.Z *= scaleFactor;
        }

        public void LimitAngVel()
        {
            if (TransformRate.AngularVelocity.X < -AngVelMax || TransformRate.AngularVelocity.X > AngVelMax || TransformRate.AngularVelocity.Y < -AngVelMax || TransformRate.AngularVelocity.Y > AngVelMax || TransformRate.AngularVelocity.Z < -AngVelMax || TransformRate.AngularVelocity.Z > AngVelMax)
            {
                float v_min, v_max, dMin, dMax, scaleFactor;


                if (TransformRate.AngularVelocity.X < TransformRate.AngularVelocity.Y)
                    v_min = TransformRate.AngularVelocity.X < TransformRate.AngularVelocity.Z ? TransformRate.AngularVelocity.X : TransformRate.AngularVelocity.Z;
                else
                    v_min = TransformRate.AngularVelocity.Y < TransformRate.AngularVelocity.Z ? TransformRate.AngularVelocity.Y : TransformRate.AngularVelocity.Z;


                if (TransformRate.AngularVelocity.X > TransformRate.AngularVelocity.Y)
                    v_max = TransformRate.AngularVelocity.X > TransformRate.AngularVelocity.Z ? TransformRate.AngularVelocity.X : TransformRate.AngularVelocity.Z;
                else
                    v_max = TransformRate.AngularVelocity.Y > TransformRate.AngularVelocity.Z ? TransformRate.AngularVelocity.Y : TransformRate.AngularVelocity.Z;


                dMin = v_min < -AngVelMax ? v_min + AngVelMax : 0f;
                dMax = v_max > AngVelMax ? v_max - AngVelMax : 0f;
                dMin = dMin < 0 ? -dMin : dMin;

                dMax = dMax < 0 ? -dMax : dMax;


                scaleFactor = dMin > dMax ? -AngVelMax / v_min : AngVelMax / v_max;


                TransformRate.AngularVelocity.X *= scaleFactor;
                TransformRate.AngularVelocity.Y *= scaleFactor;
                TransformRate.AngularVelocity.Z *= scaleFactor;
            }
        }

        public Vector3 GetVelocity(Vector3 relPos)
        {
            return new Vector3(TransformRate.Velocity.X + TransformRate.AngularVelocity.Y * relPos.Z - TransformRate.AngularVelocity.Z * relPos.Y, TransformRate.Velocity.Y + TransformRate.AngularVelocity.Z * relPos.X - TransformRate.AngularVelocity.X * relPos.Z, TransformRate.Velocity.Z + TransformRate.AngularVelocity.X * relPos.Y - TransformRate.AngularVelocity.Y * relPos.X);
        }

        public void GetVelocity(ref Vector3 relPos, out Vector3 result)
        {
            result.X = TransformRate.Velocity.X + TransformRate.AngularVelocity.Y * relPos.Z - TransformRate.AngularVelocity.Z * relPos.Y;
            result.Y = TransformRate.Velocity.Y + TransformRate.AngularVelocity.Z * relPos.X - TransformRate.AngularVelocity.X * relPos.Z;
            result.Z = TransformRate.Velocity.Z + TransformRate.AngularVelocity.X * relPos.Y - TransformRate.AngularVelocity.Y * relPos.X;
        }

        public Vector3 GetVelocityAux(Vector3 relPos)
        {
            return new Vector3(transformRateAux.Velocity.X + transformRateAux.AngularVelocity.Y * relPos.Z - transformRateAux.AngularVelocity.Z * relPos.Y, transformRateAux.Velocity.Y + transformRateAux.AngularVelocity.Z * relPos.X - transformRateAux.AngularVelocity.X * relPos.Z, transformRateAux.Velocity.Z + transformRateAux.AngularVelocity.X * relPos.Y - transformRateAux.AngularVelocity.Y * relPos.X);
        }

        public void GetVelocityAux(ref Vector3 relPos, out Vector3 result)
        {
            result = new Vector3(transformRateAux.Velocity.X + transformRateAux.AngularVelocity.Y * relPos.Z - transformRateAux.AngularVelocity.Z * relPos.Y, transformRateAux.Velocity.Y + transformRateAux.AngularVelocity.Z * relPos.X - transformRateAux.AngularVelocity.X * relPos.Z, transformRateAux.Velocity.Z + transformRateAux.AngularVelocity.X * relPos.Y - transformRateAux.AngularVelocity.Y * relPos.X);
        }

        public void AddMovementActivation(Vector3 pos, Body otherBody)
        {
            if (bodiesToBeActivatedOnMovement.Contains(otherBody)) return;
            if (bodiesToBeActivatedOnMovement.Count == 0) storedPositionForActivation = pos;
            bodiesToBeActivatedOnMovement.Add(otherBody);
        }

        public void SetOrientation(Matrix orient)
        {
            Transform.Orientation = orient;
            Matrix.Transpose(ref Transform.Orientation, out invOrientation);
            Matrix.Multiply(ref bodyInvInertia, ref invOrientation, out worldInvInertia);
            Matrix.Multiply(ref worldInvInertia, ref Transform.Orientation, out worldInvInertia);
            Matrix.Multiply(ref bodyInertia, ref invOrientation, out worldInertia);
            Matrix.Multiply(ref worldInertia, ref Transform.Orientation, out worldInertia);
        }

        public void ApplyWorldImpulse(Vector3 impulse)
        {
            if (immovable) return;
            TransformRate.Velocity.X += impulse.X * invMass;
            TransformRate.Velocity.Y += impulse.Y * invMass;
            TransformRate.Velocity.Z += impulse.Z * invMass;

            VelChanged = true;
        }

        public void ApplyNegativeWorldImpulse(Vector3 impulse)
        {
            if (immovable) return;
            TransformRate.Velocity.X -= impulse.X * invMass;
            TransformRate.Velocity.Y -= impulse.Y * invMass;
            TransformRate.Velocity.Z -= impulse.Z * invMass;

            VelChanged = true;
        }

        public void ApplyWorldImpulseAux(Vector3 impulse)
        {
            if (immovable) return;
            Vector3.Multiply(ref impulse, invMass, out impulse);
            Vector3.Add(ref transformRateAux.Velocity, ref impulse, out transformRateAux.Velocity);
            VelChanged = true;
        }

        public void ApplyNegativeWorldImpulseAux(Vector3 impulse)
        {
            if (immovable) return;
            Vector3.Multiply(ref impulse, -invMass, out impulse);
            Vector3.Add(ref transformRateAux.Velocity, ref impulse, out transformRateAux.Velocity);
            VelChanged = true;
        }

        public void ApplyWorldImpulse(ref Vector3 impulse, ref Vector3 pos)
        {
            if (immovable) return;
            Vector3.Subtract(ref pos, ref Transform.Position, out var v0);
            Vector3.Multiply(ref impulse, invMass, out var v1);
            Vector3.Add(ref TransformRate.Velocity, ref v1, out TransformRate.Velocity);
            Vector3.Cross(ref v0, ref impulse, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref TransformRate.AngularVelocity, ref v1, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyWorldImpulse(Vector3 impulse, Vector3 pos)
        {
            if (immovable) return;
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Multiply(ref impulse, invMass, out var v1);
            Vector3.Add(ref TransformRate.Velocity, ref v1, out TransformRate.Velocity);
            Vector3.Cross(ref pos, ref impulse, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref TransformRate.AngularVelocity, ref v1, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyNegativeWorldImpulse(Vector3 impulse, Vector3 pos)
        {
            if (immovable) return;
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Multiply(ref impulse, -invMass, out var v1);
            Vector3.Add(ref TransformRate.Velocity, ref v1, out TransformRate.Velocity);
            Vector3.Cross(ref impulse, ref pos, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref TransformRate.AngularVelocity, ref v1, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyWorldImpulseAux(Vector3 impulse, Vector3 pos)
        {
            if (immovable) return;
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Multiply(ref impulse, invMass, out var v1);
            Vector3.Add(ref transformRateAux.Velocity, ref v1, out transformRateAux.Velocity);
            Vector3.Cross(ref pos, ref impulse, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref transformRateAux.AngularVelocity, ref v1, out transformRateAux.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyNegativeWorldImpulseAux(Vector3 impulse, Vector3 pos)
        {
            if (immovable) return;
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Multiply(ref impulse, -invMass, out var v1);
            Vector3.Add(ref transformRateAux.Velocity, ref v1, out transformRateAux.Velocity);
            Vector3.Cross(ref impulse, ref pos, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref transformRateAux.AngularVelocity, ref v1, out transformRateAux.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyWorldAngImpulse(Vector3 angImpulse)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref angImpulse, ref worldInvInertia, out angImpulse);
            Vector3.Add(ref TransformRate.AngularVelocity, ref angImpulse, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyBodyWorldImpulse(Vector3 impulse, Vector3 delta)
        {
            if (immovable) return;
            Vector3.Multiply(ref impulse, invMass, out var v1);
            Vector3.Add(ref TransformRate.Velocity, ref v1, out TransformRate.Velocity);
            Vector3.Cross(ref delta, ref impulse, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref TransformRate.AngularVelocity, ref v1, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyBodyWorldImpulse(ref Vector3 impulse, ref Vector3 delta)
        {
            if (immovable) return;

            TransformRate.Velocity.X += impulse.X * invMass;
            TransformRate.Velocity.Y += impulse.Y * invMass;
            TransformRate.Velocity.Z += impulse.Z * invMass;

            float num2;

            var num0 = delta.Y * impulse.Z - delta.Z * impulse.Y;
            var num1 = delta.Z * impulse.X - delta.X * impulse.Z;
            num2 = delta.X * impulse.Y - delta.Y * impulse.X;

            var num3 = num0 * worldInvInertia.M11 + num1 * worldInvInertia.M21 + num2 * worldInvInertia.M31;
            var num4 = num0 * worldInvInertia.M12 + num1 * worldInvInertia.M22 + num2 * worldInvInertia.M32;
            var num5 = num0 * worldInvInertia.M13 + num1 * worldInvInertia.M23 + num2 * worldInvInertia.M33;

            TransformRate.AngularVelocity.X += num3;
            TransformRate.AngularVelocity.Y += num4;
            TransformRate.AngularVelocity.Z += num5;

            VelChanged = true;
        }

        public void ApplyNegativeBodyWorldImpulse(Vector3 impulse, Vector3 delta)
        {
            if (immovable) return;
            Vector3.Multiply(ref impulse, -invMass, out var v1);
            Vector3.Add(ref TransformRate.Velocity, ref v1, out TransformRate.Velocity);
            Vector3.Cross(ref impulse, ref delta, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref TransformRate.AngularVelocity, ref v1, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyNegativeBodyWorldImpulse(ref Vector3 impulse, ref Vector3 delta)
        {
            if (immovable) return;

            TransformRate.Velocity.X -= impulse.X * invMass;
            TransformRate.Velocity.Y -= impulse.Y * invMass;
            TransformRate.Velocity.Z -= impulse.Z * invMass;

            float num0, num1, num2;

            num0 = delta.Z * impulse.Y - delta.Y * impulse.Z;
            num1 = delta.X * impulse.Z - delta.Z * impulse.X;
            num2 = delta.Y * impulse.X - delta.X * impulse.Y;

            var num3 = num0 * worldInvInertia.M11 + num1 * worldInvInertia.M21 + num2 * worldInvInertia.M31;
            var num4 = num0 * worldInvInertia.M12 + num1 * worldInvInertia.M22 + num2 * worldInvInertia.M32;
            var num5 = num0 * worldInvInertia.M13 + num1 * worldInvInertia.M23 + num2 * worldInvInertia.M33;

            TransformRate.AngularVelocity.X += num3;
            TransformRate.AngularVelocity.Y += num4;
            TransformRate.AngularVelocity.Z += num5;

            VelChanged = true;
        }

        public void ApplyBodyWorldImpulseAux(ref Vector3 impulse, ref Vector3 delta)
        {
            if (immovable) return;
            Vector3.Multiply(ref impulse, invMass, out var v1);
            Vector3.Add(ref transformRateAux.Velocity, ref v1, out transformRateAux.Velocity);
            Vector3.Cross(ref delta, ref impulse, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref transformRateAux.AngularVelocity, ref v1, out transformRateAux.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyNegativeBodyWorldImpulseAux(ref Vector3 impulse, ref Vector3 delta)
        {
            if (immovable) return;
            Vector3.Multiply(ref impulse, -invMass, out var v1);
            Vector3.Add(ref transformRateAux.Velocity, ref v1, out transformRateAux.Velocity);
            Vector3.Cross(ref impulse, ref delta, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref transformRateAux.AngularVelocity, ref v1, out transformRateAux.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyBodyImpulse(Vector3 impulse)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref impulse, ref Transform.Orientation, out impulse);
            Vector3.Multiply(ref impulse, invMass, out impulse);
            Vector3.Add(ref TransformRate.Velocity, ref impulse, out TransformRate.Velocity);
            VelChanged = true;
        }

        public void ApplyNegativeBodyImpulse(Vector3 impulse)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref impulse, ref Transform.Orientation, out impulse);
            Vector3.Multiply(ref impulse, -invMass, out impulse);
            Vector3.Add(ref TransformRate.Velocity, ref impulse, out TransformRate.Velocity);
            VelChanged = true;
        }

        public void ApplyBodyImpulse(Vector3 impulse, Vector3 pos)
        {
            if (immovable) return;

            Vector3.TransformNormal(ref impulse, ref Transform.Orientation, out impulse);
            Vector3.TransformNormal(ref pos, ref Transform.Orientation, out pos);
            Vector3.Add(ref Transform.Position, ref pos, out pos);
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Multiply(ref impulse, invMass, out var v1);
            Vector3.Add(ref TransformRate.Velocity, ref v1, out TransformRate.Velocity);
            Vector3.Cross(ref pos, ref impulse, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref TransformRate.AngularVelocity, ref v1, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyNegativeBodyImpulse(Vector3 impulse, Vector3 pos)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref impulse, ref Transform.Orientation, out impulse);
            Vector3.TransformNormal(ref pos, ref Transform.Orientation, out pos);
            Vector3.Add(ref Transform.Position, ref pos, out pos);
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Multiply(ref impulse, -invMass, out var v1);
            Vector3.Add(ref TransformRate.Velocity, ref v1, out TransformRate.Velocity);
            Vector3.Cross(ref impulse, ref pos, out v1);
            Vector3.TransformNormal(ref v1, ref worldInvInertia, out v1);
            Vector3.Add(ref TransformRate.AngularVelocity, ref v1, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void ApplyBodyAngImpulse(Vector3 angImpulse)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref angImpulse, ref Transform.Orientation, out angImpulse);
            Vector3.TransformNormal(ref angImpulse, ref worldInvInertia, out angImpulse);
            Vector3.Add(ref TransformRate.AngularVelocity, ref angImpulse, out TransformRate.AngularVelocity);
            VelChanged = true;
        }

        public void AddWorldForce(Vector3 force)
        {
            if (immovable) return;
            Vector3.Add(ref this.force, ref force, out this.force);
            VelChanged = true;
        }

        public void AddWorldForce(Vector3 force, Vector3 pos)
        {
            if (immovable) return;
            Vector3.Add(ref this.force, ref force, out this.force);
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Cross(ref pos, ref force, out pos);
            Vector3.Add(ref pos, ref torque, out torque);
            VelChanged = true;
        }

        public void AddWorldTorque(Vector3 torque)
        {
            if (immovable) return;
            Vector3.Add(ref this.torque, ref torque, out this.torque);
            VelChanged = true;
        }

        public void AddBodyForce(Vector3 force)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref force, ref Transform.Orientation, out force);
            Vector3.Add(ref this.force, ref force, out this.force);
            VelChanged = true;
        }

        public void AddBodyForce(Vector3 force, Vector3 pos)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref force, ref Transform.Orientation, out force);
            Vector3.TransformNormal(ref pos, ref Transform.Orientation, out pos);
            Vector3.Add(ref Transform.Position, ref pos, out pos);
            Vector3.Add(ref this.force, ref force, out this.force);
            Vector3.Subtract(ref pos, ref Transform.Position, out pos);
            Vector3.Cross(ref pos, ref force, out pos);
            Vector3.Add(ref pos, ref torque, out torque);
            VelChanged = true;
        }

        public void AddBodyTorque(Vector3 torque)
        {
            if (immovable) return;
            Vector3.TransformNormal(ref torque, ref Transform.Orientation, out torque);
            Vector3.Add(ref this.torque, ref torque, out this.torque);
            VelChanged = true;
        }

        public void ClearForces()
        {
            force = torque = Vector3.Zero;
        }

        public void AddGravityToExternalForce()
        {
            if (PhysicsSystem.CurrentPhysicsSystem != null && ApplyGravity)
                force += Vector3.Multiply(PhysicsSystem.CurrentPhysicsSystem.Gravity, mass);
        }

        public virtual void AddExternalForces(float dt)
        {
            ClearForces();
            AddGravityToExternalForce();
        }

        public void CopyCurrentStateToOld()
        {
            oldTransform = Transform;
            oldTransformRate = TransformRate;
        }

        public void SetConstraintsAndCollisionsUnsatisfied()
        {
            int count;

            count = constraints.Count;
            for (var i = 0; i < count; i++) constraints[i].Satisfied = false;

            if (CollisionSkin == null) return;

            count = CollisionSkin.Collisions.Count;
            for (var i = 0; i < count; i++) CollisionSkin.Collisions[i].Satisfied = false;
        }

        public void StoreState()
        {
            storedTransform = Transform;
            storedTransformRate = TransformRate;
        }

        public void RestoreState()
        {
            Transform = storedTransform;
            TransformRate = storedTransformRate;

            Matrix.Transpose(ref Transform.Orientation, out invOrientation);


            Matrix.Multiply(ref invOrientation, ref bodyInvInertia, out worldInvInertia);
            Matrix.Multiply(ref worldInvInertia, ref Transform.Orientation, out worldInvInertia);


            Matrix.Multiply(ref invOrientation, ref bodyInertia, out worldInertia);
            Matrix.Multiply(ref worldInertia, ref Transform.Orientation, out worldInertia);
        }
    }
}