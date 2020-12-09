using Microsoft.Xna.Framework;
using JigLibX.Math;
using JigLibX.Physics;
using JigLibX.Geometry;
using JigLibX.Collision;

namespace JigLibX.Vehicles
{
    public class Wheel
    {
        private const int MaxNumRays = 32;
        
        private readonly float[] _fracs = new float[MaxNumRays];
        private readonly CollisionSkin[] _otherSkins = new CollisionSkin[MaxNumRays];
        private readonly Vector3[] _groundPositions = new Vector3[MaxNumRays];
        private readonly Vector3[] _groundNormals = new Vector3[MaxNumRays];
        private readonly Segment[] _segments = new Segment[MaxNumRays];

        private Car _car;
        private float _spring;
        private float _travel;
        private float _inertia;
        private float _sideFriction;
        private float _fwdFriction;
        private float _damping;
        private int _numRays;
        private float _angVel;
        private float _torque;
        private float _driveTorque;
        private float _upSpeed;
        private float _lastDisplacement;
        private float _angVelForGrip;
        private WheelPred _pred;


        public void Setup(Car car, Vector3 pos, Vector3 axisUp, float spring, float travel, float inertia, float radius, float sideFriction, float fwdFriction, float damping, int numRays)
        {
            _car = car;
            Pos = pos;
            LocalAxisUp = axisUp;
            _spring = spring;
            _travel = travel;
            _inertia = inertia;
            Radius = radius;
            _sideFriction = sideFriction;
            _fwdFriction = fwdFriction;
            _damping = damping;
            _numRays = numRays;

            _pred = new WheelPred(car.Chassis.Body.CollisionSkin);

            Reset();
        }

        public void Reset()
        {
            _angVel = 0.0f;
            SteerAngle = 0.0f;
            _torque = 0.0f;
            _driveTorque = 0.0f;
            AxisAngle = 0.0f;
            Displacement = 0.0f;
            _upSpeed = 0.0f;
            Lock = false;
            _lastDisplacement = 0.0f;
            OnFloor = false;
            _angVelForGrip = 0.0f;
        }
        
        public bool AddForcesToCar(float dt)
        {
            var force = Vector3.Zero;
            _lastDisplacement = Displacement;
            Displacement = 0.0f;

            var carBody = _car.Chassis.Body;

            var worldPos = carBody.Position + Vector3.TransformNormal(Pos, carBody.Orientation);
            var worldAxis = Vector3.TransformNormal(LocalAxisUp, carBody.Orientation);


            var wheelFwd = Vector3.TransformNormal(carBody.Orientation.Right, JiggleMath.RotationMatrix(SteerAngle, worldAxis));

            var wheelUp = worldAxis;
            var wheelLeft = Vector3.Cross(wheelUp, wheelFwd);
            wheelLeft.Normalize();

            wheelUp = Vector3.Cross(wheelFwd, wheelLeft);


            var rayLen = 2.0f * Radius + _travel;
            var wheelRayEnd = worldPos - Radius * worldAxis;
            var wheelRay = new Segment(wheelRayEnd + rayLen * worldAxis, -rayLen * worldAxis);


            var collSystem = PhysicsSystem.CurrentPhysicsSystem.CollisionSystem;


            var numRaysUse = System.Math.Min(_numRays, MaxNumRays);


            var deltaFwd = 2.0f * Radius / (numRaysUse + 1);
            var deltaFwdStart = deltaFwd;


            OnFloor = false;
            var bestIRay = 0;
            int iRay;

            for (iRay = 0; iRay < numRaysUse; ++iRay)
            {
                _fracs[iRay] = float.MaxValue;

                var distFwd = deltaFwdStart + iRay * deltaFwd - Radius;

                var zOffset = Radius * (1.0f - (float) System.Math.Cos(MathHelper.ToRadians(90.0f * (distFwd / Radius))));

                _segments[iRay] = wheelRay;
                _segments[iRay].Origin += distFwd * wheelFwd + zOffset * wheelUp;

                if (collSystem.SegmentIntersect(out _fracs[iRay], out _otherSkins[iRay], out _groundPositions[iRay], out _groundNormals[iRay], _segments[iRay], _pred))
                {
                    OnFloor = true;

                    if (_fracs[iRay] < _fracs[bestIRay]) bestIRay = iRay;
                }
            }


            if (!OnFloor) return false;


            var groundPos = _groundPositions[bestIRay];
            var frac = _fracs[bestIRay];
            var otherSkin = _otherSkins[bestIRay];


            var groundNormal = worldAxis;

            if (numRaysUse > 1)
            {
                for (iRay = 0; iRay < numRaysUse; ++iRay)
                    if (_fracs[iRay] <= 1.0f)
                        groundNormal += (1.0f - _fracs[iRay]) * (worldPos - _segments[iRay].GetEnd());

                JiggleMath.NormalizeSafe(ref groundNormal);
            }
            else
            {
                groundNormal = _groundNormals[bestIRay];
            }


            var worldBody = otherSkin.Owner;

            Displacement = rayLen * (1.0f - frac);
            Displacement = MathHelper.Clamp(Displacement, 0, _travel);

            var displacementForceMag = Displacement * _spring;


            displacementForceMag *= Vector3.Dot(_groundNormals[bestIRay], worldAxis);


            var dampingForceMag = _upSpeed * _damping;

            var totalForceMag = displacementForceMag + dampingForceMag;

            if (totalForceMag < 0.0f) totalForceMag = 0.0f;

            var extraForce = totalForceMag * worldAxis;

            force += extraForce;


            var groundUp = groundNormal;
            var groundLeft = Vector3.Cross(groundNormal, wheelFwd);
            JiggleMath.NormalizeSafe(ref groundLeft);

            var groundFwd = Vector3.Cross(groundLeft, groundUp);

            var wheelPointVel = carBody.Velocity + Vector3.Cross(carBody.AngularVelocity, Vector3.TransformNormal(Pos, carBody.Orientation));

            var rimVel = _angVel * Vector3.Cross(wheelLeft, groundPos - worldPos);
            wheelPointVel += rimVel;


            if (worldBody != null)
            {
                var worldVel = worldBody.Velocity + Vector3.Cross(worldBody.AngularVelocity, groundPos - worldBody.Position);

                wheelPointVel -= worldVel;
            }


            var noslipVel = 0.2f;
            var slipVel = 0.4f;
            var slipFactor = 0.7f;

            float smallVel = 3;
            var friction = _sideFriction;

            var sideVel = Vector3.Dot(wheelPointVel, groundLeft);

            if (sideVel > slipVel || sideVel < -slipVel)
                friction *= slipFactor;
            else if (sideVel > noslipVel || sideVel < -noslipVel)
                friction *= 1.0f - (1.0f - slipFactor) * (System.Math.Abs(sideVel) - noslipVel) / (slipVel - noslipVel);

            if (sideVel < 0.0f) friction *= -1.0f;

            if (System.Math.Abs(sideVel) < smallVel) friction *= System.Math.Abs(sideVel) / smallVel;

            var sideForce = -friction * totalForceMag;

            extraForce = sideForce * groundLeft;
            force += extraForce;


            friction = _fwdFriction;
            var fwdVel = Vector3.Dot(wheelPointVel, groundFwd);

            if (fwdVel > slipVel || fwdVel < -slipVel)
                friction *= slipFactor;
            else if (fwdVel > noslipVel || fwdVel < -noslipVel)
                friction *= 1.0f - (1.0f - slipFactor) * (System.Math.Abs(fwdVel) - noslipVel) / (slipVel - noslipVel);

            if (fwdVel < 0.0f) friction *= -1.0f;

            if (System.Math.Abs(fwdVel) < smallVel) friction *= System.Math.Abs(fwdVel) / smallVel;

            var fwdForce = -friction * totalForceMag;

            extraForce = fwdForce * groundFwd;
            force += extraForce;


            var wheelCentreVel = carBody.Velocity + Vector3.Cross(carBody.AngularVelocity, Vector3.TransformNormal(Pos, carBody.Orientation));

            _angVelForGrip = Vector3.Dot(wheelCentreVel, groundFwd) / Radius;
            _torque += -fwdForce * Radius;


            carBody.AddWorldForce(force, groundPos);


            if (worldBody != null && !worldBody.Immovable)
            {
                var maxOtherBodyAcc = 500.0f;
                var maxOtherBodyForce = maxOtherBodyAcc * worldBody.Mass;

                if (force.LengthSquared() > maxOtherBodyForce * maxOtherBodyForce)
                    force *= maxOtherBodyForce / force.Length();

                worldBody.AddWorldForce(-force, groundPos);
            }

            return true;
        }

        public void Update(float dt)
        {
            if (dt <= 0.0f) return;

            var origAngVel = _angVel;
            _upSpeed = (Displacement - _lastDisplacement) / System.Math.Max(dt, JiggleMath.Epsilon);

            if (Lock)
            {
                _angVel = 0;
                _torque = 0;
            }
            else
            {
                _angVel += _torque * dt / _inertia;
                _torque = 0;


                if (origAngVel > _angVelForGrip && _angVel < _angVelForGrip || origAngVel < _angVelForGrip && _angVel > _angVelForGrip) _angVel = _angVelForGrip;

                _angVel += _driveTorque * dt / _inertia;
                _driveTorque = 0;

                float maxAngVel = 200;
                _angVel = MathHelper.Clamp(_angVel, -maxAngVel, maxAngVel);

                AxisAngle += MathHelper.ToDegrees(dt * _angVel);
            }
        }

        public float SteerAngle { get; set; }

        public bool Lock { get; set; }

        public void AddTorque(float torque)
        {
            _driveTorque += torque;
        }

        public Vector3 Pos { get; private set; }

        public Vector3 LocalAxisUp { get; private set; }

        public float Radius { get; private set; }

        public float Displacement { get; private set; }

        public float AxisAngle { get; private set; }

        public bool OnFloor { get; private set; }
    }
}