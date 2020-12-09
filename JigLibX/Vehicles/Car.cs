using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;

namespace JigLibX.Vehicles
{
    public class Car
    {
        private enum WheelId
        {
            WheelBr = 0,
            WheelFr = 1,
            WheelBl = 2,
            WheelFl = 3,
            MaxWheels = 4
        }

        private readonly float _maxSteerAngle;
        private readonly float _steerRate;
        private readonly float _wheelSideFriction;
        private readonly float _wheelFwdFriction;
        private readonly float _wheelTravel;
        private readonly float _wheelRadius;
        private readonly float _wheelZOffset;
        private readonly float _wheelRestingFrac;
        private readonly float _wheelDampingFrac;
        private readonly int _wheelNumRays;
        private readonly float _driveTorque;
        private readonly float _gravity;

        private float _steering;
        private float _accelerate;
        
        public bool BwDrive { get; set; }
        public bool FwDrive { get; set; }
        public Chassis Chassis { get; private set; }
        public List<Wheel> Wheels { get; private set; }
        public float Accelerate { get; set; }
        public float Steer { get; set; }
        public float HBrake { get; set; }
        public int NumWheelsOnFloor => Wheels.Count(t => t.OnFloor);

        public Car(bool fwDrive, bool rwDrive, float maxSteerAngle, float steerRate, float wheelSideFriction, float wheelFwdFriction, float wheelTravel, float wheelRadius, float wheelZOffset, float wheelRestingFrac, float wheelDampingFrac, int wheelNumRays, float driveTorque, float gravity)
        {
            FwDrive = fwDrive;
            BwDrive = rwDrive;
            _maxSteerAngle = maxSteerAngle;
            _steerRate = steerRate;
            _wheelSideFriction = wheelSideFriction;
            _wheelFwdFriction = wheelFwdFriction;
            _wheelTravel = wheelTravel;
            _wheelRadius = wheelRadius;
            _wheelZOffset = wheelZOffset;
            _wheelRestingFrac = wheelRestingFrac;
            _wheelDampingFrac = wheelDampingFrac;
            _wheelNumRays = wheelNumRays;
            _driveTorque = driveTorque;
            _gravity = gravity;

            Chassis = new Chassis(this);

            SetupDefaultWheels();
        }

        public void SetupDefaultWheels()
        {
            if (Chassis == null) return;

            Chassis.GetDims(out var min, out var max);

            var mass = Chassis.Body.Mass;
            var mass4 = 0.25f * mass;

            var axis = Vector3.Up;
            
            var spring = mass4 * _gravity / (_wheelRestingFrac * _wheelTravel);

            var wheelMass = 0.03f * mass;
            var inertia = 0.5f * (_wheelRadius * _wheelRadius) * wheelMass;
            
            var damping = 2.0f * (float) System.Math.Sqrt(spring * mass);
            damping *= 0.25f;
            damping *= _wheelDampingFrac;


            min.X += 3.0f * _wheelRadius;
            max.X -= 3.1f * _wheelRadius;
            min.Z += _wheelRadius * 0.35f;
            max.Z -= _wheelRadius * 0.35f;

            var delta = max - min;

            min.Y += _wheelZOffset;

            var brPos = min;
            var frPos = min + new Vector3(delta.X, 0.0f, 0.0f);
            var blPos = min + new Vector3(0.0f, 0.0f, delta.Z);
            var flPos = min + new Vector3(delta.X, 0.0f, delta.Z);

            Wheels ??= new List<Wheel>();

            if (Wheels.Count == 0)
            {
                Wheels.Add(new Wheel());
                Wheels.Add(new Wheel());
                Wheels.Add(new Wheel());
                Wheels.Add(new Wheel());
            }


            Wheels[(int) WheelId.WheelBr].Setup(this, brPos, axis, spring, _wheelTravel, inertia, _wheelRadius, _wheelSideFriction, _wheelFwdFriction, damping, _wheelNumRays);
            Wheels[(int) WheelId.WheelFr].Setup(this, frPos, axis, spring, _wheelTravel, inertia, _wheelRadius, _wheelSideFriction, _wheelFwdFriction, damping, _wheelNumRays);
            Wheels[(int) WheelId.WheelBl].Setup(this, blPos, axis, spring, _wheelTravel, inertia, _wheelRadius, _wheelSideFriction, _wheelFwdFriction, damping, _wheelNumRays);
            Wheels[(int) WheelId.WheelFl].Setup(this, flPos, axis, spring, _wheelTravel, inertia, _wheelRadius, _wheelSideFriction, _wheelFwdFriction, damping, _wheelNumRays);
        }

        public void EnableCar() => Chassis?.EnableChassis();

        public void DisableCar() => Chassis?.DisableChassis();

        public void AddExternalForces(float dt)
        {
            foreach (var t in Wheels) t.AddForcesToCar(dt);
        }

        public void PostPhysics(float dt)
        {
            foreach (var t in Wheels) t.Update(dt);

            var deltaAccelerate = dt * 4.0f;
            var deltaSteering = dt * _steerRate;
            
            var dAccelerate = Accelerate - _accelerate;
            dAccelerate = MathHelper.Clamp(dAccelerate, -deltaAccelerate, deltaAccelerate);

            _accelerate += dAccelerate;

            var dSteering = Steer - _steering;
            dSteering = MathHelper.Clamp(dSteering, -deltaSteering, deltaSteering);

            _steering += dSteering;


            var maxTorque = _driveTorque;

            if (FwDrive && BwDrive) maxTorque *= 0.5f;

            if (FwDrive)
            {
                Wheels[(int) WheelId.WheelFl].AddTorque(maxTorque * _accelerate);
                Wheels[(int) WheelId.WheelFr].AddTorque(maxTorque * _accelerate);
            }

            if (BwDrive)
            {
                Wheels[(int) WheelId.WheelBl].AddTorque(maxTorque * _accelerate);
                Wheels[(int) WheelId.WheelBr].AddTorque(maxTorque * _accelerate);
            }

            Wheels[(int) WheelId.WheelBl].Lock = HBrake > 0.5f;
            Wheels[(int) WheelId.WheelBr].Lock = HBrake > 0.5f;


            int inner, outer;

            if (_steering > 0.0f)
            {
                inner = (int) WheelId.WheelFl;
                outer = (int) WheelId.WheelFr;
            }
            else
            {
                inner = (int) WheelId.WheelFr;
                outer = (int) WheelId.WheelFl;
            }

            var alpha = System.Math.Abs(_maxSteerAngle * _steering);
            var angleSgn = _steering > 0.0f ? 1.0f : -1.0f;

            Wheels[inner].SteerAngle = angleSgn * alpha;

            float beta;

            if (alpha == 0.0f)
            {
                beta = alpha;
            }
            else
            {
                var dx = Wheels[(int) WheelId.WheelFr].Pos.X - Wheels[(int) WheelId.WheelBr].Pos.X;
                var dy = Wheels[(int) WheelId.WheelFl].Pos.Z - Wheels[(int) WheelId.WheelFr].Pos.Z;

                beta = (float) System.Math.Atan2(MathHelper.ToRadians(dy), MathHelper.ToRadians(dx + dy / (float) System.Math.Tan(MathHelper.ToRadians(alpha))));
                beta = MathHelper.ToDegrees(beta);
            }

            Wheels[outer].SteerAngle = angleSgn * beta;
        }
    }
}