using JigLibX.Physics;

namespace JigLibX.Vehicles
{
    public class ChassisBody : Body
    {
        public ChassisBody(Car car) => Car = car;

        public override void AddExternalForces(float dt)
        {
            if (Car == null) return;
            ClearForces();
            AddGravityToExternalForce();
            Car.AddExternalForces(dt);
        }

        public override void PostPhysics(float dt) => Car?.PostPhysics(dt);

        public Car Car { get; }
    }
}