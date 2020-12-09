using JigLibX.Vehicles;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    internal class CarObject : PhysicObject
    {
        private readonly Car car;
        private readonly Model wheel;

        public CarObject(Game game, Model model, Model wheels, bool fwDrive, bool rwDrive, float maxSteerAngle, float steerRate, float wheelSideFriction, float wheelFwdFriction, float wheelTravel, float wheelRadius, float wheelZOffset, float wheelRestingFrac, float wheelDampingFrac, int wheelNumRays, float driveTorque, float gravity) : base(game, model)
        {
            car = new Car(fwDrive, rwDrive, maxSteerAngle, steerRate, wheelSideFriction, wheelFwdFriction, wheelTravel, wheelRadius, wheelZOffset, wheelRestingFrac, wheelDampingFrac, wheelNumRays, driveTorque, gravity);

            Body = car.Chassis.Body;
            Collision = car.Chassis.Skin;
            wheel = wheels;

            SetCarMass(100.0f);
        }

        private void DrawWheel(Wheel wh, bool rotated)
        {
            var camera = ((JiggleGame) Game).Camera;

            foreach (var mesh in wheel.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    var steer = wh.SteerAngle;

                    Matrix rot;
                    if (rotated)
                        rot = Matrix.CreateRotationY(MathHelper.ToRadians(180.0f));
                    else
                        rot = Matrix.Identity;

                    effect.World = rot * Matrix.CreateRotationZ(MathHelper.ToRadians(-wh.AxisAngle)) * Matrix.CreateRotationY(MathHelper.ToRadians(steer)) * Matrix.CreateTranslation(wh.Pos + wh.Displacement * wh.LocalAxisUp) * car.Chassis.Body.Orientation * Matrix.CreateTranslation(car.Chassis.Body.Position);

                    effect.View = camera.View;
                    effect.Projection = camera.Projection;
                    effect.EnableDefaultLighting();
                    effect.PreferPerPixelLighting = true;
                }

                mesh.Draw();
            }
        }

        public override void Draw(GameTime gameTime)
        {
            DrawWheel(car.Wheels[0], true);
            DrawWheel(car.Wheels[1], true);
            DrawWheel(car.Wheels[2], false);
            DrawWheel(car.Wheels[3], false);

            base.Draw(gameTime);
        }

        public Car Car => car;

        private void SetCarMass(float mass)
        {
            Body.Mass = mass;
            car.Chassis.GetDims(out var min, out var max);
            var sides = max - min;

            var ixx = 1.0f / 12.0f * mass * (sides.Y * sides.Y + sides.Z * sides.Z);
            var iyy = 1.0f / 12.0f * mass * (sides.X * sides.X + sides.Z * sides.Z);
            var izz = 1.0f / 12.0f * mass * (sides.X * sides.X + sides.Y * sides.Y);

            var inertia = Matrix.Identity;
            inertia.M11 = ixx;
            inertia.M22 = iyy;
            inertia.M33 = izz;
            car.Chassis.Body.BodyInertia = inertia;
            car.SetupDefaultWheels();
        }

        public override void ApplyEffects(BasicEffect effect)
        {
        }
    }
}