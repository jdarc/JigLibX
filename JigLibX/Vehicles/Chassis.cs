using Microsoft.Xna.Framework;
using JigLibX.Collision;
using JigLibX.Physics;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;

namespace JigLibX.Vehicles
{
    public sealed class Chassis
    {
        private readonly ChassisBody _body;

        private Vector3 _dimsMin;
        private Vector3 _dimsMax;

        public Chassis(Car car)
        {
            _body = new ChassisBody(car);
            Skin = new CollisionSkin(_body);

            _body.CollisionSkin = Skin;

            const float length = 6.0f;
            const float width = 2.3f;
            const float height = 1.6f;

            var min = new Vector3(-0.5f * length, 0.0f, -width * 0.5f);
            var max = new Vector3(0.5f * length, height, width * 0.5f);

            SetDims(min, max);
        }

        public void SetDims(Vector3 min, Vector3 max)
        {
            _dimsMin = min;
            _dimsMax = max;
            var sides = max - min;
            var topBotRatio = 0.4f;
            
            var max1 = max;
            max1.Y -= topBotRatio * sides.Y;
            var box1 = new Box(min, Matrix.Identity, max1 - min);
            
            var min2 = min;
            min2.Y += topBotRatio * sides.Y;
            var max2 = max;
            min2.X += sides.X * 0.05f;
            max2.X -= sides.X * 0.3f;
            min2.Z *= 0.9f;
            max2.Z *= 0.9f;

            var box2 = new Box(min2, Matrix.Identity, max2 - min2);

            Skin.RemoveAllPrimitives();
            Skin.AddPrimitive(box1, new MaterialProperties(0.3f, 0.5f, 0.3f));
            Skin.AddPrimitive(box2, new MaterialProperties(0.3f, 0.5f, 0.3f));

            _body.Car.SetupDefaultWheels();
        }

        public void GetDims(out Vector3 min, out Vector3 max)
        {
            min = _dimsMin;
            max = _dimsMax;
        }

        public void EnableChassis() => _body.EnableBody();

        public void DisableChassis() => _body.DisableBody();

        public Body Body => _body;

        public CollisionSkin Skin { get; }
    }
}