using JigLibX.Collision;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Plane = JigLibX.Geometry.Primitives.Plane;

namespace JigLibGame.PhysicObjects
{
    internal class PlaneObject : PhysicObject
    {
        public PlaneObject(Game game, Model model, float d) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(null);
            Collision.AddPrimitive(new Plane(Vector3.Up, d), new MaterialProperties(0.2f, 0.7f, 0.6f));
            PhysicsSystem.CurrentPhysicsSystem.CollisionSystem.AddCollisionSkin(Collision);
        }

        public override void ApplyEffects(BasicEffect effect)
        {
        }
    }
}