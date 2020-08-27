using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using JigLibX.Collision;
using JigLibX.Physics;
using Plane = JigLibX.Geometry.Primitives.Plane;

namespace JiggleGame.PhysicObjects {
    internal class PlaneObject : PhysicObject {
        public PlaneObject(Game game, Model model, float d) : base(game, model) {
            body = new Body();
            collision = new CollisionSkin(null);
            collision.AddPrimitive(new Plane(Vector3.Up, d), new MaterialProperties(0.2f, 0.7f, 0.6f));
            PhysicsSystem.CurrentPhysicsSystem.CollisionSystem.AddCollisionSkin(collision);
        }

        public override void ApplyEffects(BasicEffect effect) {
            //
        }
    }
}