using Microsoft.Xna.Framework;
using JigLibX.Collision;
using JigLibX.Physics;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework.Graphics;

namespace JiggleGame.PhysicObjects {
    public class SphereObject : PhysicObject {
        public SphereObject(Game game, Model model, float radius, Matrix orientation, Vector3 position) : base(game, model) {
            body = new Body();
            collision = new CollisionSkin(body);
            collision.AddPrimitive(new Sphere(Vector3.Zero * 5.0f, radius), new MaterialProperties(0.5f, 0.7f, 0.6f));
            body.CollisionSkin = collision;
            Vector3 com = SetMass(10.0f);
            body.MoveTo(position + com, orientation);
            // collision.ApplyLocalTransform(new Transform(-com, Matrix.Identity));
            body.EnableBody();
            scale = Vector3.One * radius;
        }

        public override void ApplyEffects(BasicEffect effect) {
            effect.DiffuseColor = color;
        }
    }
}