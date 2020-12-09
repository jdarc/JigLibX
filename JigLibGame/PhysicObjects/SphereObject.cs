using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public class SphereObject : PhysicObject
    {
        public SphereObject(Game game, Model model, float radius, Matrix orientation, Vector3 position) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(Body);
            Collision.AddPrimitive(new Sphere(Vector3.Zero * 5.0f, radius), new MaterialProperties(0.5f, 0.7f, 0.6f));
            Body.CollisionSkin = Collision;
            var com = SetMass(10.0f);
            Body.MoveTo(position + com, orientation);

            Body.EnableBody();
            Scale = Vector3.One * radius;
        }

        public override void ApplyEffects(BasicEffect effect)
        {
            effect.DiffuseColor = Color;
        }
    }
}