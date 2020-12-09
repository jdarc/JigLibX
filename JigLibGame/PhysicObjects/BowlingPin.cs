using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public class BowlingPin : PhysicObject
    {
        public BowlingPin(Game game, Model model, Matrix orientation, Vector3 position) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(Body);


            Primitive capsule = new Capsule(Vector3.Zero, Matrix.Identity, 0.1f, 1.3f);

            Primitive box = new Box(new Vector3(-0.1f, -0.1f, -0.1f), Matrix.Identity, Vector3.One * 0.2f);

            Primitive sphere = new Sphere(new Vector3(0.0f, 0.0f, 0.3f), 0.3f);

            Collision.AddPrimitive(capsule, new MaterialProperties(0.1f, 0.5f, 0.5f));
            Collision.AddPrimitive(box, new MaterialProperties(0.1f, 0.5f, 0.5f));
            Collision.AddPrimitive(sphere, new MaterialProperties(0.1f, 0.5f, 0.5f));

            Body.CollisionSkin = Collision;
            var com = SetMass(0.5f);

            Body.MoveTo(position, orientation);
            Collision.ApplyLocalTransform(new Transform(-com, Matrix.Identity));

            Body.EnableBody();
            Scale = Vector3.One * 10.0f;
        }

        public override void ApplyEffects(BasicEffect effect)
        {
            effect.DiffuseColor = Color;
        }
    }
}