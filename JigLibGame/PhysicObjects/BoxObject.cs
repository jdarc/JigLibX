using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public class BoxObject : PhysicObject
    {
        public BoxObject(Game game, Model model, Vector3 sideLengths, Matrix orientation, Vector3 position) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(Body);

            Collision.AddPrimitive(new Box(-0.5f * sideLengths, orientation, sideLengths), new MaterialProperties(0.8f, 0.8f, 0.7f));
            Body.CollisionSkin = Collision;
            var com = SetMass(1.0f);
            Body.MoveTo(position, Matrix.Identity);
            Collision.ApplyLocalTransform(new Transform(-com, Matrix.Identity));
            Body.EnableBody();
            Scale = sideLengths;
        }

        public override void ApplyEffects(BasicEffect effect)
        {
            effect.DiffuseColor = Color;
        }
    }
}