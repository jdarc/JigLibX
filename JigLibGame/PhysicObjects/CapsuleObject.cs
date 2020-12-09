using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public class CapsuleObject : PhysicObject
    {
        public CapsuleObject(Game game, Model model, float radius, float length, Matrix orientation, Vector3 position) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(Body);
            Collision.AddPrimitive(new Capsule(Vector3.Transform(new Vector3(-0.5f, 0, 0), orientation), orientation, radius, length), (int) MaterialTable.MaterialID.BouncyNormal);
            Body.CollisionSkin = Collision;
            var com = SetMass(10.0f);
            Body.MoveTo(position + com, Matrix.Identity);

            Collision.ApplyLocalTransform(new Transform(-com, Matrix.Identity));

            Body.EnableBody();
            Scale = new Vector3(radius, radius, length / 2);
        }

        public override void ApplyEffects(BasicEffect effect)
        {
            effect.DiffuseColor = Color;
        }
    }
}