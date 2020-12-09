using System;
using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public class CylinderObject : PhysicObject
    {
        public CylinderObject(Game game, float radius, float length, Vector3 position, Model model) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(Body);

            if (length - 2.0f * radius < 0.0f) throw new ArgumentException("Radius must be at least half length");

            var middle = new Capsule(Vector3.Zero, Matrix.Identity, radius, length - 2.0f * radius);

            var sideLength = 2.0f * radius / (float) Math.Sqrt(2.0d);

            var sides = new Vector3(-0.5f * sideLength, -0.5f * sideLength, -radius);

            var supply0 = new Box(sides, Matrix.Identity, new Vector3(sideLength, sideLength, length));

            var supply1 = new Box(Vector3.Transform(sides, Matrix.CreateRotationZ(MathHelper.PiOver4)), Matrix.CreateRotationZ(MathHelper.PiOver4), new Vector3(sideLength, sideLength, length));

            Collision.AddPrimitive(middle, new MaterialProperties(0.8f, 0.8f, 0.7f));
            Collision.AddPrimitive(supply0, new MaterialProperties(0.8f, 0.8f, 0.7f));
            Collision.AddPrimitive(supply1, new MaterialProperties(0.8f, 0.8f, 0.7f));

            Body.CollisionSkin = Collision;

            var com = SetMass(1.0f);
            Collision.ApplyLocalTransform(new Transform(-com, Matrix.Identity));

            var cylinderMass = Body.Mass;

            var comOffs = (length - 2.0f * radius) * 0.5f;

            var ixx = 0.5f * cylinderMass * radius * radius + cylinderMass * comOffs * comOffs;
            var iyy = 0.25f * cylinderMass * radius * radius + 1.0f / 12.0f * cylinderMass * length * length + cylinderMass * comOffs * comOffs;
            var izz = iyy;

            Body.SetBodyInertia(ixx, iyy, izz);

            Body.MoveTo(position, Matrix.CreateRotationX(MathHelper.PiOver2));

            Body.EnableBody();

            Scale = new Vector3(radius, radius, length * 0.5f);
        }

        public override void ApplyEffects(BasicEffect effect)
        {
            effect.DiffuseColor = Color;
        }
    }
}