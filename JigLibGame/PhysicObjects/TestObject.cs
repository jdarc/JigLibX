using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public class TestObject : PhysicObject
    {
        public TestObject(Game game, Model model) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(Body);

            var boxMiddle = new Box(new Vector3(-3, 0, -0.5f), Matrix.Identity, new Vector3(6, 1, 1));
            var boxLeft = new Box(new Vector3(-3, -3f, -0.5f), Matrix.Identity, new Vector3(1, 4, 1));
            var boxRight = new Box(new Vector3(2, -3f, -0.5f), Matrix.Identity, new Vector3(1, 4, 1));

            Collision.AddPrimitive(boxMiddle, new MaterialProperties(0.2f, 0.7f, 0.6f));
            Collision.AddPrimitive(boxLeft, new MaterialProperties(0.2f, 0.7f, 0.6f));
            Collision.AddPrimitive(boxRight, new MaterialProperties(0.2f, 0.7f, 0.6f));

            Body.CollisionSkin = Collision;


            var com = SetMass(1.0f);


            Body.MoveTo(Vector3.Up * 10, Matrix.Identity);


            Body.EnableBody();
        }

        public override void ApplyEffects(BasicEffect effect)
        {
        }
    }
}