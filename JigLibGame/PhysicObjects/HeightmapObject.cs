using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Physics;
using JigLibX.Utils;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public class HeightmapObject : PhysicObject
    {
        public HeightmapObject(Game game, Model model, Vector2 shift) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(null);

            var heightMapInfo = model.Tag as HeightMapInfo;
            var field = new Array2D(heightMapInfo.Heights.GetLength(0), heightMapInfo.Heights.GetLength(1));

            for (var x = 0; x < heightMapInfo.Heights.GetLength(0); x++)
            for (var z = 0; z < heightMapInfo.Heights.GetLength(1); z++)
                field.SetAt(x, z, heightMapInfo.Heights[x, z]);


            Body.MoveTo(new Vector3(shift.X, 0, shift.Y), Matrix.Identity);

            Collision.AddPrimitive(new Heightmap(field, shift.X, shift.Y, heightMapInfo.TerrainScale, heightMapInfo.TerrainScale), new MaterialProperties(0.7f, 0.7f, 0.6f));

            PhysicsSystem.CurrentPhysicsSystem.CollisionSystem.AddCollisionSkin(Collision);
        }

        public override void ApplyEffects(BasicEffect effect)
        {
            effect.PreferPerPixelLighting = true;
        }
    }
}