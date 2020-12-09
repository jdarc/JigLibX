using System;
using JigLibX.Collision;
using JigLibX.Geometry.Primitives;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    public abstract class PhysicObject : DrawableGameComponent
    {
        protected Body Body;
        protected CollisionSkin Collision;

        protected Model Model;
        protected Vector3 Color;

        protected Vector3 Scale = Vector3.One;

        public Body PhysicsBody => Body;

        public CollisionSkin PhysicsSkin => Collision;

        protected static Random Random = new Random();

        public PhysicObject(Game game, Model model) : base(game)
        {
            Model = model;
            Color = new Vector3(Random.Next(255), Random.Next(255), Random.Next(255));
            Color /= 255.0f;
        }

        public PhysicObject(Game game) : base(game)
        {
            Model = null;
            Color = new Vector3(Random.Next(255), Random.Next(255), Random.Next(255));
            Color /= 255.0f;
        }

        protected Vector3 SetMass(float mass)
        {
            var primitiveProperties = new PrimitiveProperties(PrimitiveProperties.MassDistributionEnum.Solid, PrimitiveProperties.MassTypeEnum.Density, mass);

            Collision.GetMassProperties(primitiveProperties, out var junk, out var com, out var it, out var itCoM);
            Body.BodyInertia = itCoM;
            Body.Mass = junk;

            return com;
        }

        private Matrix[] boneTransforms;
        private int boneCount;

        public abstract void ApplyEffects(BasicEffect effect);

        public override void Draw(GameTime gameTime)
        {
            if (Model != null)
            {
                if (boneTransforms == null || boneCount != Model.Bones.Count)
                {
                    boneTransforms = new Matrix[Model.Bones.Count];
                    boneCount = Model.Bones.Count;
                }

                Model.CopyAbsoluteBoneTransformsTo(boneTransforms);

                var camera = ((JiggleGame) Game).Camera;
                foreach (var mesh in Model.Meshes)
                {
                    foreach (BasicEffect effect in mesh.Effects)
                    {
                        if (Body.CollisionSkin != null)
                            effect.World = boneTransforms[mesh.ParentBone.Index] * Matrix.CreateScale(Scale) * Body.CollisionSkin.GetPrimitiveLocal(0).Transform.Orientation * Body.Orientation * Matrix.CreateTranslation(Body.Position);
                        else
                            effect.World = boneTransforms[mesh.ParentBone.Index] * Matrix.CreateScale(Scale) * Body.Orientation * Matrix.CreateTranslation(Body.Position);

                        effect.View = camera.View;
                        effect.Projection = camera.Projection;

                        ApplyEffects(effect);


                        effect.EnableDefaultLighting();
                        effect.PreferPerPixelLighting = true;
                    }

                    mesh.Draw();
                }
            }

            if (((JiggleGame) Game).DebugDrawer.Enabled)
            {
                wf = Collision.GetLocalSkinWireframe();


                if (Body.CollisionSkin != null) Body.TransformWireframe(wf);

                ((JiggleGame) Game).DebugDrawer.DrawShape(wf);
            }
        }

        private VertexPositionColor[] wf;
    }
}