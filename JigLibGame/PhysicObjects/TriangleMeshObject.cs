using System;
using System.Collections.Generic;
using JigLibX.Collision;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame.PhysicObjects
{
    internal class TriangleMeshObject : PhysicObject
    {
        private readonly TriangleMesh triangleMesh;

        public TriangleMeshObject(Game game, Model model, Matrix orientation, Vector3 position) : base(game, model)
        {
            Body = new Body();
            Collision = new CollisionSkin(null);

            triangleMesh = new TriangleMesh();

            var vertexList = new List<Vector3>();
            var indexList = new List<TriangleVertexIndices>();

            ExtractData(vertexList, indexList, model);

            triangleMesh.CreateMesh(vertexList, indexList, 4, 1.0f);
            Collision.AddPrimitive(triangleMesh, new MaterialProperties(0.8f, 0.7f, 0.6f));
            PhysicsSystem.CurrentPhysicsSystem.CollisionSystem.AddCollisionSkin(Collision);


            Collision.ApplyLocalTransform(new JigLibX.Math.Transform(position, orientation));

            Body.MoveTo(position, orientation);
        }

        public void ExtractData(List<Vector3> vertices, List<TriangleVertexIndices> indices, Model model)
        {
            var bones = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(bones);
            foreach (var mm in model.Meshes)
            {
                var offset = vertices.Count;
                var xform = bones[mm.ParentBone.Index];
                foreach (var mmp in mm.MeshParts)
                {
                    var a = new Vector3[mmp.NumVertices];
                    var stride = mmp.VertexBuffer.VertexDeclaration.VertexStride;

                    mmp.VertexBuffer.GetData(mmp.VertexOffset * stride, a, 0, mmp.NumVertices, stride);

                    for (var i = 0; i != a.Length; ++i) Vector3.Transform(ref a[i], ref xform, out a[i]);
                    vertices.AddRange(a);


                    if (mmp.IndexBuffer.IndexElementSize != IndexElementSize.SixteenBits)
                        throw new Exception(string.Format("Model uses 32-bit indices, which are not supported."));

                    var s = new short[mmp.PrimitiveCount * 3];

                    mmp.IndexBuffer.GetData(mmp.StartIndex * 2, s, 0, mmp.PrimitiveCount * 3);

                    var tvi = new TriangleVertexIndices[mmp.PrimitiveCount];
                    for (var i = 0; i != tvi.Length; ++i)
                    {
                        tvi[i].I0 = s[i * 3 + 2] + offset;
                        tvi[i].I1 = s[i * 3 + 1] + offset;
                        tvi[i].I2 = s[i * 3 + 0] + offset;
                    }

                    indices.AddRange(tvi);
                }
            }
        }

        public override void ApplyEffects(BasicEffect effect)
        {
            effect.DiffuseColor = Vector3.One * 0.8f;
        }
    }
}