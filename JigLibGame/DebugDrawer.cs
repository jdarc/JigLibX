using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JigLibGame
{
    public class DebugDrawer : DrawableGameComponent
    {
        private BasicEffect basicEffect;
        private readonly List<VertexPositionColor> vertexData;

        public DebugDrawer(Game game) : base(game)
        {
            vertexData = new List<VertexPositionColor>();
        }

        protected override void LoadContent()
        {
            base.LoadContent();
        }

        public override void Initialize()
        {
            base.Initialize();

            basicEffect = new BasicEffect(GraphicsDevice);
        }

        public override void Draw(GameTime gameTime)
        {
            if (vertexData.Count == 0 || !Enabled) return;

            var playGround = Game as JiggleGame;

            basicEffect.AmbientLightColor = Vector3.One;
            basicEffect.View = playGround.Camera.View;
            basicEffect.Projection = playGround.Camera.Projection;
            basicEffect.VertexColorEnabled = true;


            foreach (var pass in basicEffect.CurrentTechnique.Passes)
            {
                pass.Apply();

                GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineStrip, vertexData.ToArray(), 0, vertexData.Count - 1);
            }

            vertexData.Clear();

            base.Draw(gameTime);
        }

        public void DrawShape(List<Vector3> shape, Color color)
        {
            if (vertexData.Count > 0)
            {
                var v = vertexData[vertexData.Count - 1].Position;
                vertexData.Add(new VertexPositionColor(v, new Color(0, 0, 0, 0)));
                vertexData.Add(new VertexPositionColor(shape[0], new Color(0, 0, 0, 0)));
            }

            foreach (var p in shape) vertexData.Add(new VertexPositionColor(p, color));
        }

        public void DrawShape(List<Vector3> shape, Color color, bool closed)
        {
            DrawShape(shape, color);

            var v = shape[0];
            vertexData.Add(new VertexPositionColor(v, color));
        }

        public void DrawShape(List<VertexPositionColor> shape)
        {
            if (vertexData.Count > 0)
            {
                var v = vertexData[vertexData.Count - 1].Position;
                vertexData.Add(new VertexPositionColor(v, new Color(0, 0, 0, 0)));
                vertexData.Add(new VertexPositionColor(shape[0].Position, new Color(0, 0, 0, 0)));
            }

            foreach (var vps in shape) vertexData.Add(vps);
        }

        public void DrawShape(VertexPositionColor[] shape)
        {
            if (vertexData.Count > 0)
            {
                var v = vertexData[vertexData.Count - 1].Position;
                vertexData.Add(new VertexPositionColor(v, new Color(0, 0, 0, 0)));
                vertexData.Add(new VertexPositionColor(shape[0].Position, new Color(0, 0, 0, 0)));
            }

            foreach (var vps in shape) vertexData.Add(vps);
        }

        public void DrawShape(List<VertexPositionColor> shape, bool closed)
        {
            DrawShape(shape);

            var v = shape[0];
            vertexData.Add(v);
        }
    }
}