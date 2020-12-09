using System.IO;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content.Pipeline;
using Microsoft.Xna.Framework.Content.Pipeline.Graphics;
using Microsoft.Xna.Framework.Content.Pipeline.Processors;

namespace HeightmapProcessor
{
    [ContentProcessor]
    public class TerrainProcessor : ContentProcessor<Texture2DContent, ModelContent>
    {
        private const float TerrainScale = 1f;
        private const float TerrainBumpiness = 15;
        private const float TexCoordScale = 0.1f;
        private const string TerrainTexture = "Checker.bmp";

        public override ModelContent Process(Texture2DContent input, ContentProcessorContext context)
        {
            var builder = MeshBuilder.StartMesh("terrain");
            
            input.ConvertBitmapType(typeof(PixelBitmapContent<float>));
            
            var heightField = (PixelBitmapContent<float>) input.Mipmaps[0];
            for (var y = 0; y < heightField.Height; y++)
            for (var x = 0; x < heightField.Width; x++)
            {
                Vector3 position;
                position.X = TerrainScale * (x - (heightField.Width - 1) / 2.0f);
                position.Z = TerrainScale * (y - (heightField.Height - 1) / 2.0f);
                position.Y = (heightField.GetPixel(x, y) - 1) * TerrainBumpiness;
                builder.CreatePosition(position);
            }

            var material = new BasicMaterialContent {SpecularColor = new Vector3(.4f, .4f, .4f)};
            var directory = Path.GetDirectoryName(input.Identity.SourceFilename);
            var texture = Path.Combine(directory, TerrainTexture);

            material.Texture = new ExternalReference<TextureContent>(texture);
            builder.SetMaterial(material);
            
            var texCoordId = builder.CreateVertexChannel<Vector2>(VertexChannelNames.TextureCoordinate(0));

            for (var y = 0; y < heightField.Height - 1; y++)
            {
                for (var x = 0; x < heightField.Width - 1; x++)
                {
                    AddVertex(builder, texCoordId, heightField.Width, x, y);
                    AddVertex(builder, texCoordId, heightField.Width, x + 1, y);
                    AddVertex(builder, texCoordId, heightField.Width, x + 1, y + 1);

                    AddVertex(builder, texCoordId, heightField.Width, x, y);
                    AddVertex(builder, texCoordId, heightField.Width, x + 1, y + 1);
                    AddVertex(builder, texCoordId, heightField.Width, x, y + 1);
                }
            }

            var terrainMesh = builder.FinishMesh();
            var model = context.Convert<MeshContent, ModelContent>(terrainMesh, "ModelProcessor");
            model.Tag = new HeightMapInfoContent(heightField, TerrainScale, TerrainBumpiness);
            return model;
        }

        private static void AddVertex(MeshBuilder builder, int texCoordId, int w, int x, int y)
        {
            builder.SetVertexChannelData(texCoordId, new Vector2(x, y) * TexCoordScale);
            builder.AddTriangleVertex(x + y * w);
        }
    }
}