using Microsoft.Xna.Framework.Content.Pipeline.Graphics;

namespace HeightmapProcessor
{
    public class HeightMapInfoContent
    {
        public float[,] Height { get; }
        public float TerrainScale { get; }

        public HeightMapInfoContent(PixelBitmapContent<float> bitmap, float terrainScale, float terrainBumpiness)
        {
            TerrainScale = terrainScale;

            Height = new float[bitmap.Width, bitmap.Height];
            for (var y = 0; y < bitmap.Height; y++)
            {
                for (var x = 0; x < bitmap.Width; x++)
                {
                    Height[x, y] = (bitmap.GetPixel(x, y) - 1) * terrainBumpiness;
                }
            }
        }
    }
}