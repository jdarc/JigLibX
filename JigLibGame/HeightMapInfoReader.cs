using Microsoft.Xna.Framework.Content;

namespace JigLibGame
{
    public class HeightMapInfoReader : ContentTypeReader<HeightMapInfo>
    {
        protected override HeightMapInfo Read(ContentReader input, HeightMapInfo existingInstance)
        {
            var terrainScale = input.ReadSingle();
            var width = input.ReadInt32();
            var height = input.ReadInt32();
            var heights = new float[width, height];

            for (var x = 0; x < width; x++)
            for (var z = 0; z < height; z++)
                heights[x, z] = input.ReadSingle();
            return new HeightMapInfo(heights, terrainScale);
        }
    }
}