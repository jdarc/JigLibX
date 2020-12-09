using System;
using Microsoft.Xna.Framework;

namespace JigLibGame
{
    public class HeightMapInfo
    {
        public float TerrainScale;

        public float[,] Heights;

        private readonly Vector3 heightmapPosition;

        private readonly float heightmapWidth;

        private readonly float heightmapHeight;

        public HeightMapInfo(float[,] heights, float terrainScale)
        {
            if (heights == null) throw new ArgumentNullException("heights");

            TerrainScale = terrainScale;
            Heights = heights;

            heightmapWidth = (heights.GetLength(0) - 1) * terrainScale;
            heightmapHeight = (heights.GetLength(1) - 1) * terrainScale;

            heightmapPosition.X = -(heights.GetLength(0) - 1) / 2 * terrainScale;
            heightmapPosition.Z = -(heights.GetLength(1) - 1) / 2 * terrainScale;
        }

        public bool IsOnHeightmap(Vector3 position)
        {
            var positionOnHeightmap = position - heightmapPosition;


            return positionOnHeightmap.X > 0 && positionOnHeightmap.X < heightmapWidth && positionOnHeightmap.Z > 0 && positionOnHeightmap.Z < heightmapHeight;
        }

        public float GetHeight(Vector3 position)
        {
            var positionOnHeightmap = position - heightmapPosition;


            int left, top;
            left = (int) positionOnHeightmap.X / (int) TerrainScale;
            top = (int) positionOnHeightmap.Z / (int) TerrainScale;


            var xNormalized = positionOnHeightmap.X % TerrainScale / TerrainScale;
            var zNormalized = positionOnHeightmap.Z % TerrainScale / TerrainScale;


            var topHeight = MathHelper.Lerp(Heights[left, top], Heights[left + 1, top], xNormalized);

            var bottomHeight = MathHelper.Lerp(Heights[left, top + 1], Heights[left + 1, top + 1], xNormalized);


            return MathHelper.Lerp(topHeight, bottomHeight, zNormalized);
        }
    }
}