using Microsoft.Xna.Framework.Content.Pipeline;
using Microsoft.Xna.Framework.Content.Pipeline.Serialization.Compiler;

namespace HeightmapProcessor
{
    [ContentTypeWriter]
    public class HeightMapInfoWriter : ContentTypeWriter<HeightMapInfoContent>
    {
        protected override void Write(ContentWriter output, HeightMapInfoContent value)
        {
            output.Write(value.TerrainScale);

            output.Write(value.Height.GetLength(0));
            output.Write(value.Height.GetLength(1));
            foreach (var height in value.Height) output.Write(height);
        }

        public override string GetRuntimeType(TargetPlatform targetPlatform)
        {
            return "JigLibGame.HeightMapInfo, " + "JiggleGame, Version=1.0.0.0, Culture=neutral";
        }

        public override string GetRuntimeReader(TargetPlatform targetPlatform)
        {
            return "JigLibGame.HeightMapInfoReader, " + "JigLibGame, Version=1.0.0.0, Culture=neutral";
        }
    }
}