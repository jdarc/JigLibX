using Microsoft.Xna.Framework;

namespace JigLibX.Geometry
{
    public struct Line
    {
        public Vector3 Origin;

        public Vector3 Dir;

        public Line(Vector3 origin, Vector3 dir)
        {
            Origin = origin;
            Dir = dir;
        }

        public Vector3 GetOrigin(float t)
        {
            return new Vector3(Origin.X + t * Dir.X, Origin.Y + t * Dir.Y, Origin.Z + t * Dir.Z);
        }
    }
}