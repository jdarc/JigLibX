using Microsoft.Xna.Framework;

namespace JigLibX.Geometry
{
    public struct Rectangle
    {
        public Vector3 Origin;

        public Vector3 Edge0;

        public Vector3 Edge1;

        public Rectangle(Vector3 origin, Vector3 edge0, Vector3 edge1)
        {
            Origin = origin;
            Edge0 = edge0;
            Edge1 = edge1;
        }

        public Vector3 GetPoint(float t0, float t1)
        {
            return Origin + t0 * Edge0 + t1 * Edge1;
        }
    }
}