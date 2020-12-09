using Microsoft.Xna.Framework;

namespace JigLibX.Geometry
{
    public struct Segment
    {
        public Vector3 Origin;

        public Vector3 Delta;

        public Segment(Vector3 origin, Vector3 delta)
        {
            Origin = origin;
            Delta = delta;
        }

        public Segment(ref Vector3 origin, ref Vector3 delta)
        {
            Origin = origin;
            Delta = delta;
        }

        public void GetPoint(float t, out Vector3 point)
        {
            point = new Vector3(t * Delta.X, t * Delta.Y, t * Delta.Z);

            point.X += Origin.X;
            point.Y += Origin.Y;
            point.Z += Origin.Z;
        }

        public Vector3 GetPoint(float t)
        {
            var result = new Vector3(t * Delta.X, t * Delta.Y, t * Delta.Z);

            result.X += Origin.X;
            result.Y += Origin.Y;
            result.Z += Origin.Z;

            return result;
        }

        public void GetPoint(ref Vector3 point, float t)
        {
            point.X = t * Delta.X + Origin.X;
            point.Y = t * Delta.Y + Origin.Y;
            point.Z = t * Delta.Z + Origin.Z;
        }

        public Vector3 GetEnd()
        {
            return new Vector3(Delta.X + Origin.X, Delta.Y + Origin.Y, Delta.Z + Origin.Z);
        }
    }
}