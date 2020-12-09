using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public class BoundingBoxHelper
    {
        public static BoundingBox InitialBox = new BoundingBox(new Vector3(float.PositiveInfinity), new Vector3(float.NegativeInfinity));

        public static void AddPoint(ref Vector3 pos, ref BoundingBox bb)
        {
            Vector3.Min(ref bb.Min, ref pos, out bb.Min);
            Vector3.Max(ref bb.Max, ref pos, out bb.Max);
        }

        public static void AddPoint(Vector3 pos, ref BoundingBox bb)
        {
            Vector3.Min(ref bb.Min, ref pos, out bb.Min);
            Vector3.Max(ref bb.Max, ref pos, out bb.Max);
        }

        private static Vector3[] pts = new Vector3[8];

        public static void AddBox(Box box, ref BoundingBox bb)
        {
            box.GetCornerPoints(out pts);

            AddPoint(ref pts[0], ref bb);
            AddPoint(ref pts[1], ref bb);
            AddPoint(ref pts[2], ref bb);
            AddPoint(ref pts[3], ref bb);
            AddPoint(ref pts[4], ref bb);
            AddPoint(ref pts[5], ref bb);
            AddPoint(ref pts[6], ref bb);
            AddPoint(ref pts[7], ref bb);
        }

        public static void AddSegment(Segment seg, ref BoundingBox bb)
        {
            AddPoint(seg.Origin, ref bb);
            AddPoint(seg.GetEnd(), ref bb);
        }

        public static void AddAABox(AABox aabox, ref BoundingBox bb)
        {
            bb.Min = Vector3.Min(aabox.MinPos, bb.Min);
            bb.Max = Vector3.Max(aabox.MaxPos, bb.Max);
        }

        public static void AddBBox(BoundingBox bbox, ref BoundingBox bb)
        {
            bb.Min = Vector3.Min(bbox.Min, bb.Min);
            bb.Max = Vector3.Max(bbox.Max, bb.Max);
        }

        public static void AddSphere(Sphere sphere, ref BoundingBox bb)
        {
            var radius = new Vector3(sphere.Radius);
            var minSphere = sphere.Position;
            var maxSphere = sphere.Position;

            Vector3.Subtract(ref minSphere, ref radius, out minSphere);
            Vector3.Add(ref maxSphere, ref radius, out maxSphere);

            Vector3.Min(ref bb.Min, ref minSphere, out bb.Min);
            Vector3.Max(ref bb.Max, ref maxSphere, out bb.Max);
        }

        public static void AddSphere(BoundingSphere sphere, ref BoundingBox bb)
        {
            var radius = new Vector3(sphere.Radius);
            var minSphere = sphere.Center;
            var maxSphere = sphere.Center;

            Vector3.Subtract(ref minSphere, ref radius, out minSphere);
            Vector3.Add(ref maxSphere, ref radius, out maxSphere);

            Vector3.Min(ref bb.Min, ref minSphere, out bb.Min);
            Vector3.Max(ref bb.Max, ref maxSphere, out bb.Max);
        }

        public static void AddCapsule(Capsule capsule, ref BoundingBox bb)
        {
            AddSphere(new BoundingSphere(capsule.Position, capsule.Radius), ref bb);
            AddSphere(new BoundingSphere(capsule.Position + capsule.Length * capsule.Orientation.Backward, capsule.Radius), ref bb);
        }

        public static void AddPrimitive(Primitive prim, ref BoundingBox bb)
        {
            switch ((PrimitiveType) prim.Type)
            {
                case PrimitiveType.Box:
                    AddBox((Box) prim, ref bb);
                    break;
                case PrimitiveType.Sphere:
                    AddSphere((Sphere) prim, ref bb);
                    break;
                case PrimitiveType.Capsule:
                    AddCapsule((Capsule) prim, ref bb);
                    break;
                default:
                    AddAABox(prim.GetBoundingBox(), ref bb);
                    break;
            }
        }

        public static bool OverlapTest(ref BoundingBox box0, ref BoundingBox box1)
        {
            return box0.Min.Z >= box1.Max.Z || box0.Max.Z <= box1.Min.Z || box0.Min.Y >= box1.Max.Y || box0.Max.Y <= box1.Min.Y || box0.Min.X >= box1.Max.X || box0.Max.X <= box1.Min.X ? false : true;
        }

        public static bool OverlapTest(ref BoundingBox box0, ref BoundingBox box1, float tol)
        {
            return box0.Min.Z >= box1.Max.Z + tol || box0.Max.Z <= box1.Min.Z - tol || box0.Min.Y >= box1.Max.Y + tol || box0.Max.Y <= box1.Min.Y - tol || box0.Min.X >= box1.Max.X + tol || box0.Max.X <= box1.Min.X - tol ? false : true;
        }
    }
}