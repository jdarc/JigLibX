using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework;
using JigLibX.Math;
using JigLibX.Utils;

namespace JigLibX.Geometry
{
    public sealed class Overlap
    {
        public static bool SegmentTriangleOverlap(Segment seg, Triangle triangle)
        {
            float u, v, t;

            var e1 = triangle.Edge0;
            var e2 = triangle.Edge1;

            var p = Vector3.Cross(seg.Delta, e2);
            var a = Vector3.Dot(e1, p);

            if (a > -JiggleMath.Epsilon && a < JiggleMath.Epsilon) return false;

            var f = 1.0f / a;
            var s = seg.Origin - triangle.Origin;
            u = f * Vector3.Dot(s, p);

            if (u < 0.0f || u > 1.0f) return false;

            var q = Vector3.Cross(s, e1);
            v = f * Vector3.Dot(seg.Delta, q);

            if (v < 0.0f || u + v > 1.0f) return false;

            t = f * Vector3.Dot(e2, q);

            if (t < 0.0f || t > 1.0f) return false;

            return true;
        }

        public static bool SegmentAABoxOverlap(Segment seg, AABox AABox)
        {
            var p0 = seg.Origin;
            var p1 = seg.GetEnd();

            var faceOffsets = new float[2];


            for (var iDir = 0; iDir < 3; iDir++)
            {
                var jDir = (iDir + 1) % 3;
                var kDir = (iDir + 2) % 3;


                faceOffsets[0] = JiggleUnsafe.Get(AABox.MinPos, iDir);
                faceOffsets[1] = JiggleUnsafe.Get(AABox.MaxPos, iDir);

                for (var iFace = 0; iFace < 2; iFace++)
                {
                    var dist0 = JiggleUnsafe.Get(ref p0, iDir) - faceOffsets[iFace];
                    var dist1 = JiggleUnsafe.Get(ref p1, iDir) - faceOffsets[iFace];
                    var frac = -1.0f;

                    if (dist0 * dist1 < -JiggleMath.Epsilon)
                        frac = -dist0 / (dist1 - dist0);
                    else if (System.Math.Abs(dist0) < JiggleMath.Epsilon)
                        frac = 0.0f;
                    else if (System.Math.Abs(dist1) < JiggleMath.Epsilon) frac = 1.0f;

                    if (frac >= 0.0f)
                    {
                        var pt = seg.GetPoint(frac);


                        if (JiggleUnsafe.Get(ref pt, jDir) > JiggleUnsafe.Get(AABox.MinPos, jDir) - JiggleMath.Epsilon && JiggleUnsafe.Get(ref pt, jDir) < JiggleUnsafe.Get(AABox.MaxPos, jDir) + JiggleMath.Epsilon && JiggleUnsafe.Get(ref pt, kDir) > JiggleUnsafe.Get(AABox.MinPos, kDir) - JiggleMath.Epsilon && JiggleUnsafe.Get(ref pt, kDir) < JiggleUnsafe.Get(AABox.MaxPos, kDir) + JiggleMath.Epsilon)
                            return true;
                    }
                }
            }

            return false;
        }
    }
}