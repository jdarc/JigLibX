using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework;
using JigLibX.Math;
using Plane = JigLibX.Geometry.Primitives.Plane;

namespace JigLibX.Geometry
{
    public sealed class Intersection
    {
        public enum EdgesToTest
        {
            None = 0,

            Edge0 = 1 << 0,

            Edge1 = 1 << 1,

            Edge2 = 1 << 2,

            EdgeAll = Edge0 | Edge1 | Edge2
        }

        public enum CornersToTest
        {
            None = 0,

            Corner0 = 1 << 0,

            Corner1 = 1 << 1,

            Corner2 = 1 << 2,

            CornerAll = Corner0 | Corner1 | Corner2
        }

        private Intersection()
        {
        }

        public static bool LinePlaneIntersection(out float t, Line line, Plane plane)
        {
            var dot = Vector3.Dot(line.Dir, plane.Normal);

            if (System.Math.Abs(dot) < JiggleMath.Epsilon)
            {
                t = 0.0f;
                return false;
            }

            var dist = Distance.PointPlaneDistance(line.Origin, plane);
            t = -dist / dot;
            return true;
        }

        public static bool RayPlaneIntersection(out float t, Ray ray, Plane plane)
        {
            var dot = Vector3.Dot(ray.Dir, plane.Normal);
            if (System.Math.Abs(dot) < JiggleMath.Epsilon)
            {
                t = 0.0f;
                return false;
            }

            var dist = Distance.PointPlaneDistance(ray.Origin, plane);
            t = -dist / dot;
            return t >= 0.0f;
        }

        public static bool SegmentPlaneIntersection(out float tS, Segment seg, Plane plane)
        {
            var denom = Vector3.Dot(plane.Normal, seg.Delta);
            if (System.Math.Abs(denom) > JiggleMath.Epsilon)
            {
                var t = -(Vector3.Dot(plane.Normal, seg.Origin) + plane.D) / denom;
                if (t < 0.0f || t > 1.0f)
                {
                    tS = 0.0f;
                    return false;
                }

                tS = t;
                return true;
            }
            else
            {
                tS = 0.0f;
                return false;
            }
        }

        public static bool SweptSpherePlaneIntersection(out Vector3 pt, out float finalPenetration, BoundingSphere oldSphere, BoundingSphere newSphere, Vector3 planeNormal, float pOldDistToPlane, float pNewDistToPlane)
        {
            var oldDistToPlane = pOldDistToPlane;
            var newDistToPlane = pNewDistToPlane;
            var radius = oldSphere.Radius;

            pt = Vector3.Zero;
            finalPenetration = 0.0f;

            if (newDistToPlane >= oldDistToPlane) return false;
            if (newDistToPlane > radius) return false;


            var t = (newDistToPlane - radius) / (newDistToPlane - oldDistToPlane);
            if (t < 0.0f || t > 1.0f) return false;

            pt = oldSphere.Center + t * (newSphere.Center - oldSphere.Center) - MathHelper.Min(radius, oldDistToPlane) * planeNormal;
            finalPenetration = radius - newDistToPlane;
            return true;
        }

        public static bool SweptSphereTriangleIntersection(out Vector3 pt, out Vector3 N, out float depth, BoundingSphere oldSphere, BoundingSphere newSphere, Triangle triangle, float oldCentreDistToPlane, float newCentreDistToPlane, EdgesToTest edgesToTest, CornersToTest cornersToTest)
        {
            int i;
            var trianglePlane = triangle.Plane;
            N = Vector3.Zero;


            if (!SweptSpherePlaneIntersection(out pt, out depth, oldSphere, newSphere, trianglePlane.Normal, oldCentreDistToPlane, newCentreDistToPlane)) return false;

            var v0 = triangle.GetPoint(0);
            var v1 = triangle.GetPoint(1);
            var v2 = triangle.GetPoint(2);

            var e0 = v1 - v0;
            var e1 = v2 - v1;
            var e2 = v0 - v2;


            var allInside = true;
            var outDir0 = Vector3.Cross(e0, trianglePlane.Normal);
            if (Vector3.Dot(pt - v0, outDir0) > 0.0f) allInside = false;
            var outDir1 = Vector3.Cross(e1, trianglePlane.Normal);
            if (Vector3.Dot(pt - v1, outDir1) > 0.0f) allInside = false;
            var outDir2 = Vector3.Cross(e2, trianglePlane.Normal);
            if (Vector3.Dot(pt - v2, outDir2) > 0.0f) allInside = false;


            if (allInside)
            {
                N = trianglePlane.Normal;
                return true;
            }


            var bestT = float.MaxValue;
            var Ks = newSphere.Center - oldSphere.Center;
            var kss = Vector3.Dot(Ks, Ks);
            var radius = newSphere.Radius;
            var radiusSq = radius * radius;
            for (i = 0; i < 3; ++i)
            {
                var mask = 1 << i;
                if (!((mask != 0) & ((int) edgesToTest != 0)))
                    continue;
                Vector3 Ke;
                Vector3 vp;

                switch (i)
                {
                    case 0:
                        Ke = e0;
                        vp = v0;
                        break;
                    case 1:
                        Ke = e1;
                        vp = v1;
                        break;
                    case 2:
                    default:
                        Ke = e2;
                        vp = v2;
                        break;
                }

                var Kg = vp - oldSphere.Center;

                var kee = Vector3.Dot(Ke, Ke);
                if (System.Math.Abs(kee) < JiggleMath.Epsilon) continue;
                var kes = Vector3.Dot(Ke, Ks);
                var kgs = Vector3.Dot(Kg, Ks);
                var keg = Vector3.Dot(Ke, Kg);
                var kgg = Vector3.Dot(Kg, Kg);


                var a = kee * kss - kes * kes;
                if (System.Math.Abs(a) < JiggleMath.Epsilon) continue;
                var b = 2.0f * (keg * kes - kee * kgs);
                var c = kee * (kgg - radiusSq) - keg * keg;

                var blah = b * b - 4.0f * a * c;
                if (blah < 0.0f) continue;


                var t = (-b - (float) System.Math.Sqrt(blah)) / (2.0f * a);

                if (t < 0.0f || t > 1.0f) continue;

                if (t > bestT) continue;


                var Ct = oldSphere.Center + t * Ks;
                var d = Vector3.Dot(Ct - vp, Ke) / kee;

                if (d < 0.0f || d > 1.0f) continue;


                bestT = t;

                pt = vp + d * Ke;
                N = Ct - pt;
                JiggleMath.NormalizeSafe(ref N);
            }

            if (bestT <= 1.0f) return true;


            bestT = float.MaxValue;
            for (i = 0; i < 3; ++i)
            {
                var mask = 1 << i;
                if (!((mask != 0) & (cornersToTest != 0)))
                    continue;
                Vector3 vp;

                switch (i)
                {
                    case 0:
                        vp = v0;
                        break;
                    case 1:
                        vp = v1;
                        break;
                    case 2:
                    default:
                        vp = v2;
                        break;
                }

                var Kg = vp - oldSphere.Center;
                var kgs = Vector3.Dot(Kg, Ks);
                var kgg = Vector3.Dot(Kg, Kg);
                var a = kss;
                if (System.Math.Abs(a) < JiggleMath.Epsilon) continue;
                var b = -2.0f * kgs;
                var c = kgg - radiusSq;

                var blah = b * b - 4.0f * a * c;
                if (blah < 0.0f) continue;


                var t = (-b - (float) System.Math.Sqrt(blah)) / (2.0f * a);

                if (t < 0.0f || t > 1.0f) continue;

                if (t > bestT) continue;

                bestT = t;

                var Ct = oldSphere.Center + t * Ks;
                N = Ct - vp;
                JiggleMath.NormalizeSafe(ref N);
            }

            if (bestT <= 1.0f) return true;

            return false;
        }

        public static bool SegmentSphereIntersection(out float ts, Segment seg, Sphere sphere)
        {
            var r = seg.Delta;
            var s = seg.Origin - sphere.Position;

            var radiusSq = sphere.Radius * sphere.Radius;
            var rSq = r.LengthSquared();

            ts = float.MaxValue;

            if (rSq < radiusSq)
            {
                ts = 0.0f;
                return false;
            }

            var sDotr = Vector3.Dot(s, r);
            var sSq = s.LengthSquared();
            var sigma = sDotr * sDotr - rSq * (sSq - radiusSq);
            if (sigma < 0.0f) return false;
            var sigmaSqrt = (float) System.Math.Sqrt(sigma);
            var lambda1 = (-sDotr - sigmaSqrt) / rSq;
            var lambda2 = (-sDotr + sigmaSqrt) / rSq;
            if (lambda1 > 1.0f || lambda2 < 0.0f) return false;

            ts = MathHelper.Max(lambda1, 0.0f);
            return true;
        }

        public static bool SegmentCapsuleIntersection(out float tS, Segment seg, Capsule capsule)
        {
            var bestFrac = float.MaxValue;

            tS = 0;


            var sideFrac = float.MaxValue;
            if (!SegmentInfiniteCylinderIntersection(out sideFrac, seg, new Segment(capsule.Position, capsule.Orientation.Backward), capsule.Radius))
                return false;


            var sidePos = seg.GetPoint(sideFrac);
            if (Vector3.Dot(sidePos - capsule.Position, capsule.Orientation.Backward) < 0.0f)
                sideFrac = float.MaxValue;
            else if (Vector3.Dot(sidePos - capsule.GetEnd(), capsule.Orientation.Backward) > 0.0f)
                sideFrac = float.MaxValue;


            var originFrac = float.MaxValue;
            SegmentSphereIntersection(out originFrac, seg, new Sphere(capsule.Position, capsule.Radius));
            var endFrac = float.MaxValue;
            SegmentSphereIntersection(out endFrac, seg, new Sphere(capsule.GetEnd(), capsule.Radius));


            bestFrac = MathHelper.Min(sideFrac, originFrac);
            bestFrac = MathHelper.Min(bestFrac, endFrac);

            if (bestFrac <= 1.0f)
            {
                tS = bestFrac;
                return true;
            }

            return false;
        }

        public static bool SegmentInfiniteCylinderIntersection(out float tS, Segment seg, Segment cylinderAxis, float radius)
        {
            var Ks = seg.Delta;
            var kss = Vector3.Dot(Ks, Ks);
            var radiusSq = radius * radius;

            var Ke = cylinderAxis.Delta;
            var Kg = cylinderAxis.Origin - seg.Origin;

            tS = 0.0f;

            var kee = Vector3.Dot(Ke, Ke);
            if (System.Math.Abs(kee) < JiggleMath.Epsilon) return false;

            var kes = Vector3.Dot(Ke, Ks);
            var kgs = Vector3.Dot(Kg, Ks);
            var keg = Vector3.Dot(Ke, Kg);
            var kgg = Vector3.Dot(Kg, Kg);


            var distSq = (Kg - keg * Ke / kee).LengthSquared();
            if (distSq < radiusSq) return true;


            var a = kee * kss - kes * kes;
            if (System.Math.Abs(a) < JiggleMath.Epsilon) return false;

            var b = 2.0f * (keg * kes - kee * kgs);
            var c = kee * (kgg - radiusSq) - keg * keg;

            var blah = b * b - 4.0f * a * c;
            if (blah < 0.0f) return false;


            var t = (-b - (float) System.Math.Sqrt(blah)) / (2.0f * a);

            if (t < 0.0f || t > 1.0f) return false;

            tS = t;

            return true;
        }

        public static bool SegmentTriangleIntersection(out float tS, out float tT0, out float tT1, Segment seg, Triangle triangle)
        {
            float u, v, t;

            tS = 0;
            tT0 = 0;
            tT1 = 0;

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

            tS = t;
            tT0 = u;
            tT1 = v;


            return true;
        }
    }
}