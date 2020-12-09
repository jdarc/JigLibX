using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework;
using JigLibX.Math;
using Plane = JigLibX.Geometry.Primitives.Plane;

namespace JigLibX.Geometry
{
    public static class Distance
    {
        public static float SegmentTriangleDistanceSq(out float segT, out float triT0, out float triT1, Segment seg, Triangle triangle)
        {
            var distSq = float.MaxValue;

            if (Intersection.SegmentTriangleIntersection(out segT, out triT0, out triT1, seg, triangle))
            {
                segT = triT0 = triT1 = 0.0f;
                return 0.0f;
            }

            float distEdgeSq;
            distEdgeSq = SegmentSegmentDistanceSq(out var s, out var t, seg, new Segment(triangle.Origin, triangle.Edge0));
            if (distEdgeSq < distSq)
            {
                distSq = distEdgeSq;
                segT = s;
                triT0 = t;
                triT1 = 0.0f;
            }

            distEdgeSq = SegmentSegmentDistanceSq(out s, out t, seg, new Segment(triangle.Origin, triangle.Edge1));
            if (distEdgeSq < distSq)
            {
                distSq = distEdgeSq;
                segT = s;
                triT0 = 0.0f;
                triT1 = t;
            }

            distEdgeSq = SegmentSegmentDistanceSq(out s, out t, seg, new Segment(triangle.Origin + triangle.Edge0, triangle.Edge2));
            if (distEdgeSq < distSq)
            {
                distSq = distEdgeSq;
                segT = s;
                triT0 = 1.0f - t;
                triT1 = t;
            }


            var startTriSq = PointTriangleDistanceSq(out t, out var u, seg.Origin, triangle);
            if (startTriSq < distSq)
            {
                distSq = startTriSq;
                segT = 0.0f;
                triT0 = t;
                triT1 = u;
            }

            var endTriSq = PointTriangleDistanceSq(out t, out u, seg.GetEnd(), triangle);
            if (endTriSq < distSq)
            {
                distSq = endTriSq;
                segT = 1.0f;
                triT0 = t;
                triT1 = u;
            }

            return distSq;
        }

        public static float PointTriangleDistanceSq(out float pfSParam, out float pfTParam, Vector3 rkPoint, Triangle rkTri)
        {
            var kDiff = rkTri.Origin - rkPoint;
            var fA00 = rkTri.Edge0.LengthSquared();
            var fA01 = Vector3.Dot(rkTri.Edge0, rkTri.Edge1);
            var fA11 = rkTri.Edge1.LengthSquared();
            var fB0 = Vector3.Dot(kDiff, rkTri.Edge0);
            var fB1 = Vector3.Dot(kDiff, rkTri.Edge1);
            var fC = kDiff.LengthSquared();
            var fDet = System.Math.Abs(fA00 * fA11 - fA01 * fA01);
            var fS = fA01 * fB1 - fA11 * fB0;
            var fT = fA01 * fB0 - fA00 * fB1;
            float fSqrDist;

            if (fS + fT <= fDet)
            {
                if (fS < 0.0f)
                {
                    if (fT < 0.0f)
                    {
                        if (fB0 < 0.0f)
                        {
                            fT = 0.0f;
                            if (-fB0 >= fA00)
                            {
                                fS = 1.0f;
                                fSqrDist = fA00 + 2.0f * fB0 + fC;
                            }
                            else
                            {
                                fS = -fB0 / fA00;
                                fSqrDist = fB0 * fS + fC;
                            }
                        }
                        else
                        {
                            fS = 0.0f;
                            if (fB1 >= 0.0f)
                            {
                                fT = 0.0f;
                                fSqrDist = fC;
                            }
                            else if (-fB1 >= fA11)
                            {
                                fT = 1.0f;
                                fSqrDist = fA11 + 2.0f * fB1 + fC;
                            }
                            else
                            {
                                fT = -fB1 / fA11;
                                fSqrDist = fB1 * fT + fC;
                            }
                        }
                    }
                    else
                    {
                        fS = 0.0f;
                        if (fB1 >= 0.0f)
                        {
                            fT = 0.0f;
                            fSqrDist = fC;
                        }
                        else if (-fB1 >= fA11)
                        {
                            fT = 1.0f;
                            fSqrDist = fA11 + 2.0f * fB1 + fC;
                        }
                        else
                        {
                            fT = -fB1 / fA11;
                            fSqrDist = fB1 * fT + fC;
                        }
                    }
                }
                else if (fT < 0.0f)
                {
                    fT = 0.0f;
                    if (fB0 >= 0.0f)
                    {
                        fS = 0.0f;
                        fSqrDist = fC;
                    }
                    else if (-fB0 >= fA00)
                    {
                        fS = 1.0f;
                        fSqrDist = fA00 + 2.0f * fB0 + fC;
                    }
                    else
                    {
                        fS = -fB0 / fA00;
                        fSqrDist = fB0 * fS + fC;
                    }
                }
                else
                {
                    var fInvDet = 1.0f / fDet;
                    fS *= fInvDet;
                    fT *= fInvDet;
                    fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) + fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
                }
            }
            else
            {
                float fTmp0, fTmp1, fNumer, fDenom;

                if (fS < 0.0f)
                {
                    fTmp0 = fA01 + fB0;
                    fTmp1 = fA11 + fB1;
                    if (fTmp1 > fTmp0)
                    {
                        fNumer = fTmp1 - fTmp0;
                        fDenom = fA00 - 2.0f * fA01 + fA11;
                        if (fNumer >= fDenom)
                        {
                            fS = 1.0f;
                            fT = 0.0f;
                            fSqrDist = fA00 + 2.0f * fB0 + fC;
                        }
                        else
                        {
                            fS = fNumer / fDenom;
                            fT = 1.0f - fS;
                            fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) + fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
                        }
                    }
                    else
                    {
                        fS = 0.0f;
                        if (fTmp1 <= 0.0f)
                        {
                            fT = 1.0f;
                            fSqrDist = fA11 + 2.0f * fB1 + fC;
                        }
                        else if (fB1 >= 0.0f)
                        {
                            fT = 0.0f;
                            fSqrDist = fC;
                        }
                        else
                        {
                            fT = -fB1 / fA11;
                            fSqrDist = fB1 * fT + fC;
                        }
                    }
                }
                else if (fT < 0.0f)
                {
                    fTmp0 = fA01 + fB1;
                    fTmp1 = fA00 + fB0;
                    if (fTmp1 > fTmp0)
                    {
                        fNumer = fTmp1 - fTmp0;
                        fDenom = fA00 - 2.0f * fA01 + fA11;
                        if (fNumer >= fDenom)
                        {
                            fT = 1.0f;
                            fS = 0.0f;
                            fSqrDist = fA11 + 2.0f * fB1 + fC;
                        }
                        else
                        {
                            fT = fNumer / fDenom;
                            fS = 1.0f - fT;
                            fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) + fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
                        }
                    }
                    else
                    {
                        fT = 0.0f;
                        if (fTmp1 <= 0.0f)
                        {
                            fS = 1.0f;
                            fSqrDist = fA00 + 2.0f * fB0 + fC;
                        }
                        else if (fB0 >= 0.0f)
                        {
                            fS = 0.0f;
                            fSqrDist = fC;
                        }
                        else
                        {
                            fS = -fB0 / fA00;
                            fSqrDist = fB0 * fS + fC;
                        }
                    }
                }
                else
                {
                    fNumer = fA11 + fB1 - fA01 - fB0;
                    if (fNumer <= 0.0f)
                    {
                        fS = 0.0f;
                        fT = 1.0f;
                        fSqrDist = fA11 + 2.0f * fB1 + fC;
                    }
                    else
                    {
                        fDenom = fA00 - 2.0f * fA01 + fA11;
                        if (fNumer >= fDenom)
                        {
                            fS = 1.0f;
                            fT = 0.0f;
                            fSqrDist = fA00 + 2.0f * fB0 + fC;
                        }
                        else
                        {
                            fS = fNumer / fDenom;
                            fT = 1.0f - fS;
                            fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) + fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
                        }
                    }
                }
            }

            pfSParam = fS;
            pfTParam = fT;

            return System.Math.Abs(fSqrDist);
        }

        public static float PointPlaneDistance(Vector3 pt, Plane plane)
        {
            return Vector3.Dot(plane.Normal, pt) + plane.D;
        }

        public static float PointPlaneDistance(ref Vector3 pt, Plane plane)
        {
            Vector3.Dot(ref plane.normal, ref pt, out var num0);
            return plane.D + num0;
        }

        public static float PointPlaneDistance(Vector3 pt, Vector3 planeNormal, float planeD)
        {
            return Vector3.Dot(planeNormal, pt) + planeD;
        }

        public static float PointPlaneDistance(ref Vector3 pt, ref Vector3 planeNormal, float planeD)
        {
            Vector3.Dot(ref planeNormal, ref pt, out var num0);
            return num0 + planeD;
        }

        public static float PointPointDistanceSq(Vector3 pt1, Vector3 pt2)
        {
            var num3 = pt1.X - pt2.X;
            var num2 = pt1.Y - pt2.Y;
            var num0 = pt1.Z - pt2.Z;
            return num3 * num3 + num2 * num2 + num0 * num0;
        }

        public static void PointPointDistanceSq(ref Vector3 pt1, ref Vector3 pt2, out float result)
        {
            var num3 = pt1.X - pt2.X;
            var num2 = pt1.Y - pt2.Y;
            var num0 = pt1.Z - pt2.Z;
            result = num3 * num3 + num2 * num2 + num0 * num0;
        }

        public static float PointPointDistance(Vector3 pt1, Vector3 pt2)
        {
            var num3 = pt1.X - pt2.X;
            var num2 = pt1.Y - pt2.Y;
            var num0 = pt1.Z - pt2.Z;
            var num4 = num3 * num3 + num2 * num2 + num0 * num0;
            return (float) System.Math.Sqrt(num4);
        }

        public static void PointPointDistance(ref Vector3 pt1, ref Vector3 pt2, out float result)
        {
            var num3 = pt1.X - pt2.X;
            var num2 = pt1.Y - pt2.Y;
            var num0 = pt1.Z - pt2.Z;
            var num4 = num3 * num3 + num2 * num2 + num0 * num0;
            result = (float) System.Math.Sqrt(num4);
        }

        public static float PointSegmentDistanceSq(out float t, Vector3 pt, Segment seg)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            t = fT;

            return kDiff.LengthSquared();
        }

        public static float PointSegmentDistanceSq(Vector3 pt, Segment seg)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            return kDiff.LengthSquared();
        }

        public static void PointSegmentDistanceSq(out float t, out float result, ref Vector3 pt, ref Segment seg)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            t = fT;

            result = kDiff.LengthSquared();
        }

        public static void PointSegmentDistanceSq(ref Vector3 pt, ref Segment seg, out float result)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            result = kDiff.LengthSquared();
        }

        public static float PointSegmentDistance(out float t, Vector3 pt, Segment seg)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            t = fT;

            return kDiff.Length();
        }

        public static float PointSegmentDistance(Vector3 pt, Segment seg)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            return kDiff.Length();
        }

        public static void PointSegmentDistance(out float t, out float result, ref Vector3 pt, ref Segment seg)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            t = fT;

            result = kDiff.Length();
        }

        public static void PointSegmentDistance(ref Vector3 pt, ref Segment seg, out float result)
        {
            Vector3.Subtract(ref pt, ref seg.Origin, out var kDiff);
            Vector3.Dot(ref kDiff, ref seg.Delta, out var fT);

            if (fT <= 0.0f)
            {
                fT = 0.0f;
            }
            else
            {
                var sqrLen = seg.Delta.LengthSquared();
                if (fT >= sqrLen)
                {
                    fT = 1.0f;
                    kDiff -= seg.Delta;
                }
                else
                {
                    fT /= sqrLen;
                    kDiff -= fT * seg.Delta;
                }
            }

            result = kDiff.Length();
        }

        public static float SegmentBoxDistanceSq(out float pfLParam, out float pfBParam0, out float pfBParam1, out float pfBParam2, Segment rkSeg, Box rkBox)
        {
            pfLParam = pfBParam0 = pfBParam1 = pfBParam2 = 0.0f;

            var line = new Line(rkSeg.Origin, rkSeg.Delta);

            var fSqrDistance = SqrDistance(line, rkBox, out var fLP, out var fBP0, out var fBP1, out var fBP2);

            if (fLP >= 0.0f)
            {
                if (fLP <= 1.0f)
                {
                    pfLParam = fLP;
                    pfBParam0 = fBP0;
                    pfBParam1 = fBP1;
                    pfBParam2 = fBP2;

                    return MathHelper.Max(fSqrDistance, 0.0f);
                }
                else
                {
                    fSqrDistance = SqrDistance(rkSeg.Origin + rkSeg.Delta, rkBox, out pfBParam0, out pfBParam1, out pfBParam2);

                    pfLParam = 1.0f;
                    return MathHelper.Max(fSqrDistance, 0.0f);
                }
            }
            else
            {
                fSqrDistance = SqrDistance(rkSeg.Origin, rkBox, out pfBParam0, out pfBParam1, out pfBParam2);

                pfLParam = 0.0f;

                return MathHelper.Max(fSqrDistance, 0.0f);
            }
        }

        public static float SqrDistance(Vector3 point, Box box, out float pfBParam0, out float pfBParam1, out float pfBParam2)
        {
            var kDiff = point - box.GetCentre();
            var kClosest = new Vector3(Vector3.Dot(kDiff, box.Orientation.Right), Vector3.Dot(kDiff, box.Orientation.Up), Vector3.Dot(kDiff, box.Orientation.Backward));


            var fSqrDistance = 0.0f;
            float fDelta;

            if (kClosest.X < -box.GetHalfSideLengths().X)
            {
                fDelta = kClosest.X + box.GetHalfSideLengths().X;
                fSqrDistance += fDelta * fDelta;
                kClosest.X = -box.GetHalfSideLengths().X;
            }
            else if (kClosest.X > box.GetHalfSideLengths().X)
            {
                fDelta = kClosest.X - box.GetHalfSideLengths().X;
                fSqrDistance += fDelta * fDelta;
                kClosest.X = box.GetHalfSideLengths().X;
            }

            if (kClosest.Y < -box.GetHalfSideLengths().Y)
            {
                fDelta = kClosest.Y + box.GetHalfSideLengths().Y;
                fSqrDistance += fDelta * fDelta;
                kClosest.Y = -box.GetHalfSideLengths().Y;
            }
            else if (kClosest.Y > box.GetHalfSideLengths().Y)
            {
                fDelta = kClosest.Y - box.GetHalfSideLengths().Y;
                fSqrDistance += fDelta * fDelta;
                kClosest.Y = box.GetHalfSideLengths().Y;
            }

            if (kClosest.Z < -box.GetHalfSideLengths().Z)
            {
                fDelta = kClosest.Z + box.GetHalfSideLengths().Z;
                fSqrDistance += fDelta * fDelta;
                kClosest.Z = -box.GetHalfSideLengths().Z;
            }
            else if (kClosest.Z > box.GetHalfSideLengths().Z)
            {
                fDelta = kClosest.Z - box.GetHalfSideLengths().Z;
                fSqrDistance += fDelta * fDelta;
                kClosest.Z = box.GetHalfSideLengths().Z;
            }


            pfBParam0 = kClosest.X;
            pfBParam1 = kClosest.Y;
            pfBParam2 = kClosest.Z;

            return MathHelper.Max(fSqrDistance, 0.0f);
        }

        public static float SqrDistance(Line line, Box box, out float pfLParam, out float pfBParam0, out float pfBParam1, out float pfBParam2)
        {
            var diff = line.Origin - box.GetCentre();
            var pnt = new Vector3(Vector3.Dot(diff, box.Orientation.Right), Vector3.Dot(diff, box.Orientation.Up), Vector3.Dot(diff, box.Orientation.Backward));
            var kDir = new Vector3(Vector3.Dot(line.Dir, box.Orientation.Right), Vector3.Dot(line.Dir, box.Orientation.Up), Vector3.Dot(line.Dir, box.Orientation.Backward));


            var reflect0 = false;
            var reflect1 = false;
            var reflect2 = false;
            pfLParam = 0;

            if (kDir.X < 0.0f)
            {
                pnt.X = -pnt.X;
                kDir.X = -kDir.X;
                reflect0 = true;
            }

            if (kDir.Y < 0.0f)
            {
                pnt.Y = -pnt.Y;
                kDir.Y = -kDir.Y;
                reflect1 = true;
            }

            if (kDir.Z < 0.0f)
            {
                pnt.Z = -pnt.Z;
                kDir.Z = -kDir.Z;
                reflect2 = true;
            }

            var sqrDistance = 0.0f;

            if (kDir.X > 0.0f)
            {
                if (kDir.Y > 0.0f)
                {
                    if (kDir.Z > 0.0f)
                    {
                        var kPmE = pnt - box.GetHalfSideLengths();

                        var prodDxPy = kDir.X * kPmE.Y;
                        var prodDyPx = kDir.Y * kPmE.X;
                        float prodDzPx, prodDxPz, prodDzPy, prodDyPz;

                        if (prodDyPx >= prodDxPy)
                        {
                            prodDzPx = kDir.Z * kPmE.X;
                            prodDxPz = kDir.X * kPmE.Z;
                            if (prodDzPx >= prodDxPz)

                                FaceA(ref pnt, kDir, box, kPmE, out pfLParam, ref sqrDistance);
                            else

                                FaceB(ref pnt, kDir, box, kPmE, out pfLParam, ref sqrDistance);
                        }
                        else
                        {
                            prodDzPy = kDir.Z * kPmE.Y;
                            prodDyPz = kDir.Y * kPmE.Z;
                            if (prodDzPy >= prodDyPz)

                                FaceC(ref pnt, kDir, box, kPmE, out pfLParam, ref sqrDistance);
                            else

                                FaceB(ref pnt, kDir, box, kPmE, out pfLParam, ref sqrDistance);
                        }
                    }
                    else
                    {
                        var pmE0 = pnt.X - box.GetHalfSideLengths().X;
                        var pmE1 = pnt.Y - box.GetHalfSideLengths().Y;
                        var prod0 = kDir.Y * pmE0;
                        var prod1 = kDir.X * pmE1;
                        float delta, invLSqur, inv;

                        if (prod0 >= prod1)
                        {
                            pnt.X = box.GetHalfSideLengths().X;

                            var ppE1 = pnt.Y + box.GetHalfSideLengths().Y;
                            delta = prod0 - kDir.X * ppE1;

                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.X * kDir.X + kDir.Y * kDir.Y);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.Y = -box.GetHalfSideLengths().Y;
                                pfLParam = -(kDir.X * pmE0 + kDir.Y * ppE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.X;
                                pnt.Y -= prod0 * inv;
                                pfLParam = -pmE0 * inv;
                            }
                        }
                        else
                        {
                            pnt.Y = box.GetHalfSideLengths().Y;

                            var ppE0 = pnt.X + box.GetHalfSideLengths().X;
                            delta = prod1 - kDir.Y * ppE0;
                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.X * kDir.X + kDir.Y * kDir.Y);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.X = -box.GetHalfSideLengths().X;
                                pfLParam = -(kDir.X * ppE0 + kDir.Y * pmE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.Y;
                                pnt.X -= prod1 * inv;
                                pfLParam = -pmE1 * inv;
                            }
                        }

                        if (pnt.Z < -box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z + box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = -box.GetHalfSideLengths().Z;
                        }
                        else if (pnt.Z > box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z - box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = box.GetHalfSideLengths().Z;
                        }
                    }
                }
                else
                {
                    if (kDir.Z > 0.0f)
                    {
                        var pmE0 = pnt.X - box.GetHalfSideLengths().X;
                        var pmE1 = pnt.Z - box.GetHalfSideLengths().Z;
                        var prod0 = kDir.Z * pmE0;
                        var prod1 = kDir.X * pmE1;
                        float delta, invLSqur, inv;

                        if (prod0 >= prod1)
                        {
                            pnt.X = box.GetHalfSideLengths().X;

                            var ppE1 = pnt.Z + box.GetHalfSideLengths().Z;
                            delta = prod0 - kDir.X * ppE1;

                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.X * kDir.X + kDir.Z * kDir.Z);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.Z = -box.GetHalfSideLengths().Z;
                                pfLParam = -(kDir.X * pmE0 + kDir.Z * ppE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.X;
                                pnt.Z -= prod0 * inv;
                                pfLParam = -pmE0 * inv;
                            }
                        }
                        else
                        {
                            pnt.Z = box.GetHalfSideLengths().Z;

                            var ppE0 = pnt.X + box.GetHalfSideLengths().X;
                            delta = prod1 - kDir.Z * ppE0;
                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.X * kDir.X + kDir.Z * kDir.Z);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.X = -box.GetHalfSideLengths().X;
                                pfLParam = -(kDir.X * ppE0 + kDir.Z * pmE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.Z;
                                pnt.X -= prod1 * inv;
                                pfLParam = -pmE1 * inv;
                            }
                        }

                        if (pnt.Y < -box.GetHalfSideLengths().Y)
                        {
                            delta = pnt.Y + box.GetHalfSideLengths().Y;
                            sqrDistance += delta * delta;
                            pnt.Y = -box.GetHalfSideLengths().Y;
                        }
                        else if (pnt.Y > box.GetHalfSideLengths().Y)
                        {
                            delta = pnt.Y - box.GetHalfSideLengths().Y;
                            sqrDistance += delta * delta;
                            pnt.Y = box.GetHalfSideLengths().Y;
                        }
                    }
                    else
                    {
                        var pmE0 = pnt.X - box.GetHalfSideLengths().X;
                        var pmE1 = pnt.Y - box.GetHalfSideLengths().Y;
                        var prod0 = kDir.Y * pmE0;
                        var prod1 = kDir.X * pmE1;
                        float delta, invLSqur, inv;

                        if (prod0 >= prod1)
                        {
                            pnt.X = box.GetHalfSideLengths().X;

                            var ppE1 = pnt.Y + box.GetHalfSideLengths().Y;
                            delta = prod0 - kDir.X * ppE1;

                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.X * kDir.X + kDir.Y * kDir.Y);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.Y = -box.GetHalfSideLengths().Y;
                                pfLParam = -(kDir.X * pmE0 + kDir.Y * ppE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.X;
                                pnt.Y -= prod0 * inv;
                                pfLParam = -pmE0 * inv;
                            }
                        }
                        else
                        {
                            pnt.Y = box.GetHalfSideLengths().Y;

                            var ppE0 = pnt.X + box.GetHalfSideLengths().X;
                            delta = prod1 - kDir.Y * ppE0;
                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.X * kDir.X + kDir.Y * kDir.Y);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.X = -box.GetHalfSideLengths().X;
                                pfLParam = -(kDir.X * ppE0 + kDir.Y * pmE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.Y;
                                pnt.X -= prod1 * inv;
                                pfLParam = -pmE1 * inv;
                            }
                        }

                        if (pnt.Z < -box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z + box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = -box.GetHalfSideLengths().Z;
                        }
                        else if (pnt.Z > box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z - box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = box.GetHalfSideLengths().Z;
                        }
                    }
                }
            }
            else
            {
                if (kDir.Y > 0.0f)
                {
                    if (kDir.Z > 0.0f)
                    {
                        var pmE0 = pnt.Y - box.GetHalfSideLengths().Y;
                        var pmE1 = pnt.Z - box.GetHalfSideLengths().Z;
                        var prod0 = kDir.Z * pmE0;
                        var prod1 = kDir.Y * pmE1;
                        float delta, invLSqur, inv;

                        if (prod0 >= prod1)
                        {
                            pnt.Y = box.GetHalfSideLengths().Y;

                            var ppE1 = pnt.Z + box.GetHalfSideLengths().Z;
                            delta = prod0 - kDir.Y * ppE1;

                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.Y * kDir.Y + kDir.Z * kDir.Z);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.Z = -box.GetHalfSideLengths().Z;
                                pfLParam = -(kDir.Y * pmE0 + kDir.Z * ppE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.Y;
                                pnt.Z -= prod0 * inv;
                                pfLParam = -pmE0 * inv;
                            }
                        }
                        else
                        {
                            pnt.Z = box.GetHalfSideLengths().Z;

                            var ppE0 = pnt.Y + box.GetHalfSideLengths().Y;
                            delta = prod1 - kDir.Z * ppE0;
                            if (delta >= 0.0f)
                            {
                                invLSqur = 1.0f / (kDir.Y * kDir.Y + kDir.Z * kDir.Z);
                                sqrDistance += delta * delta * invLSqur;

                                pnt.Y = -box.GetHalfSideLengths().Y;
                                pfLParam = -(kDir.Y * ppE0 + kDir.Z * pmE1) * invLSqur;
                            }
                            else
                            {
                                inv = 1.0f / kDir.Z;
                                pnt.Y -= prod1 * inv;
                                pfLParam = -pmE1 * inv;
                            }
                        }

                        if (pnt.X < -box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X + box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = -box.GetHalfSideLengths().X;
                        }
                        else if (pnt.X > box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X - box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = box.GetHalfSideLengths().X;
                        }
                    }
                    else
                    {
                        float delta;

                        pfLParam = (box.GetHalfSideLengths().Y - pnt.Y) / kDir.Y;

                        pnt.Y = box.GetHalfSideLengths().Y;

                        if (pnt.X < -box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X + box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = -box.GetHalfSideLengths().X;
                        }
                        else if (pnt.X > box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X - box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = box.GetHalfSideLengths().X;
                        }

                        if (pnt.Z < -box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z + box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = -box.GetHalfSideLengths().Z;
                        }
                        else if (pnt.Z > box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z - box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = box.GetHalfSideLengths().Z;
                        }
                    }
                }
                else
                {
                    if (kDir.Z > 0.0f)
                    {
                        float delta;

                        pfLParam = (box.GetHalfSideLengths().Z - pnt.Z) / kDir.Z;

                        pnt.Z = box.GetHalfSideLengths().Z;

                        if (pnt.X < -box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X + box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = -box.GetHalfSideLengths().X;
                        }
                        else if (pnt.X > box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X - box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = box.GetHalfSideLengths().X;
                        }

                        if (pnt.Y < -box.GetHalfSideLengths().Y)
                        {
                            delta = pnt.Y + box.GetHalfSideLengths().Y;
                            sqrDistance += delta * delta;
                            pnt.Y = -box.GetHalfSideLengths().Y;
                        }
                        else if (pnt.Y > box.GetHalfSideLengths().Y)
                        {
                            delta = pnt.Y - box.GetHalfSideLengths().Y;
                            sqrDistance += delta * delta;
                            pnt.Y = box.GetHalfSideLengths().Y;
                        }
                    }
                    else
                    {
                        float delta;

                        if (pnt.X < -box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X + box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = -box.GetHalfSideLengths().X;
                        }
                        else if (pnt.X > box.GetHalfSideLengths().X)
                        {
                            delta = pnt.X - box.GetHalfSideLengths().X;
                            sqrDistance += delta * delta;
                            pnt.X = box.GetHalfSideLengths().X;
                        }

                        if (pnt.Y < -box.GetHalfSideLengths().Y)
                        {
                            delta = pnt.Y + box.GetHalfSideLengths().Y;
                            sqrDistance += delta * delta;
                            pnt.Y = -box.GetHalfSideLengths().Y;
                        }
                        else if (pnt.Y > box.GetHalfSideLengths().Y)
                        {
                            delta = pnt.Y - box.GetHalfSideLengths().Y;
                            sqrDistance += delta * delta;
                            pnt.Y = box.GetHalfSideLengths().Y;
                        }

                        if (pnt.Z < -box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z + box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = -box.GetHalfSideLengths().Z;
                        }
                        else if (pnt.Z > box.GetHalfSideLengths().Z)
                        {
                            delta = pnt.Z - box.GetHalfSideLengths().Z;
                            sqrDistance += delta * delta;
                            pnt.Z = box.GetHalfSideLengths().Z;
                        }
                    }
                }
            }


            if (reflect0) pnt.X = -pnt.X;
            if (reflect1) pnt.Y = -pnt.Y;
            if (reflect2) pnt.Z = -pnt.Z;

            pfBParam0 = pnt.X;
            pfBParam1 = pnt.Y;
            pfBParam2 = pnt.Z;

            return MathHelper.Max(sqrDistance, 0.0f);
        }

        private static void FaceA(ref Vector3 kPnt, Vector3 kDir, Box rkBox, Vector3 kPmE, out float pfLParam, ref float sqrDistance)
        {
            Vector3 kPpE;
            float fLSqr, fInv, fTmp, fParam, fT, fDelta;

            kPpE.Y = kPnt.Y + rkBox.GetHalfSideLengths().Y;
            kPpE.Z = kPnt.Z + rkBox.GetHalfSideLengths().Z;
            if (kDir.X * kPpE.Y >= kDir.Y * kPmE.X)
            {
                if (kDir.X * kPpE.Z >= kDir.Z * kPmE.X)
                {
                    kPnt.X = rkBox.GetHalfSideLengths().X;
                    fInv = 1.0f / kDir.X;
                    kPnt.Y -= kDir.Y * kPmE.X * fInv;
                    kPnt.Z -= kDir.Z * kPmE.X * fInv;
                    pfLParam = -kPmE.X * fInv;
                }
                else
                {
                    fLSqr = kDir.X * kDir.X + kDir.Z * kDir.Z;
                    fTmp = fLSqr * kPpE.Y - kDir.Y * (kDir.X * kPmE.X + kDir.Z * kPpE.Z);
                    if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Y)
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += kDir.Y * kDir.Y;
                        fTmp = kPpE.Y - fT;
                        fDelta = kDir.X * kPmE.X + kDir.Y * fTmp + kDir.Z * kPpE.Z;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.X * kPmE.X + fTmp * fTmp + kPpE.Z * kPpE.Z + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.X = rkBox.GetHalfSideLengths().X;
                        kPnt.Y = fT - rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                    }
                    else
                    {
                        fLSqr += kDir.Y * kDir.Y;
                        fDelta = kDir.X * kPmE.X + kDir.Y * kPmE.Y + kDir.Z * kPpE.Z;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.X * kPmE.X + kPmE.Y * kPmE.Y + kPpE.Z * kPpE.Z + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.X = rkBox.GetHalfSideLengths().X;
                        kPnt.Y = rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                    }
                }
            }
            else
            {
                if (kDir.X * kPpE.Z >= kDir.Z * kPmE.X)
                {
                    fLSqr = kDir.X * kDir.X + kDir.Y * kDir.Y;
                    fTmp = fLSqr * kPpE.Z - kDir.Z * (kDir.X * kPmE.X + kDir.Y * kPpE.Y);
                    if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Z)
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += kDir.Z * kDir.Z;
                        fTmp = kPpE.Z - fT;
                        fDelta = kDir.X * kPmE.X + kDir.Y * kPpE.Y + kDir.Z * fTmp;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.X * kPmE.X + kPpE.Y * kPpE.Y + fTmp * fTmp + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.X = rkBox.GetHalfSideLengths().X;
                        kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = fT - rkBox.GetHalfSideLengths().Z;
                    }
                    else
                    {
                        fLSqr += kDir.Z * kDir.Z;
                        fDelta = kDir.X * kPmE.X + kDir.Y * kPpE.Y + kDir.Z * kPmE.Z;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.X * kPmE.X + kPpE.Y * kPpE.Y + kPmE.Z * kPmE.Z + fDelta * fParam;


                        pfLParam = fParam;
                        kPnt.X = rkBox.GetHalfSideLengths().X;
                        kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = rkBox.GetHalfSideLengths().Z;
                    }
                }
                else
                {
                    fLSqr = kDir.X * kDir.X + kDir.Z * kDir.Z;
                    fTmp = fLSqr * kPpE.Y - kDir.Y * (kDir.X * kPmE.X + kDir.Z * kPpE.Z);
                    if (fTmp >= 0.0f)
                    {
                        if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Y)
                        {
                            fT = fTmp / fLSqr;
                            fLSqr += kDir.Y * kDir.Y;
                            fTmp = kPpE.Y - fT;
                            fDelta = kDir.X * kPmE.X + kDir.Y * fTmp + kDir.Z * kPpE.Z;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.X * kPmE.X + fTmp * fTmp + kPpE.Z * kPpE.Z + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.X = rkBox.GetHalfSideLengths().X;
                            kPnt.Y = fT - rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                        }
                        else
                        {
                            fLSqr += kDir.Y * kDir.Y;
                            fDelta = kDir.X * kPmE.X + kDir.Y * kPmE.Y + kDir.Z * kPpE.Z;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.X * kPmE.X + kPmE.Y * kPmE.Y + kPpE.Z * kPpE.Z + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.X = rkBox.GetHalfSideLengths().X;
                            kPnt.Y = rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                        }

                        return;
                    }

                    fLSqr = kDir.X * kDir.X + kDir.Y * kDir.Y;
                    fTmp = fLSqr * kPpE.Z - kDir.Z * (kDir.X * kPmE.X + kDir.Y * kPpE.Y);
                    if (fTmp >= 0.0f)
                    {
                        if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Z)
                        {
                            fT = fTmp / fLSqr;
                            fLSqr += kDir.Z * kDir.Z;
                            fTmp = kPpE.Z - fT;
                            fDelta = kDir.X * kPmE.X + kDir.Y * kPpE.Y + kDir.Z * fTmp;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.X * kPmE.X + kPpE.Y * kPpE.Y + fTmp * fTmp + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.X = rkBox.GetHalfSideLengths().X;
                            kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = fT - rkBox.GetHalfSideLengths().Z;
                        }
                        else
                        {
                            fLSqr += kDir.Z * kDir.Z;
                            fDelta = kDir.X * kPmE.X + kDir.Y * kPpE.Y + kDir.Z * kPmE.Z;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.X * kPmE.X + kPpE.Y * kPpE.Y + kPmE.Z * kPmE.Z + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.X = rkBox.GetHalfSideLengths().X;
                            kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = rkBox.GetHalfSideLengths().Z;
                        }

                        return;
                    }


                    fLSqr += kDir.Z * kDir.Z;
                    fDelta = kDir.X * kPmE.X + kDir.Y * kPpE.Y + kDir.Z * kPpE.Z;
                    fParam = -fDelta / fLSqr;
                    sqrDistance += kPmE.X * kPmE.X + kPpE.Y * kPpE.Y + kPpE.Z * kPpE.Z + fDelta * fParam;


                    pfLParam = fParam;
                    kPnt.X = rkBox.GetHalfSideLengths().X;
                    kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                    kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                }
            }
        }

        private static void FaceB(ref Vector3 kPnt, Vector3 kDir, Box rkBox, Vector3 kPmE, out float pfLParam, ref float sqrDistance)
        {
            Vector3 kPpE;
            float fLSqr, fInv, fTmp, fParam, fT, fDelta;

            kPpE.X = kPnt.X + rkBox.GetHalfSideLengths().X;
            kPpE.Y = kPnt.Y + rkBox.GetHalfSideLengths().Y;
            if (kDir.Z * kPpE.X >= kDir.X * kPmE.Z)
            {
                if (kDir.Z * kPpE.Y >= kDir.Y * kPmE.Z)
                {
                    kPnt.Z = rkBox.GetHalfSideLengths().Z;
                    fInv = 1.0f / kDir.Z;
                    kPnt.X -= kDir.X * kPmE.Z * fInv;
                    kPnt.Y -= kDir.Y * kPmE.Z * fInv;
                    pfLParam = -kPmE.Z * fInv;
                }
                else
                {
                    fLSqr = kDir.Z * kDir.Z + kDir.Y * kDir.Y;
                    fTmp = fLSqr * kPpE.X - kDir.X * (kDir.Z * kPmE.Z + kDir.Y * kPpE.Y);
                    if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().X)
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += kDir.X * kDir.X;
                        fTmp = kPpE.X - fT;
                        fDelta = kDir.Z * kPmE.Z + kDir.X * fTmp + kDir.Y * kPpE.Y;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Z * kPmE.Z + fTmp * fTmp + kPpE.Y * kPpE.Y + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.Z = rkBox.GetHalfSideLengths().Z;
                        kPnt.X = fT - rkBox.GetHalfSideLengths().X;
                        kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                    }
                    else
                    {
                        fLSqr += kDir.X * kDir.X;
                        fDelta = kDir.Z * kPmE.Z + kDir.X * kPmE.X + kDir.Y * kPpE.Y;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Z * kPmE.Z + kPmE.X * kPmE.X + kPpE.Y * kPpE.Y + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.Z = rkBox.GetHalfSideLengths().Z;
                        kPnt.X = rkBox.GetHalfSideLengths().X;
                        kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                    }
                }
            }
            else
            {
                if (kDir.Z * kPpE.Y >= kDir.Y * kPmE.Z)
                {
                    fLSqr = kDir.Z * kDir.Z + kDir.X * kDir.X;
                    fTmp = fLSqr * kPpE.Y - kDir.Y * (kDir.Z * kPmE.Z + kDir.X * kPpE.X);
                    if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Y)
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += kDir.Y * kDir.Y;
                        fTmp = kPpE.Y - fT;
                        fDelta = kDir.Z * kPmE.Z + kDir.X * kPpE.X + kDir.Y * fTmp;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Z * kPmE.Z + kPpE.X * kPpE.X + fTmp * fTmp + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.Z = rkBox.GetHalfSideLengths().Z;
                        kPnt.X = -rkBox.GetHalfSideLengths().X;
                        kPnt.Y = fT - rkBox.GetHalfSideLengths().Y;
                    }
                    else
                    {
                        fLSqr += kDir.Y * kDir.Y;
                        fDelta = kDir.Z * kPmE.Z + kDir.X * kPpE.X + kDir.Y * kPmE.Y;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Z * kPmE.Z + kPpE.X * kPpE.X + kPmE.Y * kPmE.Y + fDelta * fParam;


                        pfLParam = fParam;
                        kPnt.Z = rkBox.GetHalfSideLengths().Z;
                        kPnt.X = -rkBox.GetHalfSideLengths().X;
                        kPnt.Y = rkBox.GetHalfSideLengths().Y;
                    }
                }
                else
                {
                    fLSqr = kDir.Z * kDir.Z + kDir.Y * kDir.Y;
                    fTmp = fLSqr * kPpE.X - kDir.X * (kDir.Z * kPmE.Z + kDir.Y * kPpE.Y);
                    if (fTmp >= 0.0f)
                    {
                        if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().X)
                        {
                            fT = fTmp / fLSqr;
                            fLSqr += kDir.X * kDir.X;
                            fTmp = kPpE.X - fT;
                            fDelta = kDir.Z * kPmE.Z + kDir.X * fTmp + kDir.Y * kPpE.Y;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Z * kPmE.Z + fTmp * fTmp + kPpE.Y * kPpE.Y + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Z = rkBox.GetHalfSideLengths().Z;
                            kPnt.X = fT - rkBox.GetHalfSideLengths().X;
                            kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                        }
                        else
                        {
                            fLSqr += kDir.X * kDir.X;
                            fDelta = kDir.Z * kPmE.Z + kDir.X * kPmE.X + kDir.Y * kPpE.Y;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Z * kPmE.Z + kPmE.X * kPmE.X + kPpE.Y * kPpE.Y + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Z = rkBox.GetHalfSideLengths().Z;
                            kPnt.X = rkBox.GetHalfSideLengths().X;
                            kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                        }

                        return;
                    }

                    fLSqr = kDir.Z * kDir.Z + kDir.X * kDir.X;
                    fTmp = fLSqr * kPpE.Y - kDir.Y * (kDir.Z * kPmE.Z + kDir.X * kPpE.X);
                    if (fTmp >= 0.0f)
                    {
                        if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Y)
                        {
                            fT = fTmp / fLSqr;
                            fLSqr += kDir.Y * kDir.Y;
                            fTmp = kPpE.Y - fT;
                            fDelta = kDir.Z * kPmE.Z + kDir.X * kPpE.X + kDir.Y * fTmp;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Z * kPmE.Z + kPpE.X * kPpE.X + fTmp * fTmp + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Z = rkBox.GetHalfSideLengths().Z;
                            kPnt.X = -rkBox.GetHalfSideLengths().X;
                            kPnt.Y = fT - rkBox.GetHalfSideLengths().Y;
                        }
                        else
                        {
                            fLSqr += kDir.Y * kDir.Y;
                            fDelta = kDir.Z * kPmE.Z + kDir.X * kPpE.X + kDir.Y * kPmE.Y;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Z * kPmE.Z + kPpE.X * kPpE.X + kPmE.Y * kPmE.Y + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Z = rkBox.GetHalfSideLengths().Z;
                            kPnt.X = -rkBox.GetHalfSideLengths().X;
                            kPnt.Y = rkBox.GetHalfSideLengths().Y;
                        }

                        return;
                    }


                    fLSqr += kDir.Y * kDir.Y;
                    fDelta = kDir.Z * kPmE.Z + kDir.X * kPpE.X + kDir.Y * kPpE.Y;
                    fParam = -fDelta / fLSqr;
                    sqrDistance += kPmE.Z * kPmE.Z + kPpE.X * kPpE.X + kPpE.Y * kPpE.Y + fDelta * fParam;


                    pfLParam = fParam;
                    kPnt.Z = rkBox.GetHalfSideLengths().Z;
                    kPnt.X = -rkBox.GetHalfSideLengths().X;
                    kPnt.Y = -rkBox.GetHalfSideLengths().Y;
                }
            }
        }

        private static void FaceC(ref Vector3 kPnt, Vector3 kDir, Box rkBox, Vector3 kPmE, out float pfLParam, ref float sqrDistance)
        {
            Vector3 kPpE;
            float fLSqr, fInv, fTmp, fParam, fT, fDelta;

            kPpE.Z = kPnt.Z + rkBox.GetHalfSideLengths().Z;
            kPpE.X = kPnt.X + rkBox.GetHalfSideLengths().X;
            if (kDir.Y * kPpE.Z >= kDir.Z * kPmE.Y)
            {
                if (kDir.Y * kPpE.X >= kDir.X * kPmE.Y)
                {
                    kPnt.Y = rkBox.GetHalfSideLengths().Y;
                    fInv = 1.0f / kDir.Y;
                    kPnt.Z -= kDir.Z * kPmE.Y * fInv;
                    kPnt.X -= kDir.X * kPmE.Y * fInv;
                    pfLParam = -kPmE.Y * fInv;
                }
                else
                {
                    fLSqr = kDir.Y * kDir.Y + kDir.X * kDir.X;
                    fTmp = fLSqr * kPpE.Z - kDir.Z * (kDir.Y * kPmE.Y + kDir.X * kPpE.X);
                    if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Z)
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += kDir.Z * kDir.Z;
                        fTmp = kPpE.Z - fT;
                        fDelta = kDir.Y * kPmE.Y + kDir.Z * fTmp + kDir.X * kPpE.X;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Y * kPmE.Y + fTmp * fTmp + kPpE.X * kPpE.X + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.Y = rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = fT - rkBox.GetHalfSideLengths().Z;
                        kPnt.X = -rkBox.GetHalfSideLengths().X;
                    }
                    else
                    {
                        fLSqr += kDir.Z * kDir.Z;
                        fDelta = kDir.Y * kPmE.Y + kDir.Z * kPmE.Z + kDir.X * kPpE.X;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Y * kPmE.Y + kPmE.Z * kPmE.Z + kPpE.X * kPpE.X + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.Y = rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = rkBox.GetHalfSideLengths().Z;
                        kPnt.X = -rkBox.GetHalfSideLengths().X;
                    }
                }
            }
            else
            {
                if (kDir.Y * kPpE.X >= kDir.X * kPmE.Y)
                {
                    fLSqr = kDir.Y * kDir.Y + kDir.Z * kDir.Z;
                    fTmp = fLSqr * kPpE.X - kDir.X * (kDir.Y * kPmE.Y + kDir.Z * kPpE.Z);
                    if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().X)
                    {
                        fT = fTmp / fLSqr;
                        fLSqr += kDir.X * kDir.X;
                        fTmp = kPpE.X - fT;
                        fDelta = kDir.Y * kPmE.Y + kDir.Z * kPpE.Z + kDir.X * fTmp;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Y * kPmE.Y + kPpE.Z * kPpE.Z + fTmp * fTmp + fDelta * fParam;

                        pfLParam = fParam;
                        kPnt.Y = rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                        kPnt.X = fT - rkBox.GetHalfSideLengths().X;
                    }
                    else
                    {
                        fLSqr += kDir.X * kDir.X;
                        fDelta = kDir.Y * kPmE.Y + kDir.Z * kPpE.Z + kDir.X * kPmE.X;
                        fParam = -fDelta / fLSqr;
                        sqrDistance += kPmE.Y * kPmE.Y + kPpE.Z * kPpE.Z + kPmE.X * kPmE.X + fDelta * fParam;


                        pfLParam = fParam;
                        kPnt.Y = rkBox.GetHalfSideLengths().Y;
                        kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                        kPnt.X = rkBox.GetHalfSideLengths().X;
                    }
                }
                else
                {
                    fLSqr = kDir.Y * kDir.Y + kDir.X * kDir.X;
                    fTmp = fLSqr * kPpE.Z - kDir.Z * (kDir.Y * kPmE.Y + kDir.X * kPpE.X);
                    if (fTmp >= 0.0f)
                    {
                        if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().Z)
                        {
                            fT = fTmp / fLSqr;
                            fLSqr += kDir.Z * kDir.Z;
                            fTmp = kPpE.Z - fT;
                            fDelta = kDir.Y * kPmE.Y + kDir.Z * fTmp + kDir.X * kPpE.X;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Y * kPmE.Y + fTmp * fTmp + kPpE.X * kPpE.X + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Y = rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = fT - rkBox.GetHalfSideLengths().Z;
                            kPnt.X = -rkBox.GetHalfSideLengths().X;
                        }
                        else
                        {
                            fLSqr += kDir.Z * kDir.Z;
                            fDelta = kDir.Y * kPmE.Y + kDir.Z * kPmE.Z + kDir.X * kPpE.X;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Y * kPmE.Y + kPmE.Z * kPmE.Z + kPpE.X * kPpE.X + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Y = rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = rkBox.GetHalfSideLengths().Z;
                            kPnt.X = -rkBox.GetHalfSideLengths().X;
                        }

                        return;
                    }

                    fLSqr = kDir.Y * kDir.Y + kDir.Z * kDir.Z;
                    fTmp = fLSqr * kPpE.X - kDir.X * (kDir.Y * kPmE.Y + kDir.Z * kPpE.Z);
                    if (fTmp >= 0.0f)
                    {
                        if (fTmp <= 2.0f * fLSqr * rkBox.GetHalfSideLengths().X)
                        {
                            fT = fTmp / fLSqr;
                            fLSqr += kDir.X * kDir.X;
                            fTmp = kPpE.X - fT;
                            fDelta = kDir.Y * kPmE.Y + kDir.Z * kPpE.Z + kDir.X * fTmp;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Y * kPmE.Y + kPpE.Z * kPpE.Z + fTmp * fTmp + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Y = rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                            kPnt.X = fT - rkBox.GetHalfSideLengths().X;
                        }
                        else
                        {
                            fLSqr += kDir.X * kDir.X;
                            fDelta = kDir.Y * kPmE.Y + kDir.Z * kPpE.Z + kDir.X * kPmE.X;
                            fParam = -fDelta / fLSqr;
                            sqrDistance += kPmE.Y * kPmE.Y + kPpE.Z * kPpE.Z + kPmE.X * kPmE.X + fDelta * fParam;

                            pfLParam = fParam;
                            kPnt.Y = rkBox.GetHalfSideLengths().Y;
                            kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                            kPnt.X = rkBox.GetHalfSideLengths().X;
                        }

                        return;
                    }


                    fLSqr += kDir.X * kDir.X;
                    fDelta = kDir.Y * kPmE.Y + kDir.Z * kPpE.Z + kDir.X * kPpE.X;
                    fParam = -fDelta / fLSqr;
                    sqrDistance += kPmE.Y * kPmE.Y + kPpE.Z * kPpE.Z + kPpE.X * kPpE.X + fDelta * fParam;


                    pfLParam = fParam;
                    kPnt.Y = rkBox.GetHalfSideLengths().Y;
                    kPnt.Z = -rkBox.GetHalfSideLengths().Z;
                    kPnt.X = -rkBox.GetHalfSideLengths().X;
                }
            }
        }

        public static float SegmentSegmentDistanceSq(out float t0, out float t1, Segment seg0, Segment seg1)
        {
            var kDiff = seg0.Origin - seg1.Origin;
            var fA00 = seg0.Delta.LengthSquared();
            var fA01 = -Vector3.Dot(seg0.Delta, seg1.Delta);
            var fA11 = seg1.Delta.LengthSquared();
            var fB0 = Vector3.Dot(kDiff, seg0.Delta);
            var fC = kDiff.LengthSquared();
            var fDet = System.Math.Abs(fA00 * fA11 - fA01 * fA01);
            float fB1, fS, fT, fSqrDist, fTmp;

            if (fDet >= JiggleMath.Epsilon)
            {
                fB1 = -Vector3.Dot(kDiff, seg1.Delta);
                fS = fA01 * fB1 - fA11 * fB0;
                fT = fA01 * fB0 - fA00 * fB1;

                if (fS >= 0.0f)
                {
                    if (fS <= fDet)
                    {
                        if (fT >= 0.0f)
                        {
                            if (fT <= fDet)
                            {
                                var fInvDet = 1.0f / fDet;
                                fS *= fInvDet;
                                fT *= fInvDet;
                                fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) + fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
                            }
                            else
                            {
                                fT = 1.0f;
                                fTmp = fA01 + fB0;
                                if (fTmp >= 0.0f)
                                {
                                    fS = 0.0f;
                                    fSqrDist = fA11 + 2.0f * fB1 + fC;
                                }
                                else if (-fTmp >= fA00)
                                {
                                    fS = 1.0f;
                                    fSqrDist = fA00 + fA11 + fC + 2.0f * (fB1 + fTmp);
                                }
                                else
                                {
                                    fS = -fTmp / fA00;
                                    fSqrDist = fTmp * fS + fA11 + 2.0f * fB1 + fC;
                                }
                            }
                        }
                        else
                        {
                            fT = 0.0f;
                            if (fB0 >= 0.0f)
                            {
                                fS = 0.0f;
                                fSqrDist = fC;
                            }
                            else if (-fB0 >= fA00)
                            {
                                fS = 1.0f;
                                fSqrDist = fA00 + 2.0f * fB0 + fC;
                            }
                            else
                            {
                                fS = -fB0 / fA00;
                                fSqrDist = fB0 * fS + fC;
                            }
                        }
                    }
                    else
                    {
                        if (fT >= 0.0f)
                        {
                            if (fT <= fDet)
                            {
                                fS = 1.0f;
                                fTmp = fA01 + fB1;
                                if (fTmp >= 0.0f)
                                {
                                    fT = 0.0f;
                                    fSqrDist = fA00 + 2.0f * fB0 + fC;
                                }
                                else if (-fTmp >= fA11)
                                {
                                    fT = 1.0f;
                                    fSqrDist = fA00 + fA11 + fC + 2.0f * (fB0 + fTmp);
                                }
                                else
                                {
                                    fT = -fTmp / fA11;
                                    fSqrDist = fTmp * fT + fA00 + 2.0f * fB0 + fC;
                                }
                            }
                            else
                            {
                                fTmp = fA01 + fB0;
                                if (-fTmp <= fA00)
                                {
                                    fT = 1.0f;
                                    if (fTmp >= 0.0f)
                                    {
                                        fS = 0.0f;
                                        fSqrDist = fA11 + 2.0f * fB1 + fC;
                                    }
                                    else
                                    {
                                        fS = -fTmp / fA00;
                                        fSqrDist = fTmp * fS + fA11 + 2.0f * fB1 + fC;
                                    }
                                }
                                else
                                {
                                    fS = 1.0f;
                                    fTmp = fA01 + fB1;
                                    if (fTmp >= 0.0f)
                                    {
                                        fT = 0.0f;
                                        fSqrDist = fA00 + 2.0f * fB0 + fC;
                                    }
                                    else if (-fTmp >= fA11)
                                    {
                                        fT = 1.0f;
                                        fSqrDist = fA00 + fA11 + fC + 2.0f * (fB0 + fTmp);
                                    }
                                    else
                                    {
                                        fT = -fTmp / fA11;
                                        fSqrDist = fTmp * fT + fA00 + 2.0f * fB0 + fC;
                                    }
                                }
                            }
                        }
                        else
                        {
                            if (-fB0 < fA00)
                            {
                                fT = 0.0f;
                                if (fB0 >= 0.0f)
                                {
                                    fS = 0.0f;
                                    fSqrDist = fC;
                                }
                                else
                                {
                                    fS = -fB0 / fA00;
                                    fSqrDist = fB0 * fS + fC;
                                }
                            }
                            else
                            {
                                fS = 1.0f;
                                fTmp = fA01 + fB1;
                                if (fTmp >= 0.0f)
                                {
                                    fT = 0.0f;
                                    fSqrDist = fA00 + 2.0f * fB0 + fC;
                                }
                                else if (-fTmp >= fA11)
                                {
                                    fT = 1.0f;
                                    fSqrDist = fA00 + fA11 + fC + 2.0f * (fB0 + fTmp);
                                }
                                else
                                {
                                    fT = -fTmp / fA11;
                                    fSqrDist = fTmp * fT + fA00 + 2.0f * fB0 + fC;
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (fT >= 0.0f)
                    {
                        if (fT <= fDet)
                        {
                            fS = 0.0f;
                            if (fB1 >= 0.0f)
                            {
                                fT = 0.0f;
                                fSqrDist = fC;
                            }
                            else if (-fB1 >= fA11)
                            {
                                fT = 1.0f;
                                fSqrDist = fA11 + 2.0f * fB1 + fC;
                            }
                            else
                            {
                                fT = -fB1 / fA11;
                                fSqrDist = fB1 * fT + fC;
                            }
                        }
                        else
                        {
                            fTmp = fA01 + fB0;
                            if (fTmp < 0.0f)
                            {
                                fT = 1.0f;
                                if (-fTmp >= fA00)
                                {
                                    fS = 1.0f;
                                    fSqrDist = fA00 + fA11 + fC + 2.0f * (fB1 + fTmp);
                                }
                                else
                                {
                                    fS = -fTmp / fA00;
                                    fSqrDist = fTmp * fS + fA11 + 2.0f * fB1 + fC;
                                }
                            }
                            else
                            {
                                fS = 0.0f;
                                if (fB1 >= 0.0f)
                                {
                                    fT = 0.0f;
                                    fSqrDist = fC;
                                }
                                else if (-fB1 >= fA11)
                                {
                                    fT = 1.0f;
                                    fSqrDist = fA11 + 2.0f * fB1 + fC;
                                }
                                else
                                {
                                    fT = -fB1 / fA11;
                                    fSqrDist = fB1 * fT + fC;
                                }
                            }
                        }
                    }
                    else
                    {
                        if (fB0 < 0.0f)
                        {
                            fT = 0.0f;
                            if (-fB0 >= fA00)
                            {
                                fS = 1.0f;
                                fSqrDist = fA00 + 2.0f * fB0 + fC;
                            }
                            else
                            {
                                fS = -fB0 / fA00;
                                fSqrDist = fB0 * fS + fC;
                            }
                        }
                        else
                        {
                            fS = 0.0f;
                            if (fB1 >= 0.0f)
                            {
                                fT = 0.0f;
                                fSqrDist = fC;
                            }
                            else if (-fB1 >= fA11)
                            {
                                fT = 1.0f;
                                fSqrDist = fA11 + 2.0f * fB1 + fC;
                            }
                            else
                            {
                                fT = -fB1 / fA11;
                                fSqrDist = fB1 * fT + fC;
                            }
                        }
                    }
                }
            }
            else
            {
                if (fA01 > 0.0f)
                {
                    if (fB0 >= 0.0f)
                    {
                        fS = 0.0f;
                        fT = 0.0f;
                        fSqrDist = fC;
                    }
                    else if (-fB0 <= fA00)
                    {
                        fS = -fB0 / fA00;
                        fT = 0.0f;
                        fSqrDist = fB0 * fS + fC;
                    }
                    else
                    {
                        fB1 = -Vector3.Dot(kDiff, seg1.Delta);
                        fS = 1.0f;
                        fTmp = fA00 + fB0;
                        if (-fTmp >= fA01)
                        {
                            fT = 1.0f;
                            fSqrDist = fA00 + fA11 + fC + 2.0f * (fA01 + fB0 + fB1);
                        }
                        else
                        {
                            fT = -fTmp / fA01;
                            fSqrDist = fA00 + 2.0f * fB0 + fC + fT * (fA11 * fT + 2.0f * (fA01 + fB1));
                        }
                    }
                }
                else
                {
                    if (-fB0 >= fA00)
                    {
                        fS = 1.0f;
                        fT = 0.0f;
                        fSqrDist = fA00 + 2.0f * fB0 + fC;
                    }
                    else if (fB0 <= 0.0f)
                    {
                        fS = -fB0 / fA00;
                        fT = 0.0f;
                        fSqrDist = fB0 * fS + fC;
                    }
                    else
                    {
                        fB1 = -Vector3.Dot(kDiff, seg1.Delta);
                        fS = 0.0f;
                        if (fB0 >= -fA01)
                        {
                            fT = 1.0f;
                            fSqrDist = fA11 + 2.0f * fB1 + fC;
                        }
                        else
                        {
                            fT = -fB0 / fA01;
                            fSqrDist = fC + fT * (2.0f * fB1 + fA11 * fT);
                        }
                    }
                }
            }

            t0 = fS;
            t1 = fT;

            return System.Math.Abs(fSqrDist);
        }
    }
}