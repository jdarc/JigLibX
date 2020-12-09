using System.Collections.Generic;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using JigLibX.Utils;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectBoxBox : DetectFunctor
    {
        private struct ContactPoint
        {
            public Vector3 Pos;

            public int Count;

            public ContactPoint(ref Vector3 pos)
            {
                Pos = pos;
                Count = 1;
            }
        }

        public CollDetectBoxBox() : base((int) PrimitiveType.Box, (int) PrimitiveType.Box)
        {
        }

        private static bool Disjoint(out float d, ref Vector3 axis, Box box0, Box box1, float collTolerance)
        {
            box0.GetSpan(out var min0, out var max0, ref axis);
            box1.GetSpan(out var min1, out var max1, ref axis);

            if (min0 > max1 + collTolerance + JiggleMath.Epsilon || min1 > max0 + collTolerance + JiggleMath.Epsilon)
            {
                d = 0.0f;
                return true;
            }

            if (max0 > max1 && min1 > min0)
            {
                d = MathHelper.Min(max0 - min1, max1 - min0);
            }
            else if (max1 > max0 && min0 > min1)
            {
                d = MathHelper.Min(max1 - min0, max0 - min1);
            }
            else
            {
                d = max0 < max1 ? max0 : max1;
                d -= min0 > min1 ? min0 : min1;
            }

            return false;
        }

        private static void GetSupportPoint(out Vector3 p, Box box, Vector3 axis)
        {
            var orient0 = new Vector3();
            orient0.X = box.transform.Orientation.M11;
            orient0.Y = box.transform.Orientation.M12;
            orient0.Z = box.transform.Orientation.M13;

            var orient1 = new Vector3();
            orient1.X = box.transform.Orientation.M21;
            orient1.Y = box.transform.Orientation.M22;
            orient1.Z = box.transform.Orientation.M23;

            var orient2 = new Vector3();
            orient2.X = box.transform.Orientation.M31;
            orient2.Y = box.transform.Orientation.M32;
            orient2.Z = box.transform.Orientation.M33;

            var ass = axis.X * orient0.X + axis.Y * orient0.Y + axis.Z * orient0.Z;

            var au = axis.X * orient1.X + axis.Y * orient1.Y + axis.Z * orient1.Z;

            var ad = axis.X * orient2.X + axis.Y * orient2.Y + axis.Z * orient2.Z;

            var threshold = JiggleMath.Epsilon;

            box.GetCentre(out p);

            if (ass < -threshold)
            {
                p.X += orient0.X * (0.5f * box.SideLengths.X);
                p.Y += orient0.Y * (0.5f * box.SideLengths.X);
                p.Z += orient0.Z * (0.5f * box.SideLengths.X);
            }
            else if (ass >= threshold)
            {
                p.X -= orient0.X * (0.5f * box.SideLengths.X);
                p.Y -= orient0.Y * (0.5f * box.SideLengths.X);
                p.Z -= orient0.Z * (0.5f * box.SideLengths.X);
            }

            if (au < -threshold)
            {
                p.X += orient1.X * (0.5f * box.SideLengths.Y);
                p.Y += orient1.Y * (0.5f * box.SideLengths.Y);
                p.Z += orient1.Z * (0.5f * box.SideLengths.Y);
            }
            else if (au >= threshold)
            {
                p.X -= orient1.X * (0.5f * box.SideLengths.Y);
                p.Y -= orient1.Y * (0.5f * box.SideLengths.Y);
                p.Z -= orient1.Z * (0.5f * box.SideLengths.Y);
            }

            if (ad < -threshold)
            {
                p.X += orient2.X * (0.5f * box.SideLengths.Z);
                p.Y += orient2.Y * (0.5f * box.SideLengths.Z);
                p.Z += orient2.Z * (0.5f * box.SideLengths.Z);
            }
            else if (ad >= threshold)
            {
                p.X -= orient2.X * (0.5f * box.SideLengths.Z);
                p.Y -= orient2.Y * (0.5f * box.SideLengths.Z);
                p.Z -= orient2.Z * (0.5f * box.SideLengths.Z);
            }
        }

        private static bool AddPoint(List<ContactPoint> pts, ref Vector3 pt, float combinationDistanceSq)
        {
            for (var i = pts.Count; i-- != 0;)
            {
                var cpt = pts[i];

                var xd = cpt.Pos.X - pt.X;
                var yd = cpt.Pos.Y - pt.Y;
                var zd = cpt.Pos.Z - pt.Z;

                var len = xd * xd + yd * yd + zd * zd;

                if (len < combinationDistanceSq)
                {
                    cpt.Pos = (cpt.Count * cpt.Pos + pt) / (cpt.Count + 1);
                    cpt.Count += 1;
                    return false;
                }
            }

            pts.Add(new ContactPoint(ref pt));
            return true;
        }

        private static int GetAABox2EdgeIntersectionPoints(List<ContactPoint> pts, ref Vector3 sides, Box box, ref Vector3 edgePt0, ref Vector3 edgePt1, ref Matrix origBoxOrient, ref Vector3 origBoxPos, float combinationDistanceSq)
        {
            Vector3.Subtract(ref edgePt1, ref edgePt0, out var edgeDir);
            JiggleMath.NormalizeSafe(ref edgeDir);

            var num = 0;


            var pt = new Vector3();

            for (var idir = 3; idir-- != 0;)
            {
                if (System.Math.Abs(JiggleUnsafe.Get(ref edgeDir, idir)) < 0.1f) continue;

                var jdir = (idir + 1) % 3;
                var kdir = (idir + 2) % 3;
                for (var iface = 2; iface-- != 0;)
                {
                    var offset = 0.0f;
                    if (iface == 1) offset = JiggleUnsafe.Get(ref sides, idir);

                    var dist0 = JiggleUnsafe.Get(ref edgePt0, idir) - offset;
                    var dist1 = JiggleUnsafe.Get(ref edgePt1, idir) - offset;

                    var frac = -1.0f;

                    if (dist0 * dist1 < -JiggleMath.Epsilon)
                        frac = -dist0 / (dist1 - dist0);
                    else if (System.Math.Abs(dist0) < JiggleMath.Epsilon)
                        frac = 0.0f;
                    else if (System.Math.Abs(dist1) < JiggleMath.Epsilon) frac = 1.0f;

                    if (frac >= 0.0f)
                    {
                        var tempFrac = 1.0f - frac;
                        pt.X = tempFrac * edgePt0.X + frac * edgePt1.X;
                        pt.Y = tempFrac * edgePt0.Y + frac * edgePt1.Y;
                        pt.Z = tempFrac * edgePt0.Z + frac * edgePt1.Z;


                        var ptJdir = JiggleUnsafe.Get(ref pt, jdir);
                        var ptKdir = JiggleUnsafe.Get(ref pt, kdir);

                        if (ptJdir > -JiggleMath.Epsilon && ptJdir < JiggleUnsafe.Get(ref sides, jdir) + JiggleMath.Epsilon && ptKdir > -JiggleMath.Epsilon && ptKdir < JiggleUnsafe.Get(ref sides, kdir) + JiggleMath.Epsilon)
                        {
                            Vector3.TransformNormal(ref pt, ref origBoxOrient, out var pos);
                            pos.X += origBoxPos.X;
                            pos.Y += origBoxPos.Y;
                            pos.Z += origBoxPos.Z;

                            AddPoint(pts, ref pos, combinationDistanceSq);

                            if (++num == 2) return num;
                        }
                    }
                }
            }

            return num;
        }

        private static int GetAABox2BoxEdgesIntersectionPoints(List<ContactPoint> pts, ref Vector3 sides, Box box, ref Matrix origBoxOrient, ref Vector3 origBoxPos, float combinationDistanceSq)
        {
            var num = 0;
            box.GetCornerPoints(out var boxPts);
            box.GetEdges(out var edges);

            for (var iedge = 0; iedge < 12; ++iedge)
            {
                var edgePt0 = boxPts[(int) edges[iedge].Ind0];
                var edgePt1 = boxPts[(int) edges[iedge].Ind1];

                num += GetAABox2EdgeIntersectionPoints(pts, ref sides, box, ref edgePt0, ref edgePt1, ref origBoxOrient, ref origBoxPos, combinationDistanceSq);


                if (num >= 8) return num;
            }

            return num;
        }

        private static Box tempBox = new Box(Vector3.Zero, Matrix.Identity, Vector3.Zero);

        private static int GetBoxBoxIntersectionPoints(List<ContactPoint> pts, Box box0, Box box1, float combinationDistance, float collTolerance)
        {
            var tolVal = 0.5f * collTolerance;

            var tol = new Vector3(tolVal);

            combinationDistance += collTolerance * 2.0f * (float) System.Math.Sqrt(3.0d);

            for (var ibox = 0; ibox < 2; ++ibox)
            {
                var boxA = ibox != 0 ? box1 : box0;
                var boxB = ibox != 0 ? box0 : box1;

                Matrix.Transpose(ref boxA.transform.Orientation, out var boxAInvOrient);

                Vector3.Subtract(ref boxB.transform.Position, ref boxA.transform.Position, out var pos);
                Vector3.TransformNormal(ref pos, ref boxAInvOrient, out pos);

                Matrix.Multiply(ref boxB.transform.Orientation, ref boxAInvOrient, out var boxOrient);

                var box = tempBox;
                box.Position = pos;
                box.Orientation = boxOrient;
                box.SideLengths = boxB.SideLengths;


                var sL = boxA.SideLengths;
                GetAABox2BoxEdgesIntersectionPoints(pts, ref sL, box, ref boxA.transform.Orientation, ref boxA.transform.Position, combinationDistance * combinationDistance);
            }

            return pts.Count;
        }

        private Vector3[] seperatingAxes = new Vector3[15];

        private float[] overlapDepth = new float[15];
        private List<ContactPoint> contactPts = new List<ContactPoint>(64);

        public override void CollDetect(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var box0 = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Box;
            var box1 = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Box;

            var oldBox0 = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Box;
            var oldBox1 = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Box;

            var dirs0 = box0.Orientation;
            var dirs1 = box1.Orientation;

            var box0_Right = dirs0.Right;
            var box0_Up = dirs0.Up;
            var box0_Backward = dirs0.Backward;

            var box1_Right = dirs1.Right;
            var box1_Up = dirs1.Up;
            var box1_Backward = dirs1.Backward;

            if (Disjoint(out var testDepth, ref box0_Right, box0, box1, collTolerance)) return;

            var depth = testDepth;
            var N = box0_Right;
            var minAxis = 0;

            if (Disjoint(out testDepth, ref box0_Up, box0, box1, collTolerance)) return;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = box0_Up;
                minAxis = 1;
            }

            if (Disjoint(out testDepth, ref box0_Backward, box0, box1, collTolerance)) return;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = box0_Backward;
                minAxis = 2;
            }

            if (Disjoint(out testDepth, ref box1_Right, box0, box1, collTolerance)) return;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = box1_Right;
                minAxis = 3;
            }

            if (Disjoint(out testDepth, ref box1_Up, box0, box1, collTolerance)) return;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = box1_Up;
                minAxis = 4;
            }

            if (Disjoint(out testDepth, ref box1_Backward, box0, box1, collTolerance)) return;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = box1_Backward;
                minAxis = 5;
            }

            Vector3.Cross(ref box0_Right, ref box1_Right, out var axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 6;
            }

            Vector3.Cross(ref box0_Right, ref box1_Up, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 7;
            }

            Vector3.Cross(ref box0_Right, ref box1_Backward, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 8;
            }

            Vector3.Cross(ref box0_Up, ref box1_Right, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 9;
            }

            Vector3.Cross(ref box0_Up, ref box1_Up, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 10;
            }

            Vector3.Cross(ref box0_Up, ref box1_Backward, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 11;
            }

            Vector3.Cross(ref box0_Backward, ref box1_Right, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 12;
            }

            Vector3.Cross(ref box0_Backward, ref box1_Up, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 13;
            }

            Vector3.Cross(ref box0_Backward, ref box1_Backward, out axis);
            if (Disjoint(out testDepth, ref axis, box0, box1, collTolerance)) return;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
                minAxis = 14;
            }

            var D = box1.GetCentre() - box0.GetCentre();
            N.Normalize();
            int i;

            /*seperatingAxes[0] = dirs0.Right;
            seperatingAxes[1] = dirs0.Up;
            seperatingAxes[2] = dirs0.Backward;
            seperatingAxes[3] = dirs1.Right;
            seperatingAxes[4] = dirs1.Up;
            seperatingAxes[5] = dirs1.Backward;
            Vector3.Cross(ref seperatingAxes[0], ref seperatingAxes[3], out seperatingAxes[6]);
            Vector3.Cross(ref seperatingAxes[0], ref seperatingAxes[4], out seperatingAxes[7]);
            Vector3.Cross(ref seperatingAxes[0], ref seperatingAxes[5], out seperatingAxes[8]);
            Vector3.Cross(ref seperatingAxes[1], ref seperatingAxes[3], out seperatingAxes[9]);
            Vector3.Cross(ref seperatingAxes[1], ref seperatingAxes[4], out seperatingAxes[10]);
            Vector3.Cross(ref seperatingAxes[1], ref seperatingAxes[5], out seperatingAxes[11]);
            Vector3.Cross(ref seperatingAxes[2], ref seperatingAxes[3], out seperatingAxes[12]);
            Vector3.Cross(ref seperatingAxes[2], ref seperatingAxes[4], out seperatingAxes[13]);
            Vector3.Cross(ref seperatingAxes[2], ref seperatingAxes[5], out seperatingAxes[14]);


            
            
            int i;
            for (i = 0; i < 15; ++i)
            {
                
                float l2 = seperatingAxes[i].LengthSquared();

                if (l2 < JiggleMath.Epsilon) continue;

                overlapDepth[i] = float.MaxValue;

                if (Disjoint(out overlapDepth[i], ref seperatingAxes[i], box0, box1, collTolerance))
                    return;
            }

            
            float minDepth = float.MaxValue;
            int minAxis = -1;

            for (i = 0; i < 15; ++i)
            {
                
                float l2 = seperatingAxes[i].LengthSquared();
                if (l2 < JiggleMath.Epsilon) continue;

                
                float invl = 1.0f / (float)System.Math.Sqrt(l2);
                seperatingAxes[i] *= invl;
                overlapDepth[i] *= invl;

                
                if (overlapDepth[i] < minDepth)
                {
                    minDepth = overlapDepth[i];
                    minAxis = i;
                }
            }

            if (minAxis == -1)
                return;

            
            
            Vector3 D = box1.GetCentre() - box0.GetCentre();
            Vector3 N = seperatingAxes[minAxis];
            float depth = overlapDepth[minAxis];*/

            if (Vector3.Dot(D, N) > 0.0f) N *= -1.0f;

            var minA = MathHelper.Min(box0.SideLengths.X, MathHelper.Min(box0.SideLengths.Y, box0.SideLengths.Z));
            var minB = MathHelper.Min(box1.SideLengths.X, MathHelper.Min(box1.SideLengths.Y, box1.SideLengths.Z));

            var combinationDist = 0.05f * MathHelper.Min(minA, minB);


            var contactPointsFromOld = true;
            contactPts.Clear();

            if (depth > -JiggleMath.Epsilon)
                GetBoxBoxIntersectionPoints(contactPts, oldBox0, oldBox1, combinationDist, collTolerance);

            var numPts = contactPts.Count;
            if (numPts == 0)
            {
                contactPointsFromOld = false;
                GetBoxBoxIntersectionPoints(contactPts, box0, box1, combinationDist, collTolerance);
            }

            numPts = contactPts.Count;

            var body0OldPos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
            var body1OldPos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;
            var body0NewPos = info.Skin0.Owner?.Position ?? Vector3.Zero;
            var body1NewPos = info.Skin1.Owner?.Position ?? Vector3.Zero;

            Vector3.Subtract(ref body0NewPos, ref body0OldPos, out var bodyDelta);
            Vector3.Subtract(ref bodyDelta, ref body1NewPos, out bodyDelta);
            Vector3.Add(ref bodyDelta, ref body1OldPos, out bodyDelta);

            Vector3.Dot(ref bodyDelta, ref N, out var bodyDeltaLen);

            var oldDepth = depth + bodyDeltaLen;

            unsafe
            {
#if USE_STACKALLOC
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
#else
                var collPtArray = SCPIStackAlloc();
                fixed (SmallCollPointInfo* collPts = collPtArray)
#endif
                {
                    var numCollPts = 0;

                    Vector3 SATPoint;

                    switch (minAxis)
                    {
                        case 0:
                        case 1:
                        case 2:
                        {
                            GetSupportPoint(out SATPoint, box1, -N);
                            break;
                        }

                        case 3:
                        case 4:
                        case 5:
                        {
                            GetSupportPoint(out SATPoint, box0, N);
                            break;
                        }

                        default:
                            /*case 6:
                            case 7:
                            case 8:
                            case 9:
                            case 10:
                            case 11:
                            case 12:
                            case 13:
                            case 14:*/
                        {
                            {
                                i = minAxis - 6;
                                var ia = i / 3;
                                var ib = i - ia * 3;

                                GetSupportPoint(out var P0, box0, N);
                                GetSupportPoint(out var P1, box1, -N);


                                JiggleUnsafe.Get(ref box0.transform.Orientation, ia, out var box0Orient);
                                JiggleUnsafe.Get(ref box1.transform.Orientation, ib, out var box1Orient);

                                Vector3.Cross(ref N, ref box1Orient, out var planeNormal);

                                Vector3.Dot(ref planeNormal, ref P1, out var planeD);


                                Vector3.Dot(ref box0Orient, ref planeNormal, out var div);


                                if (System.Math.Abs(div) < JiggleMath.Epsilon) return;

                                var t = (planeD - Vector3.Dot(P0, planeNormal)) / div;


                                P0 = Vector3.Add(Vector3.Multiply(box0Orient, t), P0);

                                Vector3.Multiply(ref N, 0.5f * depth, out SATPoint);
                                Vector3.Add(ref SATPoint, ref P0, out SATPoint);
                            }
                            break;
                        }
                        /*default:
                            throw new Exception("Impossible switch");*/
                    }


                    if (numPts > 0)
                    {
                        var minDist = float.MaxValue;
                        var maxDist = float.MinValue;
                        for (i = 0; i < numPts; ++i)
                        {
                            var dist = Distance.PointPointDistance(contactPts[i].Pos, SATPoint);
                            if (dist < minDist) minDist = dist;
                            if (dist > maxDist) maxDist = dist;
                        }

                        if (maxDist < minDist + JiggleMath.Epsilon) maxDist = minDist + JiggleMath.Epsilon;


                        for (i = 0; i < numPts; ++i)
                        {
                            var minDepthScale = 0.0f;
                            var dist = Distance.PointPointDistance(contactPts[i].Pos, SATPoint);

                            var depthDiv = System.Math.Max(JiggleMath.Epsilon, maxDist - minDist);
                            var depthScale = (dist - minDist) / depthDiv;

                            depth = (1.0f - depthScale) * oldDepth + minDepthScale * depthScale * oldDepth;

                            if (contactPointsFromOld)
                            {
                                if (numCollPts < MaxLocalStackScpi)
                                {
                                    collPts[numCollPts].R0 = contactPts[i].Pos - body0OldPos;
                                    collPts[numCollPts].R1 = contactPts[i].Pos - body1OldPos;
                                    collPts[numCollPts++].InitialPenetration = depth;
                                }
                            }
                            else
                            {
                                if (numCollPts < MaxLocalStackScpi)
                                {
                                    collPts[numCollPts].R0 = contactPts[i].Pos - body0NewPos;
                                    collPts[numCollPts].R1 = contactPts[i].Pos - body1NewPos;
                                    collPts[numCollPts++].InitialPenetration = depth;
                                }
                            }
                        }
                    }
                    else
                    {
                        Vector3.Subtract(ref SATPoint, ref body0NewPos, out var cp0);

                        Vector3.Subtract(ref SATPoint, ref body1NewPos, out var cp1);

                        if (numCollPts < MaxLocalStackScpi)
                        {
                            collPts[numCollPts].R0 = cp0;
                            collPts[numCollPts].R1 = cp1;
                            collPts[numCollPts++].InitialPenetration = oldDepth;
                        }
                    }


                    collisionFunctor.CollisionNotify(ref info, ref N, collPts, numCollPts);
                }
#if !USE_STACKALLOC
                FreeStackAlloc(collPtArray);
#endif
            }
        }
    }
}