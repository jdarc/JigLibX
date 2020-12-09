using JigLibX.Math;
using JigLibX.Utils;
using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public class Box : Primitive
    {
        public readonly struct Edge
        {
            public readonly BoxPointIndex Ind0;
            public readonly BoxPointIndex Ind1;

            public Edge(BoxPointIndex ind0, BoxPointIndex ind1)
            {
                Ind0 = ind0;
                Ind1 = ind1;
            }
        }

        public enum BoxPointIndex
        {
            Brd,
            Bru,
            Bld,
            Blu,
            Frd,
            Fru,
            Fld,
            Flu
        }

        public Vector3 SideLengths;

        private static readonly Edge[] Edges = {
            new Edge(BoxPointIndex.Brd, BoxPointIndex.Bru),
            new Edge(BoxPointIndex.Brd, BoxPointIndex.Bld),
            new Edge(BoxPointIndex.Brd, BoxPointIndex.Frd),
            new Edge(BoxPointIndex.Bld, BoxPointIndex.Blu),
            new Edge(BoxPointIndex.Bld, BoxPointIndex.Fld),
            new Edge(BoxPointIndex.Frd, BoxPointIndex.Fru),
            new Edge(BoxPointIndex.Frd, BoxPointIndex.Fld),
            new Edge(BoxPointIndex.Bru, BoxPointIndex.Blu),
            new Edge(BoxPointIndex.Bru, BoxPointIndex.Fru),
            new Edge(BoxPointIndex.Blu, BoxPointIndex.Flu),
            new Edge(BoxPointIndex.Fru, BoxPointIndex.Flu),
            new Edge(BoxPointIndex.Fld, BoxPointIndex.Flu)
        };

        private readonly Vector3[] _outPoints = new Vector3[8];

        public Box(Vector3 pos, Matrix orient, Vector3 SideLengths) : base((int) PrimitiveType.Box)
        {
            transform = new Transform(pos, orient);
            this.SideLengths = SideLengths;
        }

        public override Primitive Clone()
        {
            return new Box(transform.Position, transform.Orientation, SideLengths);
        }

        public Vector3 Position
        {
            get => transform.Position;
            set => transform.Position = value;
        }

        public Vector3 GetCentre()
        {
            var result = new Vector3(SideLengths.X * 0.5f, SideLengths.Y * 0.5f, SideLengths.Z * 0.5f);

            Vector3.TransformNormal(ref result, ref transform.Orientation, out result);
            Vector3.Add(ref result, ref transform.Position, out result);

            return result;
        }

        public void GetCentre(out Vector3 centre)
        {
            centre = new Vector3();
            centre.X = SideLengths.X * 0.5f * transform.Orientation.M11 + SideLengths.Y * 0.5f * transform.Orientation.M21 + SideLengths.Z * 0.5f * transform.Orientation.M31 + transform.Orientation.M41 + transform.Position.X;
            centre.Y = SideLengths.X * 0.5f * transform.Orientation.M12 + SideLengths.Y * 0.5f * transform.Orientation.M22 + SideLengths.Z * 0.5f * transform.Orientation.M32 + transform.Orientation.M42 + transform.Position.Y;
            centre.Z = SideLengths.X * 0.5f * transform.Orientation.M13 + SideLengths.Y * 0.5f * transform.Orientation.M23 + SideLengths.Z * 0.5f * transform.Orientation.M33 + transform.Orientation.M43 + transform.Position.Z;
        }

        public float GetBoundingRadiusAroundCentre() => 0.5f * SideLengths.Length();

        public Matrix Orientation
        {
            get => transform.Orientation;
            set => transform.Orientation = value;
        }
        
        public void Expand(Vector3 amount)
        {
            transform.Position -= Vector3.TransformNormal(amount, transform.Orientation);
            SideLengths += SideLengths + 2.0f * amount;
        }

        public Vector3 GetHalfSideLengths()
        {
            var result = new Vector3(SideLengths.X * 0.5f, SideLengths.Y * 0.5f, SideLengths.Z * 0.5f);

            return result;
        }

        public Vector3 GetSide(int i)
        {
            return JiggleUnsafe.Get(transform.Orientation, i) * JiggleUnsafe.Get(ref SideLengths, i);
        }

        public float GetSqDistanceToPoint(out Vector3 closestBoxPoint, Vector3 point)
        {
            closestBoxPoint = Vector3.TransformNormal(point - transform.Position, Matrix.Transpose(transform.Orientation));

            var sqDistance = 0.0f;
            float delta;

            if (closestBoxPoint.X < 0.0f)
            {
                sqDistance += closestBoxPoint.X * closestBoxPoint.X;
                closestBoxPoint.X = 0.0f;
            }
            else if (closestBoxPoint.X > SideLengths.X)
            {
                delta = closestBoxPoint.X - SideLengths.X;
                sqDistance += delta * delta;
                closestBoxPoint.X = SideLengths.X;
            }

            if (closestBoxPoint.Y < 0.0f)
            {
                sqDistance += closestBoxPoint.Y * closestBoxPoint.Y;
                closestBoxPoint.Y = 0.0f;
            }
            else if (closestBoxPoint.Y > SideLengths.Y)
            {
                delta = closestBoxPoint.Y - SideLengths.Y;
                sqDistance += delta * delta;
                closestBoxPoint.Y = SideLengths.Y;
            }

            if (closestBoxPoint.Z < 0.0f)
            {
                sqDistance += closestBoxPoint.Z * closestBoxPoint.Z;
                closestBoxPoint.Z = 0.0f;
            }
            else if (closestBoxPoint.Z > SideLengths.Z)
            {
                delta = closestBoxPoint.Z - SideLengths.Z;
                sqDistance += delta * delta;
                closestBoxPoint.Z = SideLengths.Z;
            }

            Vector3.TransformNormal(ref closestBoxPoint, ref transform.Orientation, out closestBoxPoint);
            Vector3.Add(ref transform.Position, ref closestBoxPoint, out closestBoxPoint);

            return sqDistance;
        }

        public float GetDistanceToPoint(out Vector3 closestBoxPoint, Vector3 point)
        {
            return (float) System.Math.Sqrt(GetSqDistanceToPoint(out closestBoxPoint, point));
        }

        public void GetSpan(out float min, out float max, Vector3 axis)
        {
            var right = transform.Orientation.Right;
            var up = transform.Orientation.Up;
            var back = transform.Orientation.Backward;

            Vector3.Dot(ref axis, ref right, out var s);
            Vector3.Dot(ref axis, ref up, out var u);
            Vector3.Dot(ref axis, ref back, out var d);

            s = System.Math.Abs(s * 0.5f * SideLengths.X);
            u = System.Math.Abs(u * 0.5f * SideLengths.Y);
            d = System.Math.Abs(d * 0.5f * SideLengths.Z);

            var r = s + u + d;
            GetCentre(out right);
            Vector3.Dot(ref right, ref axis, out var p);
            min = p - r;
            max = p + r;
        }

        public void GetSpan(out float min, out float max, ref Vector3 axis)
        {
            var right = transform.Orientation.Right;
            var up = transform.Orientation.Up;
            var back = transform.Orientation.Backward;

            Vector3.Dot(ref axis, ref right, out var s);
            Vector3.Dot(ref axis, ref up, out var u);
            Vector3.Dot(ref axis, ref back, out var d);

            s = System.Math.Abs(s * 0.5f * SideLengths.X);
            u = System.Math.Abs(u * 0.5f * SideLengths.Y);
            d = System.Math.Abs(d * 0.5f * SideLengths.Z);

            var r = s + u + d;
            GetCentre(out right);
            Vector3.Dot(ref right, ref axis, out var p);
            min = p - r;
            max = p + r;
        }

        public void GetCornerPoints(out Vector3[] pts)
        {
            pts = _outPoints;
            pts[(int) BoxPointIndex.Brd] = transform.Position;
            pts[(int) BoxPointIndex.Frd] = transform.Position + SideLengths.X * transform.Orientation.Right;
            pts[(int) BoxPointIndex.Bld] = transform.Position + SideLengths.Y * transform.Orientation.Up;
            pts[(int) BoxPointIndex.Bru] = transform.Position + SideLengths.Z * transform.Orientation.Backward;
            pts[(int) BoxPointIndex.Fld] = pts[(int) BoxPointIndex.Bld] + SideLengths.X * transform.Orientation.Right;
            pts[(int) BoxPointIndex.Blu] = pts[(int) BoxPointIndex.Bru] + SideLengths.Y * transform.Orientation.Up;
            pts[(int) BoxPointIndex.Fru] = pts[(int) BoxPointIndex.Frd] + SideLengths.Z * transform.Orientation.Backward;
            pts[(int) BoxPointIndex.Flu] = pts[(int) BoxPointIndex.Fld] + SideLengths.Z * transform.Orientation.Backward;
        }

        public void GetEdges(out Edge[] edg)
        {
            edg = Edges;
        }

        public void GetEdgesAroundPoint(out int[] edgeIndices, BoxPointIndex pt)
        {
            edgeIndices = new int[3];
            var ind = 0;

            for (var i = 0; i < Edges.Length; ++i)
            {
                if (Edges[i].Ind0 == pt || Edges[i].Ind1 == pt) edgeIndices[ind++] = i;
                if (ind == 3) return;
            }
        }

        public override float GetSurfaceArea()
        {
            return 2.0f * (SideLengths.X * SideLengths.Y + SideLengths.X * SideLengths.Z + SideLengths.Y * SideLengths.Z);
        }

        public override float GetVolume()
        {
            return SideLengths.X * SideLengths.Y * SideLengths.Z;
        }

        public override Transform Transform
        {
            get => transform;
            set => transform = value;
        }

        public override bool SegmentIntersect(out float fracOut, out Vector3 posOut, out Vector3 normalOut, Segment seg)
        {
            fracOut = float.MaxValue;
            posOut = normalOut = Vector3.Zero;


            var min = float.MinValue;
            var max = float.MaxValue;


            var centre = GetCentre();
            Vector3.Subtract(ref centre, ref seg.Origin, out var p);
            Vector3 h;
            h.X = SideLengths.X * 0.5f;
            h.Y = SideLengths.Y * 0.5f;
            h.Z = SideLengths.Z * 0.5f;

            var dirMax = 0;
            var dirMin = 0;
            int dir;

            var e = Vector3.Dot(transform.Orientation.Right, p);
            var f = Vector3.Dot(transform.Orientation.Right, seg.Delta);

            if (System.Math.Abs(f) > JiggleMath.Epsilon)
            {
                var t1 = (e + h.X) / f;
                var t2 = (e - h.X) / f;

                if (t1 > t2)
                {
                    var tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                }

                if (t1 > min)
                {
                    min = t1;
                    dirMin = 0;
                }

                if (t2 < max)
                {
                    max = t2;
                    dirMax = 0;
                }

                if (min > max) return false;

                if (max < 0.0f) return false;
            }
            else if (-e - h.X > 0.0f || -e + h.X < 0.0f)
            {
                return false;
            }

            e = Vector3.Dot(transform.Orientation.Up, p);
            f = Vector3.Dot(transform.Orientation.Up, seg.Delta);

            if (System.Math.Abs(f) > JiggleMath.Epsilon)
            {
                var t1 = (e + h.Y) / f;
                var t2 = (e - h.Y) / f;

                if (t1 > t2)
                {
                    var tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                }

                if (t1 > min)
                {
                    min = t1;
                    dirMin = 1;
                }

                if (t2 < max)
                {
                    max = t2;
                    dirMax = 1;
                }

                if (min > max) return false;

                if (max < 0.0f) return false;
            }
            else if (-e - h.Y > 0.0f || -e + h.Y < 0.0f)
            {
                return false;
            }

            e = Vector3.Dot(transform.Orientation.Backward, p);
            f = Vector3.Dot(transform.Orientation.Backward, seg.Delta);

            if (System.Math.Abs(f) > JiggleMath.Epsilon)
            {
                var t1 = (e + h.Z) / f;
                var t2 = (e - h.Z) / f;

                if (t1 > t2)
                {
                    var tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                }

                if (t1 > min)
                {
                    min = t1;
                    dirMin = 2;
                }

                if (t2 < max)
                {
                    max = t2;
                    dirMax = 2;
                }

                if (min > max) return false;

                if (max < 0.0f) return false;
            }
            else if (-e - h.Z > 0.0f || -e + h.Z < 0.0f)
            {
                return false;
            }

            if (min > 0.0f)
            {
                dir = dirMin;
                fracOut = min;
            }
            else
            {
                dir = dirMax;
                fracOut = max;
            }

            if (dir == 0)
            {
                fracOut = MathHelper.Clamp(fracOut, 0.0f, 1.0f);
                posOut = seg.GetPoint(fracOut);
                if (Vector3.Dot(transform.Orientation.Right, seg.Delta) > 0.0f)
                    normalOut = -transform.Orientation.Right;
                else
                    normalOut = transform.Orientation.Right;
            }
            else if (dir == 1)
            {
                fracOut = MathHelper.Clamp(fracOut, 0.0f, 1.0f);
                posOut = seg.GetPoint(fracOut);
                if (Vector3.Dot(transform.Orientation.Up, seg.Delta) > 0.0f)
                    normalOut = -transform.Orientation.Up;
                else
                    normalOut = transform.Orientation.Up;
            }
            else
            {
                fracOut = MathHelper.Clamp(fracOut, 0.0f, 1.0f);
                posOut = seg.GetPoint(fracOut);
                if (Vector3.Dot(transform.Orientation.Backward, seg.Delta) > 0.0f)
                    normalOut = -transform.Orientation.Backward;
                else
                    normalOut = transform.Orientation.Backward;
            }

            return true;
        }

        public override void GetMassProperties(PrimitiveProperties primitiveProperties, out float mass, out Vector3 centerOfMass, out Matrix inertiaTensor)
        {
            if (primitiveProperties.MassType == PrimitiveProperties.MassTypeEnum.Mass)
            {
                mass = primitiveProperties.MassOrDensity;
            }
            else
            {
                if (primitiveProperties.MassDistribution == PrimitiveProperties.MassDistributionEnum.Solid)
                    mass = GetVolume() * primitiveProperties.MassOrDensity;
                else
                    mass = GetSurfaceArea() * primitiveProperties.MassOrDensity;
            }

            centerOfMass = GetCentre();


            var (x, y, z) = SideLengths;

            inertiaTensor = Matrix.Identity;
            inertiaTensor.M11 = 1.0f / 12.0f * mass * (y * y + z * z);
            inertiaTensor.M22 = 1.0f / 12.0f * mass * (x * x + z * z);
            inertiaTensor.M33 = 1.0f / 12.0f * mass * (x * x + y * y);

            inertiaTensor = Matrix.Transpose(transform.Orientation) * inertiaTensor * transform.Orientation;

            inertiaTensor.M11 += mass * (centerOfMass.Y * centerOfMass.Y + centerOfMass.Z * centerOfMass.Z);
            inertiaTensor.M22 += mass * (centerOfMass.Z * centerOfMass.Z + centerOfMass.X * centerOfMass.X);
            inertiaTensor.M33 += mass * (centerOfMass.X * centerOfMass.X + centerOfMass.Y * centerOfMass.Y);

            inertiaTensor.M12 = inertiaTensor.M21 = inertiaTensor.M12 - mass * centerOfMass.X * centerOfMass.Y;
            inertiaTensor.M23 = inertiaTensor.M32 = inertiaTensor.M23 - mass * centerOfMass.Y * centerOfMass.Z;
            inertiaTensor.M31 = inertiaTensor.M13 = inertiaTensor.M31 - mass * centerOfMass.Z * centerOfMass.X;
        }
    }
}