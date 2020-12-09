using System;
using System.Collections.Generic;
using JigLibX.Collision;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Plane = JigLibX.Geometry.Primitives.Plane;

namespace JigLibGame
{
    public static class BodyRenderExtensions
    {
        private static List<Vector3> CalcCirclePoints(float radius)
        {
            var elementsToCalc = 24;

            var stepSize = -360.0f / elementsToCalc;

            var l = new List<Vector3>();

            for (float slice = 0; slice <= elementsToCalc; slice++)
            {
                double stepRad = MathHelper.ToRadians(slice * stepSize);

                var x1 = (float) Math.Sin(stepRad);
                var y1 = (float) Math.Cos(stepRad);

                l.Add(new Vector3(x1, y1, 0) * radius);
            }

            return l;
        }

        private static void AddShapeToWireframe(List<Vector3> shape, List<VertexPositionColor> wireframe, Matrix orientation, Color color)
        {
            if (wireframe.Count > 0)
            {
                var v = wireframe[wireframe.Count - 1].Position;
                wireframe.Add(new VertexPositionColor(v, new Color(0, 0, 0, 0)));
                wireframe.Add(new VertexPositionColor(shape[0], new Color(0, 0, 0, 0)));
            }

            foreach (var point in shape)
                wireframe.Add(new VertexPositionColor(Vector3.Transform(point, orientation), color));
        }

        private static void AddLineToWireframe(Vector3 from, Vector3 to, List<VertexPositionColor> wireframe, Matrix orientation, Color color)
        {
            if (wireframe.Count > 0)
            {
                var v = wireframe[wireframe.Count - 1].Position;
                wireframe.Add(new VertexPositionColor(v, new Color(0, 0, 0, 0)));
                wireframe.Add(new VertexPositionColor(Vector3.Transform(from, orientation), new Color(0, 0, 0, 0)));
            }

            wireframe.Add(new VertexPositionColor(Vector3.Transform(from, orientation), color));
            wireframe.Add(new VertexPositionColor(Vector3.Transform(to, orientation), color));
        }

        private static void AddLinesToWireframe(List<Vector3> points, List<VertexPositionColor> wireframe, Matrix orientation, Color color)
        {
            for (var i = 0; i < points.Count; i += 2)
                AddLineToWireframe(points[i], points[i + 1], wireframe, orientation, color);
        }

        public static VertexPositionColor[] GetLocalSkinWireframe(this CollisionSkin skin)
        {
            var wireframe = new List<VertexPositionColor>();

            for (var i = 0; i < skin.NumPrimitives; i++)
            {
                var p = skin.GetPrimitiveLocal(i);
                var trans = p.TransformMatrix;

                if (p is Sphere)
                {
                    var np = (Sphere) p;

                    var spherePoints = CalcCirclePoints(np.Radius);

                    AddShapeToWireframe(spherePoints, wireframe, trans, Color.Blue);
                    AddShapeToWireframe(spherePoints, wireframe, Matrix.CreateRotationY(MathHelper.PiOver2) * trans, Color.Red);
                    AddShapeToWireframe(spherePoints, wireframe, Matrix.CreateRotationX(MathHelper.PiOver2) * trans, Color.Green);
                }
                else if (p is Capsule)
                {
                    var np = (Capsule) p;

                    var ball = CalcCirclePoints(np.Radius);
                    var cylPoints = new List<Vector3>();
                    var circlePoints = new List<Vector3>();
                    var sidePoints = new List<Vector3>();


                    foreach (var v in ball)
                    {
                        var t = Vector3.Transform(v, Matrix.CreateRotationX(MathHelper.PiOver2));
                        cylPoints.Add(t);
                    }

                    var len = np.Length;

                    sidePoints.Add(Vector3.Transform(new Vector3(np.Radius, len, 0), Matrix.CreateRotationX(MathHelper.PiOver2)));
                    sidePoints.Add(Vector3.Transform(new Vector3(np.Radius, 0, 0), Matrix.CreateRotationX(MathHelper.PiOver2)));
                    sidePoints.Add(Vector3.Transform(new Vector3(-np.Radius, 0, 0), Matrix.CreateRotationX(MathHelper.PiOver2)));
                    sidePoints.Add(Vector3.Transform(new Vector3(-np.Radius, len, 0), Matrix.CreateRotationX(MathHelper.PiOver2)));


                    AddShapeToWireframe(ball, wireframe, Matrix.CreateTranslation(new Vector3(0, 0, 0.0f * len)) * trans, Color.Green);
                    AddShapeToWireframe(ball, wireframe, Matrix.CreateTranslation(new Vector3(0, 0, 0.5f * np.Length)) * trans, Color.Green);
                    AddShapeToWireframe(ball, wireframe, Matrix.CreateTranslation(new Vector3(0, 0, 1.0f * np.Length)) * trans, Color.Green);


                    var zmat = Matrix.CreateRotationZ(MathHelper.PiOver2);
                    AddShapeToWireframe(cylPoints, wireframe, Matrix.CreateTranslation(new Vector3(0, 0, np.Length)) * zmat * trans, Color.Blue);
                    AddShapeToWireframe(cylPoints, wireframe, Matrix.CreateTranslation(new Vector3(0, 0, 0)) * zmat * trans, Color.Blue);
                    AddLineToWireframe(sidePoints[0], sidePoints[1], wireframe, zmat * trans, Color.Blue);
                    AddLineToWireframe(sidePoints[2], sidePoints[3], wireframe, zmat * trans, Color.Blue);


                    var xmat = Matrix.Identity;
                    AddShapeToWireframe(cylPoints, wireframe, Matrix.CreateTranslation(new Vector3(0, 0, np.Length)) * xmat * trans, Color.Red);
                    AddShapeToWireframe(cylPoints, wireframe, Matrix.CreateTranslation(new Vector3(0, 0, 0)) * xmat * trans, Color.Red);
                    AddLineToWireframe(sidePoints[0], sidePoints[1], wireframe, xmat * trans, Color.Red);
                    AddLineToWireframe(sidePoints[2], sidePoints[3], wireframe, xmat * trans, Color.Red);
                }
                else if (p is Box)
                {
                    var np = (Box) p;

                    var xPoints = new List<Vector3>();
                    var yPoints = new List<Vector3>();
                    var zPoints = new List<Vector3>();

                    var slen = np.SideLengths;

                    xPoints.Add(new Vector3(slen.X, slen.Y, slen.Z));
                    xPoints.Add(new Vector3(0, slen.Y, slen.Z));
                    xPoints.Add(new Vector3(slen.X, 0, slen.Z));
                    xPoints.Add(new Vector3(0, 0, slen.Z));
                    xPoints.Add(new Vector3(slen.X, slen.Y, 0));
                    xPoints.Add(new Vector3(0, slen.Y, 0));
                    xPoints.Add(new Vector3(slen.X, 0, 0));
                    xPoints.Add(new Vector3(0, 0, 0));

                    yPoints.Add(new Vector3(slen.X, slen.Y, slen.Z));
                    yPoints.Add(new Vector3(slen.X, 0, slen.Z));
                    yPoints.Add(new Vector3(0, slen.Y, slen.Z));
                    yPoints.Add(new Vector3(0, 0, slen.Z));
                    yPoints.Add(new Vector3(slen.X, slen.Y, 0));
                    yPoints.Add(new Vector3(slen.X, 0, 0));
                    yPoints.Add(new Vector3(0, slen.Y, 0));
                    yPoints.Add(new Vector3(0, 0, 0));

                    zPoints.Add(new Vector3(slen.X, slen.Y, slen.Z));
                    zPoints.Add(new Vector3(slen.X, slen.Y, 0));
                    zPoints.Add(new Vector3(0, slen.Y, slen.Z));
                    zPoints.Add(new Vector3(0, slen.Y, 0));
                    zPoints.Add(new Vector3(slen.X, 0, slen.Z));
                    zPoints.Add(new Vector3(slen.X, 0, 0));
                    zPoints.Add(new Vector3(0, 0, slen.Z));
                    zPoints.Add(new Vector3(0, 0, 0));

                    AddLinesToWireframe(xPoints, wireframe, trans, Color.Red);
                    AddLinesToWireframe(yPoints, wireframe, trans, Color.Green);
                    AddLinesToWireframe(zPoints, wireframe, trans, Color.Blue);
                }
                else if (p is AABox)
                {
                }
                else if (p is Heightmap)
                {
                    var hm = (Heightmap) p;

                    for (var e = 0; e < hm.Heights.Nx; e += 5)
                    for (var j = 0; j < hm.Heights.Nz; j += 5)
                    {
                        hm.GetSurfacePosAndNormal(out var point, out var normal, e, j);
                        AddLineToWireframe(point, point - 0.5f * normal, wireframe, trans, Color.GreenYellow);
                    }
                }
                else if (p is Plane)
                {
                }
                else if (p is TriangleMesh)
                {
                    var np = (TriangleMesh) p;

                    for (var j = 0; j < np.GetNumTriangles(); j++)
                    {
                        var t = np.GetTriangle(j);

                        var p1 = np.GetVertex(t.GetVertexIndex(0));
                        var p2 = np.GetVertex(t.GetVertexIndex(1));
                        var p3 = np.GetVertex(t.GetVertexIndex(2));

                        var tPoints = new List<Vector3>();

                        tPoints.Add(p1);
                        tPoints.Add(p2);
                        tPoints.Add(p3);
                        tPoints.Add(p1);

                        AddShapeToWireframe(tPoints, wireframe, trans, Color.Red);
                    }
                }
            }

            return wireframe.ToArray();
        }

        public static void TransformWireframe(this Body body, VertexPositionColor[] wireframe)
        {
            for (var i = 0; i < wireframe.Length; i++)
                wireframe[i].Position = Vector3.Transform(wireframe[i].Position, body.Orientation * Matrix.CreateTranslation(body.Position));
        }
    }
}