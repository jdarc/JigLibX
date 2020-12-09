using System;
using System.Collections.Generic;
using JigLibGame.PhysicObjects;
using JigLibX.Collision;
using JigLibX.Geometry;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace JigLibGame
{
    public class JiggleGame : Game
    {
        private readonly GraphicsDeviceManager graphics;
        private readonly ContentManager content;

        private Model boxModel;
        private Model sphereModel;
        private Model capsuleModel;
        private Model compoundModel;
        private Model terrainModel;
        private Model cylinderModel;
        private Model carModel;
        private Model wheelModel;
        private Model staticModel;
        private Model planeModel;
        private Model pinModel;

        private readonly PhysicsSystem physicSystem;
        private readonly DebugDrawer debugDrawer;
        private readonly Camera camera;

        private CarObject carObject;

        private readonly ConstraintWorldPoint objectController = new ConstraintWorldPoint();
        private readonly ConstraintVelocity damperController = new ConstraintVelocity();

        public JiggleGame()
        {
            var aspectRatio = GraphicsAdapter.DefaultAdapter.CurrentDisplayMode.AspectRatio;
            var screenHeight = GraphicsAdapter.DefaultAdapter.CurrentDisplayMode.Height * 90 / 100;
            var screenWidth = (int) (screenHeight * aspectRatio);
            content = new ContentManager(Services);
            graphics = new GraphicsDeviceManager(this)
            {
                PreferredBackBufferWidth = screenWidth,
                PreferredBackBufferHeight = screenHeight,
                SynchronizeWithVerticalRetrace = true
            };
            graphics.ApplyChanges();

            IsMouseVisible = true;
            Window.Title = "JigLibX Physic Library " + System.Reflection.Assembly.GetAssembly(typeof(PhysicsSystem))?.GetName().Version;

            IsFixedTimeStep = false;

            physicSystem = new PhysicsSystem();


            physicSystem.CollisionSystem = new CollisionSystemSAP();

            physicSystem.EnableFreezing = true;
            physicSystem.SolverType = PhysicsSystem.Solver.Normal;
            physicSystem.CollisionSystem.UseSweepTests = true;

            physicSystem.NumCollisionIterations = 10;
            physicSystem.NumContactIterations = 10;
            physicSystem.NumPenetrationRelaxtionTimesteps = 15;

            camera = new Camera(this);

            var physStats = new FrameRateCounter(this, physicSystem);

            debugDrawer = new DebugDrawer(this) {Enabled = false};

            Components.Add(physStats);
            Components.Add(camera);
            Components.Add(debugDrawer);

            physStats.DrawOrder = 2;
            debugDrawer.DrawOrder = 3;
        }

        public DebugDrawer DebugDrawer => debugDrawer;

        public Camera Camera => camera;

        protected override void LoadContent()
        {
            boxModel = content.Load<Model>("Content/box");
            sphereModel = content.Load<Model>("Content/sphere");
            capsuleModel = content.Load<Model>("Content/capsule");
            carModel = content.Load<Model>("Content/car");
            wheelModel = content.Load<Model>("Content/wheel");
            staticModel = content.Load<Model>("Content/StaticMesh");
            planeModel = content.Load<Model>("Content/plane");
            pinModel = content.Load<Model>("Content/pin");
            compoundModel = content.Load<Model>("Content/compound");
            cylinderModel = content.Load<Model>("Content/cylinder");

            try
            {
                terrainModel = content.Load<Model>("content/terrain");
                var heightmapObj = new HeightmapObject(this, terrainModel, Vector2.Zero);
                Components.Add(heightmapObj);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex);
                var planeObj = new PlaneObject(this, planeModel, 15.0f);
                Components.Add(planeObj);
            }

            var triObj = new TriangleMeshObject(this, staticModel, Matrix.Identity, Vector3.Zero);
            Components.Add(triObj);

            carObject = new CarObject(this, carModel, wheelModel, true, true, 30.0f, 5.0f, 4.7f, 5.0f, 0.20f, 0.4f, 0.05f, 0.45f, 0.3f, 1, 520.0f, physicSystem.Gravity.Length());
            carObject.Car.Chassis.Body.MoveTo(new Vector3(-5, -13, 5), Matrix.Identity);
            carObject.Car.EnableCar();
            carObject.Car.Chassis.Body.AllowFreezing = false;
            Components.Add(carObject);

            camera.Position = Vector3.Down * 12 + Vector3.Backward * 30.0f;

            CreateScene6();

            base.LoadContent();
        }

        private void CreateScene0()
        {
            var holder = new BoxObject(this, boxModel, new Vector3(5, 1, 1), Matrix.Identity, new Vector3(-10, -5, 5));
            holder.PhysicsBody.Immovable = true;
            Components.Add(holder);

            for (var i = 0; i < 5; i++)
            {
                var obj = new SphereObject(this, sphereModel, 0.5f, Matrix.Identity, new Vector3(-12 + i, -8, 5));
                obj.PhysicsBody.CollisionSkin.SetMaterialProperties(0, new MaterialProperties(1.0f, 0.2f, 0.2f));
                obj.PhysicsBody.AllowFreezing = false;

                var maxDist1 = new ConstraintMaxDistance(holder.PhysicsBody, new Vector3(-2 + i, -0.5f, 0.5f), obj.PhysicsBody, Vector3.Up * 0.5f, 3f);
                var maxDist2 = new ConstraintMaxDistance(holder.PhysicsBody, new Vector3(-2 + i, -0.5f, -0.5f), obj.PhysicsBody, Vector3.Up * 0.5f, 3f);
                maxDist1.EnableConstraint();
                maxDist2.EnableConstraint();

                Components.Add(obj);

                if (i == 4) obj.PhysicsBody.MoveTo(new Vector3(-6, -6, 5), Matrix.Identity);
                if (i == 3) obj.PhysicsBody.MoveTo(new Vector3(-7, -6, 5), Matrix.Identity);
            }
        }

        private void CreateScene1(int dim)
        {
            for (var x = 0; x < dim; x++)
            for (var y = 0; y < dim; y++)
                if (y % 2 == 0)
                    Components.Add(new BoxObject(this, boxModel, new Vector3(1, 1, 1), Matrix.Identity, new Vector3(x * 1.01f - 10.0f, y * 1.01f - 14.5f, 25)));
                else
                    Components.Add(new BoxObject(this, boxModel, new Vector3(1, 1, 1), Matrix.Identity, new Vector3(x * 1.01f - 10.5f, y * 1.01f - 14.5f, 25)));
        }

        private void CreateScene2()
        {
            for (var i = 0; i < 20; i++) Components.Add(SpawnPrimitive(new Vector3(2, 3 * i + 10, 2), Matrix.Identity));
            for (var i = 0; i < 20; i++)
                Components.Add(SpawnPrimitive(new Vector3(2, 3 * i + 10, -2), Matrix.Identity));
            for (var i = 0; i < 20; i++)
                Components.Add(SpawnPrimitive(new Vector3(-2, 3 * i + 10, -2), Matrix.Identity));
            for (var i = 0; i < 20; i++)
                Components.Add(SpawnPrimitive(new Vector3(-2, 3 * i + 10, -2), Matrix.Identity));
        }

        private void CreateScene4(int dim)
        {
            for (var x = 0; x < dim; x++)
            {
                var obj = new BoxObject(this, boxModel, Vector3.One, Matrix.Identity, new Vector3(0, x * 1.01f - 14.0f, 25));
                Components.Add(obj);
            }
        }

        private void CreateScene5(int dim)
        {
            for (var x = 0; x < dim; x++)
            for (var e = x; e < dim; e++)
                Components.Add(new BoxObject(this, boxModel, Vector3.One, Matrix.Identity, new Vector3(e - 0.5f * x, x * 1.01f - 14, 25)));
        }

        private void CreateScene6()
        {
            for (var i = 0; i < 10; i += 2)
            {
                var boxObj0 = new BoxObject(this, boxModel, new Vector3(1, 1f, 3), Matrix.Identity, new Vector3(0, i * 1f - 14, 1));
                var boxObj1 = new BoxObject(this, boxModel, new Vector3(1, 1f, 3), Matrix.Identity, new Vector3(1, i * 1f - 14, 1));
                var boxObj2 = new BoxObject(this, boxModel, new Vector3(1, 1f, 3), Matrix.Identity, new Vector3(2, i * 1f - 14, 1));
                Components.Add(boxObj0);
                Components.Add(boxObj1);
                Components.Add(boxObj2);

                var boxObj3 = new BoxObject(this, boxModel, new Vector3(3, 1f, 1), Matrix.Identity, new Vector3(1, i * 1f + 1f - 14, 0));
                var boxObj4 = new BoxObject(this, boxModel, new Vector3(3, 1f, 1), Matrix.Identity, new Vector3(1, i * 1f + 1f - 14, 1));
                var boxObj5 = new BoxObject(this, boxModel, new Vector3(3, 1f, 1), Matrix.Identity, new Vector3(1, i * 1f + 1f - 14, 2));
                Components.Add(boxObj3);
                Components.Add(boxObj4);
                Components.Add(boxObj5);
            }

            for (var i = 0; i < 10; i++)
            {
                var cyl = new CylinderObject(this, 0.5f, 1.0f, new Vector3(5, i * 1.01f - 14.2f, 0), cylinderModel);
                Components.Add(cyl);
            }

            RagdollObject rgd;


            for (var e = 0; e < 2; e++)
            for (var i = 0; i < 2; i++)
            {
                rgd = new RagdollObject(this, capsuleModel, sphereModel, boxModel, RagdollObject.RagdollType.Simple, 1.0f);
                rgd.Position = new Vector3(e * 2, -14, 10 + i * 2);
                rgd.PutToSleep();
            }


            for (var x = 0; x < 8; x++)
            for (var y = 0; y < 3; y++)
                if (y % 2 == 0)
                    Components.Add(new BoxObject(this, boxModel, new Vector3(1, 1, 1), Matrix.Identity, new Vector3(x * 1.01f - 10.0f, y * 1.01f - 14.5f, 0)));
                else
                    Components.Add(new BoxObject(this, boxModel, new Vector3(1, 1, 1), Matrix.Identity, new Vector3(x * 1.01f - 10.5f, y * 1.01f - 14.5f, 0)));
        }

        private void CreateScene7()
        {
            var rotM = Matrix.CreateRotationY(0.5f);

            for (var i = 0; i < 15; i += 2)
            {
                var boxObj0 = new BoxObject(this, boxModel, new Vector3(1, 1, 4), rotM, new Vector3(0, i - 10, 25));
                var boxObj2 = new BoxObject(this, boxModel, new Vector3(1, 1, 4), rotM, new Vector3(2, i - 10, 25));
                Components.Add(boxObj0);
                Components.Add(boxObj2);

                var boxObj3 = new BoxObject(this, boxModel, new Vector3(4, 1, 1), rotM, new Vector3(1, i + 1 - 10, 24));
                var boxObj5 = new BoxObject(this, boxModel, new Vector3(4, 1, 1), rotM, new Vector3(1, i + 1 - 10, 26));
                Components.Add(boxObj3);
                Components.Add(boxObj5);
            }
        }

        private void CreateScene8()
        {
            for (var e = 0; e < 5; e++)
            for (var i = e; i < 5; i++)
            {
                var bp = new BowlingPin(this, pinModel, Matrix.CreateRotationX(-MathHelper.PiOver2) * Matrix.CreateRotationY(i * e), new Vector3(e, -14.2f, i));
                Components.Add(bp);
                bp.PhysicsBody.SetDeactivationTime(5.0f);
            }
        }

        private void CreateScene9()
        {
            RagdollObject rgd;


            for (var e = 0; e < 5; e++)
            for (var i = 0; i < 5; i++)
            {
                rgd = new RagdollObject(this, capsuleModel, sphereModel, boxModel, RagdollObject.RagdollType.Simple, 1.0f);
                rgd.Position = new Vector3(e * 2, -14, 10 + i * 2);
                rgd.PutToSleep();
            }
        }

        private void CreateScene3()
        {
            var chainBoxes = new List<BoxObject>();

            for (var i = 0; i < 25; i++)
            {
                var boxObject = new BoxObject(this, boxModel, Vector3.One, Matrix.Identity, new Vector3(i, 25 - i, 0));
                if (i == 0) boxObject.PhysicsBody.Immovable = true;
                chainBoxes.Add(boxObject);
            }

            for (var i = 1; i < 25; i++)
            {
                var hingeJoint = new HingeJoint();
                hingeJoint.Initialise(chainBoxes[i - 1].PhysicsBody, chainBoxes[i].PhysicsBody, Vector3.Backward, new Vector3(0.5f, -0.5f, 0.0f), 0.5f, 90.0f, 90.0f, 0.0f, 0.2f);
                hingeJoint.EnableController();
                hingeJoint.EnableHinge();
            }

            foreach (var obj in chainBoxes) Components.Add(obj);
        }

        protected override void UnloadContent()
        {
            content.Unload();
            base.UnloadContent();
        }

        private readonly bool singleStep = false;
        private bool leftButton;

        private float camPickDistance;
        private bool middleButton;
        private int oldWheel;

        protected override void Update(GameTime gameTime)
        {
            var keyState = Keyboard.GetState();
            var mouseState = Mouse.GetState();

            if (keyState.IsKeyDown(Keys.Escape)) Exit();

            if (mouseState.MiddleButton == ButtonState.Pressed)
            {
                if (middleButton == false)
                {
                    var ray = RayTo(mouseState.X, mouseState.Y);

                    var pred = new ImmovableSkinPredicate();

                    physicSystem.CollisionSystem.SegmentIntersect(out var frac, out var skin, out var pos, out var normal, new Segment(camera.Position, ray * 1000.0f), pred);

                    if (skin?.Owner != null)
                        if (!skin.Owner.Immovable)
                        {
                            var delta = pos - skin.Owner.Position;
                            delta = Vector3.Transform(delta, Matrix.Transpose(skin.Owner.Orientation));

                            camPickDistance = (camera.Position - pos).Length();
                            oldWheel = mouseState.ScrollWheelValue;

                            skin.Owner.SetActive();
                            objectController.Destroy();
                            damperController.Destroy();
                            objectController.Initialise(skin.Owner, delta, pos);
                            damperController.Initialise(skin.Owner, ConstraintVelocity.ReferenceFrame.Body, Vector3.Zero, Vector3.Zero);
                            objectController.EnableConstraint();
                            damperController.EnableConstraint();
                        }

                    middleButton = true;
                }

                if (objectController.IsConstraintEnabled && objectController.Body != null)
                {
                    var delta = objectController.Body.Position - camera.Position;
                    var ray = RayTo(mouseState.X, mouseState.Y);
                    ray.Normalize();
                    float deltaWheel = mouseState.ScrollWheelValue - oldWheel;
                    camPickDistance += deltaWheel * 0.01f;
                    var result = camera.Position + camPickDistance * ray;
                    oldWheel = mouseState.ScrollWheelValue;
                    objectController.WorldPosition = result;
                    objectController.Body.SetActive();
                }
            }
            else
            {
                objectController.DisableConstraint();
                damperController.DisableConstraint();
                middleButton = false;
            }

            if (mouseState.LeftButton == ButtonState.Pressed && leftButton == false)
            {
                var physObj = SpawnPrimitive(camera.Position, Matrix.CreateRotationX(0.5f));
                physObj.PhysicsBody.Velocity = (camera.Target - camera.Position) * 20.0f;
                Components.Add(physObj);
                leftButton = true;
            }

            if (mouseState.LeftButton == ButtonState.Released) leftButton = false;

            var pressedKeys = keyState.GetPressedKeys();

            if (pressedKeys.Length != 0)
                switch (pressedKeys[0])
                {
                    case Keys.D1:
                        ResetScene();
                        CreateScene1(9);
                        break;
                    case Keys.D2:
                        ResetScene();
                        CreateScene2();
                        break;
                    case Keys.D3:
                        ResetScene();
                        CreateScene3();
                        break;
                    case Keys.D4:
                        ResetScene();
                        CreateScene4(12);
                        break;
                    case Keys.D5:
                        ResetScene();
                        CreateScene5(20);
                        break;
                    case Keys.D6:
                        ResetScene();
                        CreateScene6();
                        break;
                    case Keys.D7:
                        ResetScene();
                        CreateScene7();
                        break;
                    case Keys.D8:
                        ResetScene();
                        CreateScene8();
                        break;
                    case Keys.D9:
                        ResetScene();
                        CreateScene9();
                        break;
                    case Keys.D0:
                        ResetScene();
                        CreateScene0();
                        break;
                }

            debugDrawer.Enabled = keyState.IsKeyDown(Keys.C);

            if (keyState.IsKeyDown(Keys.Up) || keyState.IsKeyDown(Keys.Down))
            {
                if (keyState.IsKeyDown(Keys.Up))
                    carObject.Car.Accelerate = 1.0f;
                else
                    carObject.Car.Accelerate = -1.0f;
            }
            else
                carObject.Car.Accelerate = 0.0f;

            if (keyState.IsKeyDown(Keys.Left) || keyState.IsKeyDown(Keys.Right))
            {
                if (keyState.IsKeyDown(Keys.Left))
                    carObject.Car.Steer = 1.0f;
                else
                    carObject.Car.Steer = -1.0f;
            }
            else
                carObject.Car.Steer = 0.0f;

            if (keyState.IsKeyDown(Keys.B))
                carObject.Car.HBrake = 1.0f;
            else
                carObject.Car.HBrake = 0.0f;


            if (singleStep == true && keyState.IsKeyDown(Keys.Space) == false)
            {
            }
            else
            {
                var timeStep = (float) gameTime.ElapsedGameTime.Ticks / TimeSpan.TicksPerSecond;
                if (timeStep < 1.0f / 60.0f)
                    physicSystem.Integrate(timeStep);
                else
                    physicSystem.Integrate(1.0f / 60.0f);
            }

            base.Update(gameTime);
        }

        private void ResetScene()
        {
            var toBeRemoved = new List<PhysicObject>();
            foreach (var gameComponent in Components)
            {
                var gc = (GameComponent) gameComponent;
                if (gc is PhysicObject && !(gc is HeightmapObject) && !(gc is CarObject) && !(gc is TriangleMeshObject) && !(gc is PlaneObject))
                {
                    var physObj = gc as PhysicObject;
                    toBeRemoved.Add(physObj);
                }
            }

            foreach (var physObj in toBeRemoved)
            {
                physObj.PhysicsBody.DisableBody();
                Components.Remove(physObj);


                physObj.Dispose();
            }

            var count = physicSystem.Controllers.Count;
            for (var i = 0; i < count; i++) physicSystem.Controllers[0].DisableController();
            count = physicSystem.Constraints.Count;
            for (var i = 0; i < count; i++) physicSystem.RemoveConstraint(physicSystem.Constraints[0]);
        }

        public void SetRenderStates()
        {
            graphics.GraphicsDevice.BlendState = BlendState.AlphaBlend;
            graphics.GraphicsDevice.DepthStencilState = DepthStencilState.Default;
            graphics.GraphicsDevice.SamplerStates[0] = SamplerState.LinearWrap;
        }

        private Vector3 RayTo(int x, int y)
        {
            var nearSource = new Vector3(x, y, 0);
            var farSource = new Vector3(x, y, 1);

            var world = Matrix.CreateTranslation(0, 0, 0);

            var nearPoint = graphics.GraphicsDevice.Viewport.Unproject(nearSource, camera.Projection, camera.View, world);
            var farPoint = graphics.GraphicsDevice.Viewport.Unproject(farSource, camera.Projection, camera.View, world);

            var direction = farPoint - nearPoint;

            return direction;
        }

        private readonly Random random = new Random();

        private PhysicObject SpawnPrimitive(Vector3 pos, Matrix ori)
        {
            var prim = random.Next(3);

            var a = 1.0f + (float) random.NextDouble() * 1.0f;
            var b = a + (float) random.NextDouble() * 0.5f;
            var c = 2.0f / a / b;

            PhysicObject physicObj = prim switch
            {
                0 => new BoxObject(this, boxModel, new Vector3(a, b, c), ori, pos),
                1 => new SphereObject(this, sphereModel, 0.5f, ori, pos),
                2 => new CapsuleObject(this, capsuleModel, 0.5f, 1f, ori, pos),
                _ => new SphereObject(this, sphereModel, random.Next(5, 15), ori, pos)
            };

            return physicObj;
        }

        protected override void Draw(GameTime gameTime)
        {
            graphics.GraphicsDevice.Clear(Color.CornflowerBlue);

            base.Draw(gameTime);
        }
    }
}