using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace JigLibGame
{
    public class Camera : GameComponent
    {
        private Matrix view;
        private Matrix projection;

        private Vector3 position = new Vector3(0, 0, 10);
        private Vector2 angles = Vector2.Zero;

        private readonly int widthOver2;
        private readonly int heightOver2;

        private float fieldOfView = MathHelper.PiOver4;
        private float aspectRatio;
        private float nearPlaneDistance = 0.1f;
        private float farPlaneDistance = 10000.0f;

        private MouseState prevMouseState;

        public Camera(Game game) : base(game)
        {
            widthOver2 = game.Window.ClientBounds.Width / 2;
            heightOver2 = game.Window.ClientBounds.Height / 2;
            aspectRatio = game.Window.ClientBounds.Width / (float) game.Window.ClientBounds.Height;
            UpdateProjection();
            Mouse.SetPosition(widthOver2, heightOver2);
        }

        public Matrix View => view;

        public Matrix Projection
        {
            get => projection;
            set => projection = value;
        }

        public Matrix ViewProjection => view * projection;

        public Vector3 Position
        {
            get => position;
            set => position = value;
        }

        public float FieldOfView
        {
            get => fieldOfView;
            set
            {
                fieldOfView = value;
                UpdateProjection();
            }
        }

        public float AspectRatio
        {
            get => aspectRatio;
            set
            {
                aspectRatio = value;
                UpdateProjection();
            }
        }

        public float NearPlaneDistance
        {
            get => nearPlaneDistance;
            set
            {
                nearPlaneDistance = value;
                UpdateProjection();
            }
        }

        public float FarPlaneDistance
        {
            get => farPlaneDistance;
            set
            {
                farPlaneDistance = value;
                UpdateProjection();
            }
        }

        public Vector3 Target
        {
            get
            {
                var cameraRotation = Matrix.CreateRotationX(angles.X) * Matrix.CreateRotationY(angles.Y);
                return position + Vector3.Transform(Vector3.Forward, cameraRotation);
            }
            set
            {
                var forward = Vector3.Normalize(position - value);
                var right = Vector3.Normalize(Vector3.Cross(forward, Vector3.Up));
                var up = Vector3.Normalize(Vector3.Cross(right, forward));

                var test = Matrix.Identity;
                test.Forward = forward;
                test.Right = right;
                test.Up = up;
                angles.X = -(float) Math.Asin(test.M32);
                angles.Y = -(float) Math.Asin(test.M13);
            }
        }

        public override void Update(GameTime gameTime)
        {
            if (Enabled)
            {
                var elapsedTime = gameTime.ElapsedGameTime.Ticks / (double) TimeSpan.TicksPerSecond;
                ProcessInput((float) elapsedTime * 50.0f);
                UpdateView();

                base.Update(gameTime);
            }
        }

        private void ProcessInput(float amountOfMovement)
        {
            var moveVector = new Vector3();

            var keys = Keyboard.GetState();
            if (keys.IsKeyDown(Keys.D)) moveVector.X += amountOfMovement;
            if (keys.IsKeyDown(Keys.A)) moveVector.X -= amountOfMovement;
            if (keys.IsKeyDown(Keys.S)) moveVector.Z += amountOfMovement;
            if (keys.IsKeyDown(Keys.W)) moveVector.Z -= amountOfMovement;

            var cameraRotation = Matrix.CreateRotationX(angles.X) * Matrix.CreateRotationY(angles.Y);
            position += Vector3.Transform(moveVector, cameraRotation);

            var currentMouseState = Mouse.GetState();

            if (currentMouseState.RightButton == ButtonState.Pressed && prevMouseState.RightButton == ButtonState.Released)
            {
                Mouse.SetPosition(widthOver2, heightOver2);
            }
            else if (currentMouseState.RightButton == ButtonState.Pressed)
            {
                if (currentMouseState.X != widthOver2)
                    angles.Y -= amountOfMovement / 80.0f * (currentMouseState.X - widthOver2);
                if (currentMouseState.Y != heightOver2)
                    angles.X -= amountOfMovement / 80.0f * (currentMouseState.Y - heightOver2);

                if (angles.X > 1.4) angles.X = 1.4f;
                if (angles.X < -1.4) angles.X = -1.4f;
                if (angles.Y > Math.PI) angles.Y -= 2 * (float) Math.PI;
                if (angles.Y < -Math.PI) angles.Y += 2 * (float) Math.PI;

                Mouse.SetPosition(widthOver2, heightOver2);
            }

            prevMouseState = currentMouseState;
        }

        private void UpdateProjection()
        {
            projection = Matrix.CreatePerspectiveFieldOfView(fieldOfView, aspectRatio, nearPlaneDistance, farPlaneDistance);
        }

        private void UpdateView()
        {
            var cameraRotation = Matrix.CreateRotationX(angles.X) * Matrix.CreateRotationY(angles.Y);
            var targetPos = position + Vector3.Transform(Vector3.Forward, cameraRotation);

            var upVector = Vector3.Transform(Vector3.Up, cameraRotation);

            view = Matrix.CreateLookAt(position, targetPos, upVector);
        }
    }
}