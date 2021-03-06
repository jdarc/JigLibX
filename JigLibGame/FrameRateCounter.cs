using System;
using JigLibX.Physics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace JigLibGame
{
    public class FrameRateCounter : DrawableGameComponent
    {
        private readonly ContentManager content;
        private SpriteBatch spriteBatch;
        private SpriteFont spriteFont, spriteFont2;
        private Texture2D tex;

        private PhysicsSystem physics;

        private int frameRate;
        private int frameCounter;
        private TimeSpan elapsedTime = TimeSpan.Zero;

        private int bbWidth, bbHeight;

        public FrameRateCounter(Game game, PhysicsSystem physics) : base(game)
        {
            content = new ContentManager(game.Services);
            this.physics = physics;
        }

        private void GraphicsDevice_DeviceReset(object sender, EventArgs e)
        {
            bbWidth = GraphicsDevice.PresentationParameters.BackBufferWidth;
            bbHeight = GraphicsDevice.PresentationParameters.BackBufferHeight;
        }

        protected override void LoadContent()
        {
            GraphicsDevice.DeviceReset += new EventHandler<EventArgs>(GraphicsDevice_DeviceReset);
            GraphicsDevice_DeviceReset(null, null);

            spriteBatch = new SpriteBatch(GraphicsDevice);
            spriteFont = content.Load<SpriteFont>("Content/Font");
            spriteFont2 = content.Load<SpriteFont>("Content/Font2");
            tex = content.Load<Texture2D>("Content/jiglib");
        }

        protected override void UnloadContent()
        {
            content.Unload();
        }

        public override void Update(GameTime gameTime)
        {
            elapsedTime += gameTime.ElapsedGameTime;

            if (elapsedTime > TimeSpan.FromSeconds(1))
            {
                elapsedTime -= TimeSpan.FromSeconds(1);
                frameRate = frameCounter;
                frameCounter = 0;
            }

            var state = Keyboard.GetState();
            if (!DrawHelp && state.IsKeyDown(Keys.H)) DrawHelp = true;
            if (DrawHelp && state.IsKeyUp(Keys.H)) DrawHelp = false;
        }

        public bool DrawHelp { get; internal set; }

        public override void Draw(GameTime gameTime)
        {
            frameCounter++;

            var fps = frameRate.ToString();

            spriteBatch.Begin(SpriteSortMode.Deferred, null, null, null, null, null, null);
            spriteBatch.Draw(tex, new Rectangle(0, 0, 100, 80), Color.White);
            spriteBatch.DrawString(spriteFont, fps, new Vector2(11, 6), Color.Black);
            spriteBatch.DrawString(spriteFont, fps, new Vector2(12, 7), Color.Yellow);

            if (!DrawHelp)
            {
                spriteBatch.DrawString(spriteFont2, "Press 'h' for help, 'c' for debug view", new Vector2(0, bbHeight - 15), Color.White);
            }
            else
            {
                spriteBatch.DrawString(spriteFont2, "Use 'wasd' and hold right mouse button to move the camera around", new Vector2(0, bbHeight - 15 * 4), Color.White);
                spriteBatch.DrawString(spriteFont2, "Use the arrow keys to drive the car", new Vector2(0, bbHeight - 15 * 3), Color.White);
                spriteBatch.DrawString(spriteFont2, "Left mouse click to shoot a random body", new Vector2(0, bbHeight - 15 * 2), Color.White);
                spriteBatch.DrawString(spriteFont2, "Use mousewheel to pick and drag objects", new Vector2(0, bbHeight - 15 * 1), Color.White);
            }

            spriteBatch.End();

            ((JiggleGame) Game).SetRenderStates();
        }
    }
}