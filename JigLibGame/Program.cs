using System;

namespace JigLibGame
{
    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            using var game = new JiggleGame();
            game.Run();
        }
    }
}