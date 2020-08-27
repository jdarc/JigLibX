namespace JiggleGame {
    internal static class Program {
        private static void Main(string[] args) {
            using (JiggleGame game = new JiggleGame()) { game.Run(); }
        }
    }
}