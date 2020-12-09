namespace JigLibX.Collision.Detection
{
    public abstract class DetectFunctor
    {
        public const int MaxLocalStackTris = 2048;
        public const int MaxLocalStackScpi = 10;

        protected DetectFunctor(int primType0, int primType1)
        {
            Type0 = primType0;
            Type1 = primType1;
        }
        
        public int Type0 { get; }
        public int Type1 { get; }

        public abstract void CollDetect(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor);
    }
}