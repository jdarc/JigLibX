namespace JigLibX.Collision
{
    public struct CollDetectInfo
    {
        public int IndexPrim0;
        public int IndexPrim1;
        public CollisionSkin Skin0;
        public CollisionSkin Skin1;

        public CollDetectInfo(CollisionSkin skin0, CollisionSkin skin1, int indexPrim0, int indexPrim1)
        {
            IndexPrim0 = indexPrim0;
            IndexPrim1 = indexPrim1;
            Skin0 = skin0;
            Skin1 = skin1;
        }

        public static CollDetectInfo Empty => new CollDetectInfo(null, null, 0, 0);
    }
}