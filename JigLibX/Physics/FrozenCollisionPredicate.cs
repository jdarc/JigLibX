using JigLibX.Collision;

namespace JigLibX.Physics
{
    public class FrozenCollisionPredicate : CollisionSkinPredicate2
    {
        private Body body;

        public FrozenCollisionPredicate(Body body)
        {
            this.body = body;
        }

        public override bool ConsiderSkinPair(CollisionSkin skin0, CollisionSkin skin1)
        {
            if (skin0.Owner != null && skin0.Owner != body)
                if (!skin0.Owner.IsActive)
                    return true;

            if (skin1.Owner != null && skin1.Owner != body)
                if (!skin1.Owner.IsActive)
                    return true;

            return false;
        }
    }
}