using JigLibX.Collision;

namespace JigLibGame
{
    internal class ImmovableSkinPredicate : CollisionSkinPredicate1
    {
        public override bool ConsiderSkin(CollisionSkin skin0)
        {
            return skin0.Owner != null && !skin0.Owner.Immovable;
        }
    }
}