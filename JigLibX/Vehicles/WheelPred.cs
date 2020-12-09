using JigLibX.Collision;

namespace JigLibX.Vehicles
{
    internal class WheelPred : CollisionSkinPredicate1
    {
        private readonly CollisionSkin _mSkin;

        public WheelPred(CollisionSkin carSkin) => _mSkin = carSkin;

        public override bool ConsiderSkin(CollisionSkin skin) => skin.ID != _mSkin.ID;
    }
}