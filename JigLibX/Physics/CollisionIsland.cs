using System.Collections.Generic;

namespace JigLibX.Physics
{
    public class CollisionIsland : List<Body>
    {
        public CollisionIsland() : base(64)
        {
        }

        public static CollisionIsland Empty { get; } = new CollisionIsland();

        public bool WantsDeactivation(float dt)
        {
            for (var i = 0; i < Count; i++)
                if (this[i].GetShouldBeActive())
                    return false;
            return true;
        }

        public void Deactivate()
        {
            var count = Count;
            for (var i = 0; i < count; i++) this[i].SetInactive();
        }

        public void Activate()
        {
            var count = Count;
            for (var i = 0; i < count; i++) this[i].SetActive();
        }
    }
}