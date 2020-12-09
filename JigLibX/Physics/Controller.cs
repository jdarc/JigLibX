namespace JigLibX.Physics
{
    public abstract class Controller
    {
        public void EnableController()
        {
            if (PhysicsSystem.CurrentPhysicsSystem == null) return;
            if (IsControllerEnabled) return;

            IsControllerEnabled = true;

            PhysicsSystem.CurrentPhysicsSystem.AddController(this);
        }

        public void DisableController()
        {
            if (PhysicsSystem.CurrentPhysicsSystem == null) return;
            if (!IsControllerEnabled) return;

            IsControllerEnabled = false;
            PhysicsSystem.CurrentPhysicsSystem.RemoveController(this);
        }

        public bool IsControllerEnabled { get; private set; }

        public abstract void UpdateController(float dt);
    }
}