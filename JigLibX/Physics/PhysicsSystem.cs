using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using JigLibX.Collision;
using JigLibX.Math;
using System.Collections.ObjectModel;
using System.Diagnostics;

namespace JigLibX.Physics
{
    public class PhysicsSystem
    {
        public enum Solver
        {
            Fast,

            Normal,

            Combined,

            Accumulated
        }

        private class Contact
        {
            public struct BodyPair
            {
                public Body BodyA;

                public Body BodyB;

                public Vector3 RA;

                public BodyPair(Body bodyA, Body bodyB, ref Vector3 rA, ref Vector3 rB)
                {
                    var skinId = -1;
                    if (bodyB != null) skinId = bodyB.ID;

                    if (bodyA.ID > skinId)
                    {
                        BodyA = bodyA;
                        BodyB = bodyB;
                        RA = rA;
                    }
                    else
                    {
                        BodyA = bodyB;
                        BodyB = bodyA;
                        RA = rB;
                    }
                }

                public BodyPair(Body bodyA, Body bodyB)
                {
                    var skinId = -1;
                    if (bodyB != null) skinId = bodyB.ID;

                    if (bodyA.ID > skinId)
                    {
                        BodyA = bodyA;
                        BodyB = bodyB;
                    }
                    else
                    {
                        BodyA = bodyB;
                        BodyB = bodyA;
                    }

                    RA = Vector3.Zero;
                }
            }

            public struct CachedImpulse
            {
                public float NormalImpulse;

                public float NormalImpulseAux;

                public Vector3 FrictionImpulse;

                public CachedImpulse(float normalImpulse, float normalImpulseAux, ref Vector3 frictionImpulse)
                {
                    NormalImpulse = normalImpulse;
                    NormalImpulseAux = normalImpulseAux;
                    FrictionImpulse = frictionImpulse;
                }
            }

            public BodyPair Pair;

            public CachedImpulse Impulse;
        }

        private List<Body> bodies = new List<Body>();
        private List<Body> activeBodies = new List<Body>();
        private List<Controller> controllers = new List<Controller>();
        private List<Constraint> constraints = new List<Constraint>();

        private List<Contact> catchedContacts = new List<Contact>();

        public List<CollisionIsland> islands = new List<CollisionIsland>();

        private delegate void PreProcessCollisionFn(CollisionInfo collision, float dt);

        private delegate bool ProcessCollisionFn(CollisionInfo collision, float dt, bool firstContact);

        private PreProcessCollisionFn preProcessContactFn;
        private ProcessCollisionFn processContactFn;
        private PreProcessCollisionFn preProcessCollisionFn;
        private ProcessCollisionFn processCollisionFn;

        private const float maxVelMag = 0.5f;
        private const float maxShockVelMag = 0.05f;
        private const float minVelForProcessing = 0.001f;
        private const float penetrationShockRelaxtionTimestep = 10.0f;

        private Solver solverType = Solver.Normal;

        private Vector3 gravity;

        public PhysicsSystem()
        {
            CurrentPhysicsSystem = this;
            Gravity = -10.0f * Vector3.Up;

            SetCollisionFns();
        }

        public ReadOnlyCollection<Body> Bodies => bodies.AsReadOnly();

        public ReadOnlyCollection<Constraint> Constraints => constraints.AsReadOnly();

        public ReadOnlyCollection<Controller> Controllers => controllers.AsReadOnly();

        public void AddConstraint(Constraint constraint)
        {
            if (!constraints.Contains(constraint))
                constraints.Add(constraint);
            else
                Debug.WriteLine("Warning: tried to add constraint to physics but it's already registered");
        }

        public bool RemoveConstraint(Constraint constraint)
        {
            if (!constraints.Contains(constraint)) return false;
            constraints.Remove(constraint);
            return true;
        }

        public void AddController(Controller controller)
        {
            if (!controllers.Contains(controller))
                controllers.Add(controller);
            else
                Debug.WriteLine("Warning: tried to add controller to physics but it's already registered");
        }

        public bool RemoveController(Controller controller)
        {
            if (!controllers.Contains(controller)) return false;
            controllers.Remove(controller);
            return true;
        }

        public void AddBody(Body body)
        {
            if (!bodies.Contains(body))
                bodies.Add(body);
            else
                Debug.WriteLine("Warning: tried to add body to physics but it's already registered");

            if (CollisionSystem != null && body.CollisionSkin != null)
                CollisionSystem.AddCollisionSkin(body.CollisionSkin);
        }

        public bool RemoveBody(Body body)
        {
            if (CollisionSystem != null && body.CollisionSkin != null)
                CollisionSystem.RemoveCollisionSkin(body.CollisionSkin);

            if (!bodies.Contains(body)) return false;

            bodies.Remove(body);
            return true;
        }

        private void FindAllActiveBodies()
        {
            activeBodies.Clear();
            var numBodies = bodies.Count;
            for (var i = 0; i < numBodies; ++i)
                if (bodies[i].IsActive)
                    activeBodies.Add(bodies[i]);
        }

        private int LessBodyX(Body body0, Body body1)
        {
            return body1.Position.X > body0.Position.X ? -1 : 1;
        }

        private int LessBodyY(Body body0, Body body1)
        {
            return body1.Position.Y > body0.Position.Y ? -1 : 1;
        }

        private int LessBodyZ(Body body0, Body body1)
        {
            return body1.Position.Z > body0.Position.Z ? -1 : 1;
        }

        private void DoShockStep(float dt)
        {
            var numBodies = bodies.Count;

            if (System.Math.Abs(gravity.X) > System.Math.Abs(gravity.Y) && System.Math.Abs(gravity.X) > System.Math.Abs(gravity.Z))
                bodies.Sort(LessBodyX);
            else if (System.Math.Abs(gravity.Y) > System.Math.Abs(gravity.Z) && System.Math.Abs(gravity.Y) > System.Math.Abs(gravity.X))
                bodies.Sort(LessBodyY);
            else if (System.Math.Abs(gravity.Z) > System.Math.Abs(gravity.X) && System.Math.Abs(gravity.Z) > System.Math.Abs(gravity.Y)) bodies.Sort(LessBodyZ);

            var gotOne = true;
            var loops = 0;

            while (gotOne)
            {
                gotOne = false;
                ++loops;
                for (var ibody = 0; ibody < numBodies; ++ibody)
                {
                    var body = bodies[ibody];
                    if (!body.Immovable && body.DoShockProcessing)
                    {
                        var skin = body.CollisionSkin;

                        if (skin != null)
                        {
                            var colls = skin.Collisions;
                            var numColls = colls.Count;

                            if (0 == numColls || !body.IsActive)
                            {
                                body.InternalSetImmovable();
                            }
                            else
                            {
                                var setImmovable = false;


                                for (var i = 0; i < numColls; ++i)
                                {
                                    var info = colls[i];


                                    if (info.SkinInfo.Skin0 == body.CollisionSkin && (info.SkinInfo.Skin1.Owner == null || info.SkinInfo.Skin1.Owner.Immovable) || info.SkinInfo.Skin1 == body.CollisionSkin && (info.SkinInfo.Skin0.Owner == null || info.SkinInfo.Skin0.Owner.Immovable))
                                    {
                                        preProcessCollisionFn(info, dt);
                                        ProcessCollisionsForShock(info, dt);
                                        setImmovable = true;
                                    }
                                }

                                if (setImmovable)
                                {
                                    body.InternalSetImmovable();
                                    gotOne = true;
                                }
                            }
                        }
                        else
                        {
                            body.InternalSetImmovable();
                        }
                    }
                }
            }

            for (var i = 0; i < numBodies; ++i) bodies[i].InternalRestoreImmovable();
        }

        private void GetAllExternalForces(float dt)
        {
            var numBodies = bodies.Count;
            for (var i = 0; i < numBodies; ++i) bodies[i].AddExternalForces(dt);

            var numControllers = controllers.Count;
            for (var i = 0; i < numControllers; ++i) controllers[i].UpdateController(dt);
        }

        private void UpdateAllVelocities(float dt)
        {
            var numBodies = bodies.Count;
            for (var i = 0; i < numBodies; ++i)
                if (bodies[i].IsActive || bodies[i].VelChanged)
                    bodies[i].UpdateVelocity(dt);
        }

        private void UpdateAllPositions(float dt)
        {
            var numBodies = activeBodies.Count;
            for (var i = 0; i < numBodies; ++i) activeBodies[i].UpdatePositionWithAux(dt);
        }

        private void CopyAllCurrentStatesToOld()
        {
            var numBodies = bodies.Count;
            for (var i = 0; i < numBodies; ++i)
                if (bodies[i].IsActive || bodies[i].VelChanged)
                    bodies[i].CopyCurrentStateToOld();
        }

        private Random rand = new Random();

        private void DetectAllCollisions(float dt)
        {
            if (CollisionSystem == null) return;

            var numBodies = bodies.Count;
            var numColls = Collisions.Count;
            var numActiveBodies = activeBodies.Count;

            int i;

            for (i = 0; i < numActiveBodies; ++i) activeBodies[i].StoreState();

            UpdateAllVelocities(dt);
            UpdateAllPositions(dt);

            for (i = 0; i < numColls; ++i) CollisionInfo.FreeCollisionInfo(Collisions[i]);

            Collisions.Clear();

            for (i = 0; i < numBodies; ++i)
                if (bodies[i].CollisionSkin != null)
                    bodies[i].CollisionSkin.Collisions.Clear();

            var collisionFunctor = new BasicCollisionFunctor(Collisions);
            CollisionSystem.DetectAllCollisions(activeBodies, collisionFunctor, null, CollisionTollerance);

            int index;
            CollisionInfo collInfo;
            for (i = 1; i < Collisions.Count; i++)
            {
                index = rand.Next(i + 1);
                collInfo = Collisions[i];
                Collisions[i] = Collisions[index];
                Collisions[index] = collInfo;
            }

            for (i = 0; i < numActiveBodies; ++i) activeBodies[i].RestoreState();
        }

        private void NotifyAllPostPhysics(float dt)
        {
            var numBodies = bodies.Count;
            for (var i = 0; i < numBodies; ++i) bodies[i].PostPhysics(dt);
        }

        private void LimitAllVelocities()
        {
            var numActiveBodies = activeBodies.Count;
            for (var i = 0; i < numActiveBodies; ++i)
            {
                activeBodies[i].LimitVel();
                activeBodies[i].LimitAngVel();
            }
        }

        private bool ProcessCollision(CollisionInfo collision, float dt, bool firstContact)
        {
            collision.Satisfied = true;

            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;

            var N = collision.DirToBody0;

            var gotOne = false;

            for (var pos = 0; pos < collision.NumCollPts; ++pos)
            {
                var ptInfo = collision.PointInfo[pos];
                float normalVel;

                if (body1 != null)
                {
                    body0.GetVelocity(ref ptInfo.Info.R0, out var v0);
                    body1.GetVelocity(ref ptInfo.Info.R1, out var v1);
                    Vector3.Subtract(ref v0, ref v1, out v0);
                    Vector3.Dot(ref v0, ref N, out normalVel);
                }
                else
                {
                    body0.GetVelocity(ref ptInfo.Info.R0, out var v0);
                    Vector3.Dot(ref v0, ref N, out normalVel);
                }

                if (normalVel > ptInfo.MinSeparationVel) continue;

                var finalNormalVel = -collision.MatPairProperties.Restitution * normalVel;

                if (finalNormalVel < minVelForProcessing)


                    finalNormalVel = ptInfo.MinSeparationVel;

                var deltaVel = finalNormalVel - normalVel;

                if (deltaVel <= minVelForProcessing) continue;

                if (ptInfo.Denominator < JiggleMath.Epsilon) ptInfo.Denominator = JiggleMath.Epsilon;

                var normalImpulse = deltaVel / ptInfo.Denominator;


                gotOne = true;
                Vector3.Multiply(ref N, normalImpulse, out var impulse);

                body0.ApplyBodyWorldImpulse(ref impulse, ref ptInfo.Info.R0);

                body1?.ApplyNegativeBodyWorldImpulse(ref impulse, ref ptInfo.Info.R1);


                body0.GetVelocity(ref ptInfo.Info.R0, out var vrNew);

                if (body1 != null)
                {
                    body1.GetVelocity(ref ptInfo.Info.R1, out var v1);
                    Vector3.Subtract(ref vrNew, ref v1, out vrNew);
                }


                Vector3.Dot(ref vrNew, ref N, out var f1);
                Vector3.Multiply(ref N, f1, out var tangentVel);
                Vector3.Subtract(ref vrNew, ref tangentVel, out tangentVel);

                var tangentSpeed = tangentVel.Length();

                if (tangentSpeed > minVelForProcessing)
                {
                    var T = -tangentVel / tangentSpeed;


                    var denominator = 0.0f;
                    if (!body0.Immovable)
                    {
                        var num0 = ptInfo.Info.R0.Y * T.Z - ptInfo.Info.R0.Z * T.Y;
                        var num1 = ptInfo.Info.R0.Z * T.X - ptInfo.Info.R0.X * T.Z;
                        var num2 = ptInfo.Info.R0.X * T.Y - ptInfo.Info.R0.Y * T.X;

                        var num3 = num0 * body0.worldInvInertia.M11 + num1 * body0.worldInvInertia.M21 + num2 * body0.worldInvInertia.M31;
                        var num4 = num0 * body0.worldInvInertia.M12 + num1 * body0.worldInvInertia.M22 + num2 * body0.worldInvInertia.M32;
                        var num5 = num0 * body0.worldInvInertia.M13 + num1 * body0.worldInvInertia.M23 + num2 * body0.worldInvInertia.M33;

                        num0 = num4 * ptInfo.Info.R0.Z - num5 * ptInfo.Info.R0.Y;
                        num1 = num5 * ptInfo.Info.R0.X - num3 * ptInfo.Info.R0.Z;
                        num2 = num3 * ptInfo.Info.R0.Y - num4 * ptInfo.Info.R0.X;

                        denominator = body0.InverseMass + (num0 * T.X + num1 * T.Y + num2 * T.Z);
                    }

                    if (body1 != null && !body1.Immovable)
                    {
                        var num0 = ptInfo.Info.R1.Y * T.Z - ptInfo.Info.R1.Z * T.Y;
                        var num1 = ptInfo.Info.R1.Z * T.X - ptInfo.Info.R1.X * T.Z;
                        var num2 = ptInfo.Info.R1.X * T.Y - ptInfo.Info.R1.Y * T.X;

                        var num3 = num0 * body1.worldInvInertia.M11 + num1 * body1.worldInvInertia.M21 + num2 * body1.worldInvInertia.M31;
                        var num4 = num0 * body1.worldInvInertia.M12 + num1 * body1.worldInvInertia.M22 + num2 * body1.worldInvInertia.M32;
                        var num5 = num0 * body1.worldInvInertia.M13 + num1 * body1.worldInvInertia.M23 + num2 * body1.worldInvInertia.M33;

                        num0 = num4 * ptInfo.Info.R1.Z - num5 * ptInfo.Info.R1.Y;
                        num1 = num5 * ptInfo.Info.R1.X - num3 * ptInfo.Info.R1.Z;
                        num2 = num3 * ptInfo.Info.R1.Y - num4 * ptInfo.Info.R1.X;

                        denominator += body1.InverseMass + (num0 * T.X + num1 * T.Y + num2 * T.Z);
                    }

                    if (denominator > JiggleMath.Epsilon)
                    {
                        var impulseToReserve = tangentSpeed / denominator;

                        var impulseFromNormalImpulse = collision.MatPairProperties.StaticFriction * normalImpulse;
                        float frictionImpulse;

                        if (impulseToReserve < impulseFromNormalImpulse)
                            frictionImpulse = impulseToReserve;
                        else
                            frictionImpulse = collision.MatPairProperties.DynamicFriction * normalImpulse;

                        T *= frictionImpulse;
                        body0.ApplyBodyWorldImpulse(ref T, ref ptInfo.Info.R0);
                        body1?.ApplyNegativeBodyWorldImpulse(ref T, ref ptInfo.Info.R1);
                    }
                }
            }

            if (gotOne)
            {
                body0.SetConstraintsAndCollisionsUnsatisfied();
                body1?.SetConstraintsAndCollisionsUnsatisfied();
            }

            return gotOne;
        }

        private bool ProcessCollisionFast(CollisionInfo collision, float dt, bool firstContact)
        {
            collision.Satisfied = true;

            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;

            var N = collision.DirToBody0;

            var gotOne = false;
            for (var pos = collision.NumCollPts; pos-- != 0;)
            {
                var ptInfo = collision.PointInfo[pos];

                float normalVel;
                if (body1 != null)
                {
                    body0.GetVelocity(ref ptInfo.Info.R0, out var v0);
                    body1.GetVelocity(ref ptInfo.Info.R1, out var v1);
                    Vector3.Subtract(ref v0, ref v1, out v0);
                    normalVel = Vector3.Dot(v0, N);
                }
                else
                {
                    body0.GetVelocity(ref ptInfo.Info.R0, out var v0);
                    normalVel = Vector3.Dot(v0, N);
                }

                if (normalVel > ptInfo.MinSeparationVel) continue;

                var finalNormalVel = -collision.MatPairProperties.Restitution * normalVel;

                if (finalNormalVel < minVelForProcessing)


                    finalNormalVel = ptInfo.MinSeparationVel;

                var deltaVel = finalNormalVel - normalVel;

                if (deltaVel < minVelForProcessing) continue;

                var normalImpulse = deltaVel / ptInfo.Denominator;


                gotOne = true;
                var impulse = normalImpulse * N;

                body0.ApplyBodyWorldImpulse(ref impulse, ref ptInfo.Info.R0);
                body1?.ApplyNegativeBodyWorldImpulse(ref impulse, ref ptInfo.Info.R1);


                var vrNew = body0.GetVelocity(ptInfo.Info.R0);
                if (body1 != null) vrNew -= body1.GetVelocity(ptInfo.Info.R1);

                var tangentVel = vrNew - Vector3.Dot(vrNew, N) * N;
                var tangentSpeed = tangentVel.Length();

                if (tangentSpeed > minVelForProcessing)
                {
                    var T = -tangentVel / tangentSpeed;


                    var denominator = 0.0f;
                    if (!body0.Immovable)
                    {
                        Vector3.Cross(ref ptInfo.Info.R0, ref T, out var v1);
                        Vector3.TransformNormal(ref v1, ref body0.worldInvInertia, out v1);
                        Vector3.Cross(ref v1, ref ptInfo.Info.R0, out v1);
                        Vector3.Dot(ref T, ref v1, out var f2);
                        denominator = body0.InverseMass + f2;
                    }

                    if (body1 != null && !body1.Immovable)
                    {
                        Vector3.Cross(ref ptInfo.Info.R1, ref T, out var v1);
                        Vector3.TransformNormal(ref v1, ref body1.worldInvInertia, out v1);
                        Vector3.Cross(ref v1, ref ptInfo.Info.R1, out v1);
                        Vector3.Dot(ref T, ref v1, out var f2);
                        denominator += body1.InverseMass + f2;
                    }

                    if (denominator > JiggleMath.Epsilon)
                    {
                        var impulseToReverse = tangentSpeed / denominator;
                        T *= impulseToReverse;
                        body0.ApplyBodyWorldImpulse(ref T, ref ptInfo.Info.R0);
                        body1?.ApplyNegativeBodyWorldImpulse(ref T, ref ptInfo.Info.R1);
                    }
                }
            }

            if (gotOne)
            {
                body0.SetConstraintsAndCollisionsUnsatisfied();
                body1?.SetConstraintsAndCollisionsUnsatisfied();
            }

            return gotOne;
        }

        private bool ProcessCollisionAccumulated(CollisionInfo collision, float dt, bool firstContact)
        {
            collision.Satisfied = true;

            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;

            var N = collision.DirToBody0;

            var gotOne = false;

            for (var pos = collision.NumCollPts; pos-- != 0;)
            {
                var ptInfo = collision.PointInfo[pos];
                float normalImpulse;


                {
                    float normalVel;
                    if (body1 != null)
                    {
                        body0.GetVelocity(ref ptInfo.Info.R0, out var v0);
                        body1.GetVelocity(ref ptInfo.Info.R1, out var v1);
                        Vector3.Subtract(ref v0, ref v1, out v0);
                        Vector3.Dot(ref v0, ref N, out normalVel);
                    }
                    else
                    {
                        body0.GetVelocity(ref ptInfo.Info.R0, out var v0);

                        Vector3.Dot(ref v0, ref N, out normalVel);
                    }


                    var deltaVel = -normalVel;


                    if (ptInfo.MinSeparationVel < 0.0f) deltaVel += ptInfo.MinSeparationVel;

                    if (System.Math.Abs(deltaVel) > minVelForProcessing)
                    {
                        normalImpulse = deltaVel / ptInfo.Denominator;

                        var origAccumulatedNormalImpulse = ptInfo.AccumulatedNormalImpulse;
                        ptInfo.AccumulatedNormalImpulse = MathHelper.Max(ptInfo.AccumulatedNormalImpulse + normalImpulse, 0.0f);
                        var actualImpulse = ptInfo.AccumulatedNormalImpulse - origAccumulatedNormalImpulse;


                        Vector3.Multiply(ref N, actualImpulse, out var impulse);

                        body0.ApplyBodyWorldImpulse(ref impulse, ref ptInfo.Info.R0);

                        body1?.ApplyNegativeBodyWorldImpulse(ref impulse, ref ptInfo.Info.R1);


                        gotOne = true;
                    }
                }


                var doCorrection = true;

                if (doCorrection)
                {
                    float normalVel;
                    if (body1 != null)
                    {
                        body0.GetVelocityAux(ref ptInfo.Info.R0, out var v0);
                        body1.GetVelocityAux(ref ptInfo.Info.R1, out var v1);
                        Vector3.Subtract(ref v0, ref v1, out v0);
                        Vector3.Dot(ref v0, ref N, out normalVel);
                    }
                    else
                    {
                        body0.GetVelocityAux(ref ptInfo.Info.R0, out var v0);

                        Vector3.Dot(ref v0, ref N, out normalVel);
                    }

                    var deltaVel = -normalVel;

                    if (ptInfo.MinSeparationVel > 0.0f) deltaVel += ptInfo.MinSeparationVel;

                    if (System.Math.Abs(deltaVel) > minVelForProcessing)
                    {
                        normalImpulse = deltaVel / ptInfo.Denominator;

                        var origAccumulatedNormalImpulse = ptInfo.AccumulatedNormalImpulseAux;
                        ptInfo.AccumulatedNormalImpulseAux = System.Math.Max(ptInfo.AccumulatedNormalImpulseAux + normalImpulse, 0.0f);
                        var actualImpulse = ptInfo.AccumulatedNormalImpulseAux - origAccumulatedNormalImpulse;


                        Vector3.Multiply(ref N, actualImpulse, out var impulse);

                        body0.ApplyBodyWorldImpulseAux(ref impulse, ref ptInfo.Info.R0);
                        body1?.ApplyNegativeBodyWorldImpulseAux(ref impulse, ref ptInfo.Info.R1);

                        gotOne = true;
                    }
                }


                if (ptInfo.AccumulatedNormalImpulse > 0.0f)
                {
                    var vrNew = body0.GetVelocity(ptInfo.Info.R0);
                    if (body1 != null)
                    {
                        body1.GetVelocity(ref ptInfo.Info.R1, out var pt1Vel);
                        Vector3.Subtract(ref vrNew, ref pt1Vel, out vrNew);
                    }


                    Vector3.Dot(ref vrNew, ref N, out var f1);
                    Vector3.Multiply(ref N, f1, out var tangentVel);
                    Vector3.Subtract(ref vrNew, ref tangentVel, out tangentVel);

                    var tangentSpeed = tangentVel.Length();

                    if (tangentSpeed > minVelForProcessing)
                    {
                        Vector3.Divide(ref tangentVel, -tangentSpeed, out var T);


                        var denominator = 0.0f;
                        if (!body0.Immovable)
                        {
                            var num0 = ptInfo.Info.R0.Y * T.Z - ptInfo.Info.R0.Z * T.Y;
                            var num1 = ptInfo.Info.R0.Z * T.X - ptInfo.Info.R0.X * T.Z;
                            var num2 = ptInfo.Info.R0.X * T.Y - ptInfo.Info.R0.Y * T.X;

                            var num3 = num0 * body0.worldInvInertia.M11 + num1 * body0.worldInvInertia.M21 + num2 * body0.worldInvInertia.M31;
                            var num4 = num0 * body0.worldInvInertia.M12 + num1 * body0.worldInvInertia.M22 + num2 * body0.worldInvInertia.M32;
                            var num5 = num0 * body0.worldInvInertia.M13 + num1 * body0.worldInvInertia.M23 + num2 * body0.worldInvInertia.M33;

                            num0 = num4 * ptInfo.Info.R0.Z - num5 * ptInfo.Info.R0.Y;
                            num1 = num5 * ptInfo.Info.R0.X - num3 * ptInfo.Info.R0.Z;
                            num2 = num3 * ptInfo.Info.R0.Y - num4 * ptInfo.Info.R0.X;

                            denominator = body0.InverseMass + (num0 * T.X + num1 * T.Y + num2 * T.Z);
                        }

                        if (body1 != null && !body1.Immovable)
                        {
                            var num0 = ptInfo.Info.R1.Y * T.Z - ptInfo.Info.R1.Z * T.Y;
                            var num1 = ptInfo.Info.R1.Z * T.X - ptInfo.Info.R1.X * T.Z;
                            var num2 = ptInfo.Info.R1.X * T.Y - ptInfo.Info.R1.Y * T.X;

                            var num3 = num0 * body1.worldInvInertia.M11 + num1 * body1.worldInvInertia.M21 + num2 * body1.worldInvInertia.M31;
                            var num4 = num0 * body1.worldInvInertia.M12 + num1 * body1.worldInvInertia.M22 + num2 * body1.worldInvInertia.M32;
                            var num5 = num0 * body1.worldInvInertia.M13 + num1 * body1.worldInvInertia.M23 + num2 * body1.worldInvInertia.M33;

                            num0 = num4 * ptInfo.Info.R1.Z - num5 * ptInfo.Info.R1.Y;
                            num1 = num5 * ptInfo.Info.R1.X - num3 * ptInfo.Info.R1.Z;
                            num2 = num3 * ptInfo.Info.R1.Y - num4 * ptInfo.Info.R1.X;

                            denominator += body1.InverseMass + (num0 * T.X + num1 * T.Y + num2 * T.Z);
                        }

                        if (denominator > JiggleMath.Epsilon)
                        {
                            var impulseToReverse = tangentSpeed / denominator;

                            Vector3.Multiply(ref T, impulseToReverse, out var frictionImpulseVec);

                            var origAccumulatedFrictionImpulse = ptInfo.AccumulatedFrictionImpulse;


                            Vector3.Add(ref ptInfo.AccumulatedFrictionImpulse, ref frictionImpulseVec, out ptInfo.AccumulatedFrictionImpulse);

                            var AFIMag = ptInfo.AccumulatedFrictionImpulse.Length();
                            var maxAllowedAFIMAg = collision.MatPairProperties.StaticFriction * ptInfo.AccumulatedNormalImpulse;

                            if (AFIMag > JiggleMath.Epsilon && AFIMag > maxAllowedAFIMAg)

                                Vector3.Multiply(ref ptInfo.AccumulatedFrictionImpulse, maxAllowedAFIMAg / AFIMag, out ptInfo.AccumulatedFrictionImpulse);


                            Vector3.Subtract(ref ptInfo.AccumulatedFrictionImpulse, ref origAccumulatedFrictionImpulse, out var actualFrictionImpulse);

                            body0.ApplyBodyWorldImpulse(ref actualFrictionImpulse, ref ptInfo.Info.R0);
                            body1?.ApplyNegativeBodyWorldImpulse(ref actualFrictionImpulse, ref ptInfo.Info.R1);
                        }
                    }
                }
            }

            if (gotOne)
            {
                body0.SetConstraintsAndCollisionsUnsatisfied();
                body1?.SetConstraintsAndCollisionsUnsatisfied();
            }

            return gotOne;
        }

        private unsafe bool ProcessCollisionCombined(CollisionInfo collision, float dt, bool firstContact)
        {
            collision.Satisfied = true;

            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;

            var N = collision.DirToBody0;


            var totalImpulse = 0.0f;
            int pos;

            var avPos = Vector3.Zero;
            var avMinSeparationVel = 0.0f;


            var impulses = stackalloc float[CollisionInfo.MaxCollisionPoints];

            for (pos = collision.NumCollPts; pos-- != 0;)
            {
                var ptInfo = collision.PointInfo[pos];
                impulses[pos] = 0.0f;

                float normalVel;
                if (body1 != null)
                    normalVel = Vector3.Dot(body0.GetVelocity(ptInfo.Info.R0) - body1.GetVelocity(ptInfo.Info.R1), N);
                else
                    normalVel = Vector3.Dot(body0.GetVelocity(ptInfo.Info.R0), N);

                if (normalVel > ptInfo.MinSeparationVel) continue;

                var finalNormalVel = -collision.MatPairProperties.Restitution * normalVel;

                if (finalNormalVel < minVelForProcessing)


                    finalNormalVel = ptInfo.MinSeparationVel;

                var deltaVel = finalNormalVel - normalVel;
                if (deltaVel < minVelForProcessing) continue;

                var normalImpulse = deltaVel / ptInfo.Denominator;

                impulses[pos] = normalImpulse;
                totalImpulse += normalImpulse;

                avPos += normalImpulse * ptInfo.Position;
                avMinSeparationVel += ptInfo.MinSeparationVel * normalImpulse;
            }

            if (totalImpulse <= JiggleMath.Epsilon) return false;

            var scale = 1.0f / totalImpulse;


            for (pos = collision.NumCollPts; pos-- != 0;)
                if (impulses[pos] > JiggleMath.Epsilon)
                {
                    var ptInfo = collision.PointInfo[pos];
                    var sc = impulses[pos] * scale;

                    Vector3.Multiply(ref N, impulses[pos] * sc, out var impulse);

                    body0.ApplyBodyWorldImpulse(ref impulse, ref ptInfo.Info.R0);

                    body1?.ApplyNegativeBodyWorldImpulse(ref impulse, ref ptInfo.Info.R1);
                }

            Vector3.Multiply(ref avPos, scale, out avPos);
            avMinSeparationVel *= scale;


            Vector3 R0, R1 = Vector3.Zero;
            R0 = avPos - body0.Position;
            var Vr = body0.GetVelocity(R0);
            if (body1 != null)
            {
                R1 = avPos - body1.Position;
                Vr -= body1.GetVelocity(R1);
            }

            var normalVel2 = Vector3.Dot(Vr, N);

            var normalImpulse2 = 0.0f;

            if (normalVel2 < avMinSeparationVel)
            {
                var finalNormalVel = -collision.MatPairProperties.Restitution * normalVel2;

                if (finalNormalVel < minVelForProcessing)


                    finalNormalVel = avMinSeparationVel;

                var deltaVel = finalNormalVel - normalVel2;

                if (deltaVel > minVelForProcessing)
                {
                    var denominator = 0.0f;
                    if (!body0.Immovable)
                        denominator = body0.InverseMass + Vector3.Dot(N, Vector3.Cross(Vector3.TransformNormal(Vector3.Cross(R0, N), body0.WorldInvInertia), R0));
                    if (body1 != null && !body1.Immovable)
                        denominator += body1.InverseMass + Vector3.Dot(N, Vector3.Cross(Vector3.TransformNormal(Vector3.Cross(R1, N), body1.WorldInvInertia), R1));
                    if (denominator < JiggleMath.Epsilon) denominator = JiggleMath.Epsilon;

                    normalImpulse2 = deltaVel / denominator;
                    var impulse = normalImpulse2 * N;

                    body0.ApplyWorldImpulse(impulse, avPos);
                    body1?.ApplyNegativeWorldImpulse(impulse, avPos);
                }
            }


            for (pos = collision.NumCollPts; pos-- != 0;)
            {
                var ptInfo = collision.PointInfo[pos];

                var vrNew = body1 != null ? body0.GetVelocity(ptInfo.Info.R0) - body1.GetVelocity(ptInfo.Info.R1) : body0.GetVelocity(ptInfo.Info.R0);

                var T = vrNew - Vector3.Dot(vrNew, N) * N;
                var tangentSpeed = T.Length();
                if (tangentSpeed > minVelForProcessing)
                {
                    T /= -tangentSpeed;

                    var sc = impulses[pos] * scale;
                    var ptNormalImpulse = sc * (normalImpulse2 + impulses[pos]);


                    var denominator = 0.0f;


                    if (!body0.Immovable)
                        denominator = body0.InverseMass + Vector3.Dot(T, Vector3.Cross(Vector3.TransformNormal(Vector3.Cross(ptInfo.Info.R0, T), body0.WorldInvInertia), ptInfo.Info.R0));

                    if (body1 != null && !body1.Immovable)
                        denominator += body1.InverseMass + Vector3.Dot(T, Vector3.Cross(Vector3.TransformNormal(Vector3.Cross(ptInfo.Info.R1, T), body1.WorldInvInertia), ptInfo.Info.R1));

                    if (denominator > JiggleMath.Epsilon)
                    {
                        var impulseToReverse = tangentSpeed / denominator;
                        var impulseFromNormalImpulse = collision.MatPairProperties.StaticFriction * ptNormalImpulse;
                        float frictionImpulse;

                        if (impulseToReverse < impulseFromNormalImpulse)
                            frictionImpulse = impulseToReverse;
                        else
                            frictionImpulse = collision.MatPairProperties.DynamicFriction * ptNormalImpulse;

                        T *= frictionImpulse;
                        body0.ApplyBodyWorldImpulse(T, ptInfo.Info.R0);
                        body1?.ApplyNegativeBodyWorldImpulse(ref T, ref ptInfo.Info.R1);
                    }
                }
            }

            body0.SetConstraintsAndCollisionsUnsatisfied();
            body1?.SetConstraintsAndCollisionsUnsatisfied();

            return true;
        }

        private bool ProcessCollisionsForShock(CollisionInfo collision, float dt)
        {
            collision.Satisfied = true;
            var N = collision.DirToBody0;

            N.X = N.Z = 0.0f;
            JiggleMath.NormalizeSafe(ref N);
            var iterations = 5;
            int pos;
            var timescale = penetrationShockRelaxtionTimestep * dt;
            for (pos = 0; pos < collision.NumCollPts; ++pos)
            {
                var ptInfo = collision.PointInfo[pos];
            }


            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;
            if (body0.Immovable) body0 = null;
            if (body1 != null && body1.Immovable) body1 = null;

            if (body0 == null && body1 == null) return false;

            for (var iteration = 0; iteration < iterations; ++iteration)
            for (pos = 0; pos < collision.NumCollPts; ++pos)
            {
                var ptInfo = collision.PointInfo[pos];
                var normalVel = 0.0f;
                if (body0 != null)
                    normalVel = Vector3.Dot(body0.GetVelocity(ptInfo.Info.R0), N) + Vector3.Dot(body0.GetVelocityAux(ptInfo.Info.R0), N);
                if (body1 != null)
                    normalVel -= Vector3.Dot(body1.GetVelocity(ptInfo.Info.R1), N) + Vector3.Dot(body1.GetVelocityAux(ptInfo.Info.R1), N);

                var finalNormalVel = (ptInfo.Info.InitialPenetration - AllowedPenetration) / timescale;

                if (finalNormalVel < 0.0f) continue;

                var impulse = (finalNormalVel - normalVel) / ptInfo.Denominator;

                var orig = ptInfo.AccumulatedNormalImpulseAux;
                ptInfo.AccumulatedNormalImpulseAux = System.Math.Max(ptInfo.AccumulatedNormalImpulseAux + impulse, 0.0f);
                var actualImpulse = (ptInfo.AccumulatedNormalImpulseAux - orig) * N;

                body0?.ApplyBodyWorldImpulseAux(ref actualImpulse, ref ptInfo.Info.R0);
                body1?.ApplyNegativeBodyWorldImpulseAux(ref actualImpulse, ref ptInfo.Info.R1);
            }

            body0?.SetConstraintsAndCollisionsUnsatisfied();
            body1?.SetConstraintsAndCollisionsUnsatisfied();

            return true;
        }

        private void PreProcessCollision(CollisionInfo collision, float dt)
        {
            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;


            collision.Satisfied = false;


            var N = collision.DirToBody0;
            var timescale = NumPenetrationRelaxtionTimesteps * dt;

            for (var pos = 0; pos < collision.NumCollPts; ++pos)
            {
                var ptInfo = collision.PointInfo[pos];


                if (body0.Immovable)
                {
                    ptInfo.Denominator = 0.0f;
                }
                else
                {
                    var num0 = ptInfo.Info.R0.Y * N.Z - ptInfo.Info.R0.Z * N.Y;
                    var num1 = ptInfo.Info.R0.Z * N.X - ptInfo.Info.R0.X * N.Z;
                    var num2 = ptInfo.Info.R0.X * N.Y - ptInfo.Info.R0.Y * N.X;

                    var num3 = num0 * body0.worldInvInertia.M11 + num1 * body0.worldInvInertia.M21 + num2 * body0.worldInvInertia.M31;
                    var num4 = num0 * body0.worldInvInertia.M12 + num1 * body0.worldInvInertia.M22 + num2 * body0.worldInvInertia.M32;
                    var num5 = num0 * body0.worldInvInertia.M13 + num1 * body0.worldInvInertia.M23 + num2 * body0.worldInvInertia.M33;

                    num0 = num4 * ptInfo.Info.R0.Z - num5 * ptInfo.Info.R0.Y;
                    num1 = num5 * ptInfo.Info.R0.X - num3 * ptInfo.Info.R0.Z;
                    num2 = num3 * ptInfo.Info.R0.Y - num4 * ptInfo.Info.R0.X;

                    ptInfo.Denominator = body0.InverseMass + (num0 * N.X + num1 * N.Y + num2 * N.Z);
                }

                if (body1 != null && !body1.Immovable)
                {
                    var num0 = ptInfo.Info.R1.Y * N.Z - ptInfo.Info.R1.Z * N.Y;
                    var num1 = ptInfo.Info.R1.Z * N.X - ptInfo.Info.R1.X * N.Z;
                    var num2 = ptInfo.Info.R1.X * N.Y - ptInfo.Info.R1.Y * N.X;

                    var num3 = num0 * body1.worldInvInertia.M11 + num1 * body1.worldInvInertia.M21 + num2 * body1.worldInvInertia.M31;
                    var num4 = num0 * body1.worldInvInertia.M12 + num1 * body1.worldInvInertia.M22 + num2 * body1.worldInvInertia.M32;
                    var num5 = num0 * body1.worldInvInertia.M13 + num1 * body1.worldInvInertia.M23 + num2 * body1.worldInvInertia.M33;

                    num0 = num4 * ptInfo.Info.R1.Z - num5 * ptInfo.Info.R1.Y;
                    num1 = num5 * ptInfo.Info.R1.X - num3 * ptInfo.Info.R1.Z;
                    num2 = num3 * ptInfo.Info.R1.Y - num4 * ptInfo.Info.R1.X;

                    ptInfo.Denominator += body1.InverseMass + (num0 * N.X + num1 * N.Y + num2 * N.Z);
                }

                if (ptInfo.Denominator < JiggleMath.Epsilon) ptInfo.Denominator = JiggleMath.Epsilon;


                Vector3.Add(ref body0.oldTransform.Position, ref ptInfo.Info.R0, out ptInfo.Position);


                if (ptInfo.Info.InitialPenetration > AllowedPenetration)
                {
                    ptInfo.MinSeparationVel = (ptInfo.Info.InitialPenetration - AllowedPenetration) / timescale;
                }
                else
                {
                    var approachScale = -0.1f * (ptInfo.Info.InitialPenetration - AllowedPenetration) / (JiggleMath.Epsilon + AllowedPenetration);
                    approachScale = MathHelper.Clamp(approachScale, JiggleMath.Epsilon, 1.0f);
                    ptInfo.MinSeparationVel = approachScale * (ptInfo.Info.InitialPenetration - AllowedPenetration) / MathHelper.Max(dt, JiggleMath.Epsilon);
                }

                if (ptInfo.MinSeparationVel > maxVelMag) ptInfo.MinSeparationVel = maxVelMag;
            }
        }

        private int MoreCollPtPenetration(CollPointInfo info1, CollPointInfo info2)
        {
            if (info1 == null && info2 == null) return 0;
            if (info1 == null) return 1;
            if (info2 == null) return -1;

            if (info1.Info.InitialPenetration == info2.Info.InitialPenetration) return 0;
            return info1.Info.InitialPenetration < info2.Info.InitialPenetration ? 1 : -1;
        }

        private void PreProcessCollisionFast(CollisionInfo collision, float dt)
        {
            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;


            collision.Satisfied = false;


            var N = collision.DirToBody0;
            var timescale = NumPenetrationRelaxtionTimesteps * dt;

            const int keep = 3;
            if (collision.NumCollPts > keep)
            {
                Array.Sort(collision.PointInfo, MoreCollPtPenetration);
                collision.NumCollPts = keep;
            }

            for (var pos = 0; pos < collision.NumCollPts; ++pos)
            {
                var ptInfo = collision.PointInfo[pos];


                if (body0.Immovable)
                {
                    ptInfo.Denominator = 0.0f;
                }
                else
                {
                    Vector3.Cross(ref ptInfo.Info.R0, ref N, out var cross);
                    Vector3.TransformNormal(ref cross, ref body0.worldInvInertia, out cross);
                    Vector3.Cross(ref cross, ref ptInfo.Info.R0, out cross);
                    Vector3.Dot(ref N, ref cross, out var res);
                    ptInfo.Denominator = body0.InverseMass + res;
                }

                if (body1 != null && !body1.Immovable)
                {
                    Vector3.Cross(ref ptInfo.Info.R1, ref N, out var cross);
                    Vector3.TransformNormal(ref cross, ref body1.worldInvInertia, out cross);
                    Vector3.Cross(ref cross, ref ptInfo.Info.R1, out cross);
                    Vector3.Dot(ref N, ref cross, out var res);
                    ptInfo.Denominator += body1.InverseMass + res;
                }

                if (ptInfo.Denominator < JiggleMath.Epsilon) ptInfo.Denominator = JiggleMath.Epsilon;


                Vector3.Add(ref body0.oldTransform.Position, ref ptInfo.Info.R0, out ptInfo.Position);


                if (ptInfo.Info.InitialPenetration > AllowedPenetration)
                {
                    ptInfo.MinSeparationVel = (ptInfo.Info.InitialPenetration - AllowedPenetration) / timescale;
                }
                else
                {
                    var approachScale = -0.1f * (ptInfo.Info.InitialPenetration - AllowedPenetration) / (JiggleMath.Epsilon + AllowedPenetration);
                    approachScale = MathHelper.Clamp(approachScale, JiggleMath.Epsilon, 1.0f);
                    ptInfo.MinSeparationVel = approachScale * (ptInfo.Info.InitialPenetration - AllowedPenetration) / MathHelper.Max(dt, JiggleMath.Epsilon);
                }

                if (ptInfo.MinSeparationVel > maxVelMag) ptInfo.MinSeparationVel = maxVelMag;
            }
        }

        private void PreProcessCollisionAccumulated(CollisionInfo collision, float dt)
        {
            var body0 = collision.SkinInfo.Skin0.Owner;
            var body1 = collision.SkinInfo.Skin1.Owner;


            collision.Satisfied = false;


            var N = collision.DirToBody0;
            var timescale = NumPenetrationRelaxtionTimesteps * dt;

            for (var pos = 0; pos < collision.NumCollPts; ++pos)
            {
                var ptInfo = collision.PointInfo[pos];


                if (body0.Immovable)
                {
                    ptInfo.Denominator = 0.0f;
                }
                else
                {
                    var num0 = ptInfo.Info.R0.Y * N.Z - ptInfo.Info.R0.Z * N.Y;
                    var num1 = ptInfo.Info.R0.Z * N.X - ptInfo.Info.R0.X * N.Z;
                    var num2 = ptInfo.Info.R0.X * N.Y - ptInfo.Info.R0.Y * N.X;

                    var num3 = num0 * body0.worldInvInertia.M11 + num1 * body0.worldInvInertia.M21 + num2 * body0.worldInvInertia.M31;
                    var num4 = num0 * body0.worldInvInertia.M12 + num1 * body0.worldInvInertia.M22 + num2 * body0.worldInvInertia.M32;
                    var num5 = num0 * body0.worldInvInertia.M13 + num1 * body0.worldInvInertia.M23 + num2 * body0.worldInvInertia.M33;

                    num0 = num4 * ptInfo.Info.R0.Z - num5 * ptInfo.Info.R0.Y;
                    num1 = num5 * ptInfo.Info.R0.X - num3 * ptInfo.Info.R0.Z;
                    num2 = num3 * ptInfo.Info.R0.Y - num4 * ptInfo.Info.R0.X;

                    ptInfo.Denominator = body0.InverseMass + (num0 * N.X + num1 * N.Y + num2 * N.Z);
                }

                if (body1 != null && !body1.Immovable)
                {
                    var num0 = ptInfo.Info.R1.Y * N.Z - ptInfo.Info.R1.Z * N.Y;
                    var num1 = ptInfo.Info.R1.Z * N.X - ptInfo.Info.R1.X * N.Z;
                    var num2 = ptInfo.Info.R1.X * N.Y - ptInfo.Info.R1.Y * N.X;

                    var num3 = num0 * body1.worldInvInertia.M11 + num1 * body1.worldInvInertia.M21 + num2 * body1.worldInvInertia.M31;
                    var num4 = num0 * body1.worldInvInertia.M12 + num1 * body1.worldInvInertia.M22 + num2 * body1.worldInvInertia.M32;
                    var num5 = num0 * body1.worldInvInertia.M13 + num1 * body1.worldInvInertia.M23 + num2 * body1.worldInvInertia.M33;

                    num0 = num4 * ptInfo.Info.R1.Z - num5 * ptInfo.Info.R1.Y;
                    num1 = num5 * ptInfo.Info.R1.X - num3 * ptInfo.Info.R1.Z;
                    num2 = num3 * ptInfo.Info.R1.Y - num4 * ptInfo.Info.R1.X;

                    ptInfo.Denominator += body1.InverseMass + (num0 * N.X + num1 * N.Y + num2 * N.Z);
                }


                if (ptInfo.Denominator < JiggleMath.Epsilon) ptInfo.Denominator = JiggleMath.Epsilon;


                Vector3.Add(ref body0.oldTransform.Position, ref ptInfo.Info.R0, out ptInfo.Position);


                if (ptInfo.Info.InitialPenetration > AllowedPenetration)
                {
                    ptInfo.MinSeparationVel = (ptInfo.Info.InitialPenetration - AllowedPenetration) / timescale;
                }
                else
                {
                    var approachScale = -0.1f * (ptInfo.Info.InitialPenetration - AllowedPenetration) / (JiggleMath.Epsilon + AllowedPenetration);
                    approachScale = MathHelper.Clamp(approachScale, JiggleMath.Epsilon, 1.0f);
                    ptInfo.MinSeparationVel = approachScale * (ptInfo.Info.InitialPenetration - AllowedPenetration) / MathHelper.Max(dt, JiggleMath.Epsilon);
                }

                ptInfo.AccumulatedNormalImpulse = 0.0f;
                ptInfo.AccumulatedNormalImpulseAux = 0.0f;
                ptInfo.AccumulatedFrictionImpulse = Vector3.Zero;


                var minDist = 0.2f;
                var bestDistSq = minDist * minDist;

                var bp = new Contact.BodyPair(body0, body1);
                var count = catchedContacts.Count;

                for (var i = 0; i < count; i++)
                {
                    if (!(bp.BodyA == catchedContacts[i].Pair.BodyA && bp.BodyB == catchedContacts[i].Pair.BodyB))
                        continue;


                    float distSq;
                    if (catchedContacts[i].Pair.BodyA == collision.SkinInfo.Skin0.Owner)
                    {
                        var num3 = catchedContacts[i].Pair.RA.X - ptInfo.Info.R0.X;
                        var num2 = catchedContacts[i].Pair.RA.Y - ptInfo.Info.R0.Y;
                        var num0 = catchedContacts[i].Pair.RA.Z - ptInfo.Info.R0.Z;
                        distSq = num3 * num3 + num2 * num2 + num0 * num0;
                    }

                    else
                    {
                        var num3 = catchedContacts[i].Pair.RA.X - ptInfo.Info.R1.X;
                        var num2 = catchedContacts[i].Pair.RA.Y - ptInfo.Info.R1.Y;
                        var num0 = catchedContacts[i].Pair.RA.Z - ptInfo.Info.R1.Z;
                        distSq = num3 * num3 + num2 * num2 + num0 * num0;
                    }

                    if (distSq < bestDistSq)
                    {
                        bestDistSq = distSq;

                        ptInfo.AccumulatedNormalImpulse = catchedContacts[i].Impulse.NormalImpulse;
                        ptInfo.AccumulatedNormalImpulseAux = catchedContacts[i].Impulse.NormalImpulseAux;
                        ptInfo.AccumulatedFrictionImpulse = catchedContacts[i].Impulse.FrictionImpulse;

                        if (catchedContacts[i].Pair.BodyA != collision.SkinInfo.Skin0.Owner)
                            ptInfo.AccumulatedFrictionImpulse *= -1;
                    }
                }


                if (ptInfo.AccumulatedNormalImpulse != 0.0f)
                {
                    Vector3.Multiply(ref N, ptInfo.AccumulatedNormalImpulse, out var impulse);

                    Vector3.Add(ref impulse, ref ptInfo.AccumulatedFrictionImpulse, out impulse);
                    body0.ApplyBodyWorldImpulse(ref impulse, ref ptInfo.Info.R0);
                    body1?.ApplyNegativeBodyWorldImpulse(ref impulse, ref ptInfo.Info.R1);
                }

                if (ptInfo.AccumulatedNormalImpulseAux != 0.0f)
                {
                    Vector3.Multiply(ref N, ptInfo.AccumulatedNormalImpulseAux, out var impulse);
                    body0.ApplyBodyWorldImpulseAux(ref impulse, ref ptInfo.Info.R0);
                    body1?.ApplyNegativeBodyWorldImpulseAux(ref impulse, ref ptInfo.Info.R1);
                }
            }
        }

        private void SetCollisionFns()
        {
            switch (solverType)
            {
                case Solver.Fast:
                    preProcessCollisionFn = preProcessContactFn = PreProcessCollisionFast;
                    processCollisionFn = processContactFn = ProcessCollisionFast;
                    break;
                case Solver.Normal:
                    preProcessCollisionFn = preProcessContactFn = PreProcessCollision;
                    processCollisionFn = processContactFn = ProcessCollision;
                    break;
                case Solver.Combined:
                    preProcessCollisionFn = preProcessContactFn = PreProcessCollision;
                    processCollisionFn = processContactFn = ProcessCollisionCombined;
                    break;
                case Solver.Accumulated:
                    preProcessCollisionFn = PreProcessCollision;
                    processCollisionFn = ProcessCollision;
                    preProcessContactFn = PreProcessCollisionAccumulated;
                    processContactFn = ProcessCollisionAccumulated;
                    break;
            }
        }

        private void HandleAllConstraints(float dt, int iter, bool forceInelastic)
        {
            var numConstraints = constraints.Count;
            var numCollisions = Collisions.Count;


            for (var i = 0; i < numConstraints; ++i) constraints[i].PreApply(dt);


            if (forceInelastic)
                for (var i = 0; i < numCollisions; ++i)
                {
                    preProcessContactFn(Collisions[i], dt);
                    Collisions[i].MatPairProperties.Restitution = 0.0f;
                    Collisions[i].Satisfied = false;
                }
            else

                for (var i = 0; i < numCollisions; ++i)
                    preProcessCollisionFn(Collisions[i], dt);


            var dir = true;

            for (var step = 0; step < iter; ++step)
            {
                var gotOne = true;


                dir = !dir;

                for (var i = dir ? 0 : numCollisions - 1; i >= 0 && i < numCollisions; i += dir ? 1 : -1)
                    if (!Collisions[i].Satisfied)
                    {
                        if (forceInelastic)
                            gotOne |= processContactFn(Collisions[i], dt, step == 0);
                        else
                            gotOne |= processCollisionFn(Collisions[i], dt, step == 0);
                    }

                for (var i = 0; i < numConstraints; ++i)
                    if (!constraints[i].Satisfied)
                        gotOne |= constraints[i].Apply(dt);

                if (!gotOne) break;
            }
        }

        public static PhysicsSystem CurrentPhysicsSystem { get; set; }

        public CollisionSystem CollisionSystem { get; set; }

        private void NotifyAllPrePhysics(float dt)
        {
            var numBodies = bodies.Count;
            for (var i = 0; i < numBodies; ++i) bodies[i].PrePhysics(dt);
        }

        private void BuildIslands()
        {
            foreach (var c in islands) freeCollisionIslands.Push(c);
            islands.Clear();

            foreach (var body in bodies) body.FoundIsland = false;

            foreach (var body in bodies)
                if (!body.FoundIsland)
                {
                    if (freeCollisionIslands.Count == 0) freeCollisionIslands.Push(new CollisionIsland());
                    var island = freeCollisionIslands.Pop();
                    island.Clear();
                    FindConnected(body, island);
                    islands.Add(island);
                }
        }

        private void DampAllActiveBodies()
        {
            var numBodies = activeBodies.Count;
            for (var i = 0; i < numBodies; ++i) activeBodies[i].DampForDeactivation();
        }

        private void FindConnected(Body body, CollisionIsland island)
        {
            if (body.FoundIsland || body.Immovable) return;

            body.FoundIsland = true;

            island.Add(body);

            foreach (var collision in body.CollisionSkin.Collisions)
                if (collision.SkinInfo.Skin1.Owner != null)
                {
                    if (collision.SkinInfo.Skin1.Owner == body)
                        FindConnected(collision.SkinInfo.Skin0.Owner, island);
                    else
                        FindConnected(collision.SkinInfo.Skin1.Owner, island);
                }
        }

        public void Integrate(float dt)
        {
            OldTime = TargetTime;
            TargetTime += dt;

            NotifyAllPrePhysics(dt);

            FindAllActiveBodies();

            CopyAllCurrentStatesToOld();

            GetAllExternalForces(dt);

            DetectAllCollisions(dt);

            HandleAllConstraints(dt, NumCollisionIterations, false);

            UpdateAllVelocities(dt);

            HandleAllConstraints(dt, NumContactIterations, true);


            if (IsShockStepEnabled) DoShockStep(dt);

            if (IsFreezingEnabled && Collisions.Count != 0)
            {
                BuildIslands();

                for (var i = 0; i < bodies.Count; i++) bodies[i].UpdateDeactivation(dt);

                for (var i = 0; i < islands.Count; i++)
                    if (islands[i].WantsDeactivation(dt))
                        islands[i].Deactivate();
                    else
                        islands[i].Activate();


                DampAllActiveBodies();
            }

            LimitAllVelocities();

            UpdateAllPositions(dt);

            NotifyAllPostPhysics(dt);

            if (solverType == Solver.Accumulated) UpdateContactCache();

            if (NullUpdate)
                for (var i = 0; i < activeBodies.Count; ++i)
                    activeBodies[i].RestoreState();
        }

        public float TargetTime { get; private set; }

        public float OldTime { get; private set; }

        public void ResetTime(float time)
        {
            TargetTime = OldTime = time;
        }

        public int NumCollisionIterations { set; get; } = 4;

        public int NumContactIterations { set; get; } = 12;

        public int NumPenetrationRelaxtionTimesteps { set; get; } = 3;

        public float AllowedPenetration { set; get; } = 0.01f;

        public bool IsShockStepEnabled { set; get; }

        public float CollisionTollerance { get; set; } = 0.05f;

        public Solver SolverType
        {
            get => solverType;
            set
            {
                solverType = value;
                SetCollisionFns();
            }
        }

        public bool NullUpdate { set; get; }

        public Vector3 Gravity
        {
            get => gravity;
            set
            {
                this.gravity = value;
                GravityMagnitude = value.Length();

                if (value.X == value.Y && value.Y == value.Z) MainGravityAxis = -1;

                MainGravityAxis = 0;

                if (System.Math.Abs(value.Y) > System.Math.Abs(value.X)) MainGravityAxis = 1;

                var gravity = new float[3] {value.X, value.Y, value.Z};

                if (System.Math.Abs(value.Z) > System.Math.Abs(gravity[MainGravityAxis])) MainGravityAxis = 2;
            }
        }

        public float GravityMagnitude { get; private set; }

        public int MainGravityAxis { get; private set; }

        public bool EnableFreezing
        {
            set
            {
                IsFreezingEnabled = value;

                if (!IsFreezingEnabled)
                {
                    var numBodies = bodies.Count;
                    for (var i = 0; i < numBodies; ++i) bodies[i].SetActive();
                }
            }
        }

        public bool IsFreezingEnabled { get; private set; }

        public List<CollisionInfo> Collisions { get; } = new List<CollisionInfo>();

        private static Stack<Contact> freeContacts = new Stack<Contact>(128);
        private static Stack<CollisionIsland> freeCollisionIslands = new Stack<CollisionIsland>(64);

        static PhysicsSystem()
        {
            for (var i = 0; i < 128; ++i) freeContacts.Push(new Contact());
            for (var i = 0; i < 64; ++i) freeCollisionIslands.Push(new CollisionIsland());
        }

        private void UpdateContactCache()
        {
            foreach (var c in catchedContacts) freeContacts.Push(c);
            catchedContacts.Clear();

            for (var i = Collisions.Count; i-- != 0;)
            {
                var collInfo = Collisions[i];
                for (var pos = 0; pos < collInfo.NumCollPts; ++pos)
                {
                    var ptInfo = collInfo.PointInfo[pos];

                    var skinId1 = -1;
                    if (collInfo.SkinInfo.Skin1.Owner != null) skinId1 = collInfo.SkinInfo.Skin1.Owner.ID;


                    var fricImpulse = collInfo.SkinInfo.Skin0.Owner.ID > skinId1 ? ptInfo.AccumulatedFrictionImpulse : -ptInfo.AccumulatedFrictionImpulse;

                    if (freeContacts.Count == 0) freeContacts.Push(new Contact());

                    var contact = freeContacts.Pop();
                    contact.Impulse = new Contact.CachedImpulse(ptInfo.AccumulatedNormalImpulse, ptInfo.AccumulatedNormalImpulseAux, ref fricImpulse);
                    contact.Pair = new Contact.BodyPair(collInfo.SkinInfo.Skin0.Owner, collInfo.SkinInfo.Skin1.Owner, ref ptInfo.Info.R0, ref ptInfo.Info.R1);

                    catchedContacts.Add(contact);
                }
            }
        }
    }
}