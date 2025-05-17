using System;
using EntityStates;
using HarmonyLib;
using Mono.Cecil.Cil;
using MonoMod.Cil;
using R2API.Utils;
using RoR2;
using RoR2.Projectile;
using UnityEngine;

namespace AdvancedPrediction.Prediction
{
    public static class PredictionUtils
    {
        internal const float zero = 0.00001f;

        /// <summary>
        /// aimRay must be on the stack before calling this!
        /// </summary>
        /// <param name="c"></param>
        /// <param name="type"></param>
        internal static void EmitPredictAimray<T>(this ILCursor c, string prefabName = "projectilePrefab")
        {
            //this.characterbody
            c.Emit(OpCodes.Ldarg_0);
            c.Emit(OpCodes.Call, AccessTools.PropertyGetter(typeof(EntityState), nameof(EntityState.characterBody)));

            // this.projectilePrefab
            // - or -
            // {TYPE}.projectilePrefab
            var fieldInfo = AccessTools.Field(typeof(T), prefabName);
            if (!fieldInfo.IsStatic) c.Emit(OpCodes.Ldarg_0);
            if (!fieldInfo.IsStatic) c.Emit(OpCodes.Ldfld, fieldInfo);
            else c.Emit(OpCodes.Ldsfld, fieldInfo);

            // Utils.PredictAimRay(aimRay, characterBody, projectilePrefab);
            c.Emit(OpCodes.Call, typeof(PredictionUtils).GetMethodCached(nameof(PredictAimray)));
        }

        public static Ray PredictAimray(Ray aimRay, CharacterBody body, GameObject projectilePrefab)
        {
            if (!AdvPredictionConfig.enablePrediction.Value || !body || !body.master || !projectilePrefab)
                return aimRay;

            var targetBody = GetAimTargetBody(body);
            if (!targetBody)
                return aimRay;

            var projectileSpeed = 0f;
            if (projectilePrefab.TryGetComponent<ProjectileSimple>(out var ps))
                projectileSpeed = ps.desiredForwardSpeed;

            if (projectilePrefab.TryGetComponent<ProjectileCharacterController>(out var pcc))
                projectileSpeed = pcc.velocity;

            if (body.teamComponent.teamIndex != TeamIndex.Player)
                AdvPredictionPlugin.GetProjectileSpeedModifiers(ref projectileSpeed);

            if (projectileSpeed > 0f)
            {
                //Velocity shows up as 0 for clients due to not having authority over the CharacterMotor
                //Less accurate, but it works online.
                var dt = 1 / Time.fixedDeltaTime;
                var pT = targetBody.transform.position;
                var vT = (pT - targetBody.previousPosition) * dt;
                var aT = Vector3.zero;

                var motor = targetBody.characterMotor;
                if (motor)
                {
                    if (motor.Motor && motor.Motor.enabled)
                        vT = motor.velocity;

                    aT = PreMove(motor, vT) * dt;
                }

                if (vT.sqrMagnitude > zero) //Dont bother predicting stationary targets
                {
                    return GetRay(aimRay, projectileSpeed, Vector3.zero, pT, vT, aT);
                }
            }

            return aimRay;
        }

        private static CharacterBody GetAimTargetBody(CharacterBody body)
        {
            var aiComponents = body.master.aiComponents;
            for (var i = 0; i < aiComponents.Length; i++)
            {
                var ai = aiComponents[i];
                if (ai && ai.hasAimTarget)
                {
                    var aimTarget = ai.skillDriverEvaluation.aimTarget;
                    if (aimTarget.characterBody && aimTarget.characterBody.teamComponent.teamIndex != body.teamComponent.teamIndex && aimTarget.healthComponent && aimTarget.healthComponent.alive)
                    {
                        return aimTarget.characterBody;
                    }
                }
            }
            return null;
        }

        private static Vector3 PreMove(CharacterMotor motor, Vector3 velocity)
        {
            var accel = motor.acceleration;
            if (motor.isAirControlForced || !motor.isGrounded)
            {
                accel *= motor.disableAirControlUntilCollision ? 0f : motor.airControl;
            }

            var moveVector = motor.moveDirection;
            if (!motor.isFlying)
                moveVector.y = 0f;

            if (motor.body.isSprinting && moveVector.magnitude is < 1f and > 0f)
                moveVector *= 1f / moveVector.magnitude;

            moveVector *= motor.walkSpeed;
            if (!motor.isFlying)
                moveVector.y = velocity.y;

            var nextVelocity = Vector3.MoveTowards(velocity, moveVector, accel * Time.fixedDeltaTime);
            if (motor.useGravity)
            {
                ref var y = ref nextVelocity.y;
                y += Physics.gravity.y * Time.fixedDeltaTime;
                if (motor.isGrounded)
                {
                    y = Mathf.Max(y, 0f);
                }
            }
            return nextVelocity - velocity;
        }

        //All in world space! Gets point you have to aim to
        //NOTE: this will break with infinite speed projectiles!
        //https://gamedev.stackexchange.com/questions/149327/projectile-aim-prediction-with-acceleration
        private static Ray GetRay(Ray aimRay, float sP, Vector3 aP, Vector3 pT, Vector3 vT, Vector3 aT)
        {
            // target position relative to ray position
            pT -= aimRay.origin;

            var useAccel = aT.sqrMagnitude > zero;

            //quartic coefficients
            // a = t^4 * (aT·aT / 4.0)
            // b = t^3 * (aT·vT)
            // c = t^2 * (aT·pT + vT·vT - s^2)
            // d = t   * (2.0 * vT·pT)
            // e =       pT·pT
            var c = vT.sqrMagnitude - (sP * sP);
            var d = 2f * Vector3.Dot(vT, pT);
            var e = pT.sqrMagnitude;

            float[] tValues;
            if (useAccel)
            {
                var a = aT.sqrMagnitude * 0.25f;
                var b = Vector3.Dot(aT, vT);
                c += Vector3.Dot(aT, pT);

                tValues = SolveQuartic(a, b, c, d, e);
            }
            else
            {
                tValues = SolveQuadratic(c, d, e);
            }

            if (tValues is not null)
            {
                var t = float.MaxValue;
                for (var i = 0; i < tValues.Length; i++)
                {
                    var val = tValues[i];
                    if (0f < val && val < t)
                        t = tValues[i];
                }

                if (t < float.MaxValue)
                {
                    var pT_final = pT + (vT * t) + (0.5f * aT * t * t);
                    if (Physics.Linecast(pT + aimRay.origin, pT_final + aimRay.origin, out var hitInfo))
                    {
                        pT_final = Vector3.MoveTowards(pT, pT_final, hitInfo.distance) + hitInfo.normal;
                    }
                    //var vP = (pT_final / t) - (0.5f * aP * t);
                    pT_final /= t;
                    aimRay.direction = Vector3.Lerp(aimRay.direction, pT_final.normalized, AdvPredictionConfig.accuracy.Value * 0.01f);
                }
            }

            return aimRay;
        }

        /*
         * Solves the equation ax^2+bx+c=0. Solutions are returned in a sorted array
         * if they exist.
         * 
         * @param a coefficient of x^2
         * @param b coefficient of x^1
         * @param c coefficient of x^0
         * @return an array containing the two real roots, or <code>null</code> if
         *         no real solutions exist
         */
        public static float[] SolveQuadratic(float a, float b, float c)
        {
            var disc = (b * b) - (4f * a * c);
            if (disc < 0f)
                return null;

            disc = Mathf.Sqrt(disc);
            var q = (b < 0f) ? -0.5f * (b - disc) : -0.5f * (b + disc);
            var t0 = q / a;
            var t1 = c / q;

            return [t0, t1];
        }

        /*
         * Solve a quartic equation of the form ax^4+bx^3+cx^2+cx^1+d=0. The roots
         * are returned in a sorted array of floats in increasing order.
         * 
         * @param a coefficient of x^4
         * @param b coefficient of x^3
         * @param c coefficient of x^2
         * @param d coefficient of x^1
         * @param e coefficient of x^0
         * @return a sorted array of roots, or <code>null</code> if no solutions
         *         exist
         */
        public static float[] SolveQuartic(float a, float b, float c, float d, float e)
        {
            var inverseA = 1f / a;
            var c1 = b * inverseA;
            var c2 = c * inverseA;
            var c3 = d * inverseA;
            var c4 = e * inverseA;

            // cubic resolvant
            var c12 = c1 * c1;
            var p = (-0.375f * c12) + c2;
            var q = (0.125f * c12 * c1) - (0.5f * c1 * c2) + c3;
            var r = (-0.01171875f * c12 * c12) + (0.0625f * c12 * c2) - (0.25f * c1 * c3) + c4;
            var z = SolveCubicForQuartic(-0.5f * p, -r, (0.5f * r * p) - (0.125f * q * q));
            var d1 = (2.0f * z) - p;
            if (d1 < 0f)
            {
                if (d1 > 1.0e-10f)
                    d1 = 0f;
                else
                    return null;
            }
            float d2;
            if (d1 < 1.0e-10f)
            {
                d2 = (z * z) - r;
                if (d2 < 0f)
                    return null;
                d2 = Mathf.Sqrt(d2);
            }
            else
            {
                d1 = Mathf.Sqrt(d1);
                d2 = 0.5f * q / d1;
            }

            // setup useful values for the quadratic factors
            var q1 = d1 * d1;
            var q2 = -0.25f * c1;
            var pm = q1 - (4f * (z - d2));
            var pp = q1 - (4f * (z + d2));
            if (pm >= 0f && pp >= 0f)
            {
                // 4 roots (!)
                pm = Mathf.Sqrt(pm);
                pp = Mathf.Sqrt(pp);
                float[] results =
                [
                    (-0.5f * (d1 + pm)) + q2,
                    (-0.5f * (d1 - pm)) + q2,
                    (0.5f * (d1 + pp)) + q2,
                    (0.5f * (d1 - pp)) + q2,
                ];

                return results;
            }
            else if (pm >= 0)
            {
                pm = Mathf.Sqrt(pm);
                float[] results = [(-0.5f * (d1 + pm)) + q2, (-0.5f * (d1 - pm)) + q2];
                return results;
            }
            else if (pp >= 0)
            {
                pp = Mathf.Sqrt(pp);
                float[] results = [(0.5f * (d1 - pp)) + q2, (0.5f * (d1 + pp)) + q2];
                return results;
            }
            return null;
        }

        /*
         * Return only one root for the specified cubic equation. This routine is
         * only meant to be called by the quartic solver. It assumes the cubic is of
         * the form: x^3+px^2+qx+r.
         * 
         * @param p
         * @param q
         * @param r
         * @return
         */
        private static float SolveCubicForQuartic(float p, float q, float r)
        {
            var A2 = p * p;
            var Q = (A2 - (3f * q)) / 9f;
            var R = ((p * (A2 - (4.5f * q))) + (13.5f * r)) / 27f;
            var Q3 = Q * Q * Q;
            var R2 = R * R;
            var d = Q3 - R2;
            var an = p / 3f;

            if (d >= 0f)
            {
                d = R / Mathf.Sqrt(Q3);
                var theta = Mathf.Acos(d) / 3f;
                var sQ = -2f * Mathf.Sqrt(Q);
                return (sQ * Mathf.Cos(theta)) - an;
            }
            else
            {
                var sQ = Mathf.Pow(Mathf.Sqrt(R2 - Q3) + Mathf.Abs(R), 1f / 3f);
                return R < 0f
                    ? sQ + (Q / sQ) - an
                    : -(sQ + (Q / sQ)) - an;
            }
        }
    }
}
