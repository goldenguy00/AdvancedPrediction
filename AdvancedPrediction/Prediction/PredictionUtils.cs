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

            var projectileSpeed = 0f;
            if (projectilePrefab.TryGetComponent<ProjectileSimple>(out var ps))
                projectileSpeed = ps.desiredForwardSpeed;

            if (projectilePrefab.TryGetComponent<ProjectileCharacterController>(out var pcc))
                projectileSpeed = pcc.velocity;

            if (body.teamComponent.teamIndex != TeamIndex.Player)
                AdvPredictionPlugin.GetProjectileSpeedModifiers(ref projectileSpeed);

            var targetBody = GetAimTargetBody(body);
            if (projectileSpeed > 0f && targetBody)
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

            double[] tValues;
            if (useAccel)
            {
                var a = aT.sqrMagnitude * 0.25f;
                var b = Vector3.Dot(aT, vT);
                c += Vector3.Dot(aT, pT);

                //solve with newton
                tValues = SolveQuartic(a, b, c, d, e);
            }
            else
            {
                tValues = SolveQuadratic(c, d, e);
            }

            if (tValues != null)
            {
                var t = float.MaxValue;
                for (var i = 0; i < tValues.Length; i++ )
                {
                    var val = (float)tValues[i];
                    if (val > 0f & val < t)
                        t = val;
                }

                if (t is > 0f and < float.MaxValue)
                {
                    var pT_final = pT + (vT * t) + (0.5f * aT * t * t);
                    if (Physics.Linecast(pT + aimRay.origin, pT_final + aimRay.origin, out var hitInfo))
                    {
                        pT_final = Vector3.MoveTowards(pT, pT_final, hitInfo.distance) + hitInfo.normal;
                    }
                    var vP = (pT_final / t) - (0.5f * aP * t);
                    vP.Normalize();
                    return new Ray(aimRay.origin, Vector3.Lerp(aimRay.direction, vP, AdvPredictionConfig.accuracy.Value * 0.01f));
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
        public static double[] SolveQuadratic(double a, double b, double c)
        {
            var disc = (b * b) - (4 * a * c);
            if (disc < 0)
                return null;

            disc = Math.Sqrt(disc);
            var q = (b < 0) ? -0.5 * (b - disc) : -0.5 * (b + disc);
            var t0 = q / a;
            var t1 = c / q;

            // return sorted array
            return (t0 > t1) ? [t1, t0] : [t0, t1];
        }

        /*
         * Solve a quartic equation of the form ax^4+bx^3+cx^2+cx^1+d=0. The roots
         * are returned in a sorted array of doubles in increasing order.
         * 
         * @param a coefficient of x^4
         * @param b coefficient of x^3
         * @param c coefficient of x^2
         * @param d coefficient of x^1
         * @param e coefficient of x^0
         * @return a sorted array of roots, or <code>null</code> if no solutions
         *         exist
         */
        public static double[] SolveQuartic(double a, double b, double c, double d, double e)
        {
            var inverseA = 1 / a;
            var c1 = b * inverseA;
            var c2 = c * inverseA;
            var c3 = d * inverseA;
            var c4 = e * inverseA;

            // cubic resolvant
            var c12 = c1 * c1;
            var p = (-0.375 * c12) + c2;
            var q = (0.125 * c12 * c1) - (0.5 * c1 * c2) + c3;
            var r = (-0.01171875 * c12 * c12) + (0.0625 * c12 * c2) - (0.25 * c1 * c3) + c4;
            var z = SolveCubicForQuartic(-0.5 * p, -r, (0.5 * r * p) - (0.125 * q * q));
            var d1 = (2.0 * z) - p;
            if (d1 < 0)
            {
                if (d1 > 1.0e-10)
                    d1 = 0;
                else
                    return null;
            }
            double d2;
            if (d1 < 1.0e-10)
            {
                d2 = (z * z) - r;
                if (d2 < 0)
                    return null;
                d2 = Math.Sqrt(d2);
            }
            else
            {
                d1 = Math.Sqrt(d1);
                d2 = 0.5 * q / d1;
            }
            // setup usefull values for the quadratic factors
            var q1 = d1 * d1;
            var q2 = -0.25 * c1;
            var pm = q1 - (4 * (z - d2));
            var pp = q1 - (4 * (z + d2));
            if (pm >= 0 && pp >= 0)
            {
                // 4 roots (!)
                pm = Math.Sqrt(pm);
                pp = Math.Sqrt(pp);
                double[] results =
                [
                    (-0.5 * (d1 + pm)) + q2,
                    (-0.5 * (d1 - pm)) + q2,
                    (0.5 * (d1 + pp)) + q2,
                    (0.5 * (d1 - pp)) + q2,
                ];
                // tiny insertion sort
                for (var i = 1; i < 4; i++)
                {
                    for (var j = i; j > 0 && results[j - 1] > results[j]; j--)
                    {
                        (results[j - 1], results[j]) = (results[j], results[j - 1]);
                    }
                }
                return results;
            }
            else if (pm >= 0)
            {
                pm = Math.Sqrt(pm);
                double[] results = [(-0.5 * (d1 + pm)) + q2, (-0.5 * (d1 - pm)) + q2];
                return results;
            }
            else if (pp >= 0)
            {
                pp = Math.Sqrt(pp);
                double[] results = [(0.5 * (d1 - pp)) + q2, (0.5 * (d1 + pp)) + q2];
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
        private static double SolveCubicForQuartic(double p, double q, double r)
        {
            var A2 = p * p;
            var Q = (A2 - (3.0 * q)) / 9.0;
            var R = ((p * (A2 - (4.5 * q))) + (13.5 * r)) / 27.0;
            var Q3 = Q * Q * Q;
            var R2 = R * R;
            var d = Q3 - R2;
            var an = p / 3.0;

            if (d >= 0)
            {
                d = R / Math.Sqrt(Q3);
                var theta = Math.Acos(d) / 3.0;
                var sQ = -2.0 * Math.Sqrt(Q);
                return (sQ * Math.Cos(theta)) - an;
            }
            else
            {
                var sQ = Math.Pow(Math.Sqrt(R2 - Q3) + Math.Abs(R), 1.0 / 3.0);
                return R < 0
                    ? sQ + (Q / sQ) - an
                    : -(sQ + (Q / sQ)) - an;
            }
        }
    }
}
