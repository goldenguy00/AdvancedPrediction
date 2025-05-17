using System;
using System.Runtime.CompilerServices;
using EntityStates;
using EntityStates.BeetleGuardMonster;
using EntityStates.Bell.BellWeapon;
using EntityStates.ClayBoss;
using EntityStates.ClayBoss.ClayBossWeapon;
using EntityStates.GravekeeperBoss;
using EntityStates.GreaterWispMonster;
using EntityStates.ImpBossMonster;
using EntityStates.ImpMonster;
using EntityStates.LemurianBruiserMonster;
using EntityStates.LemurianMonster;
using EntityStates.RoboBallBoss.Weapon;
using EntityStates.ScavMonster;
using EntityStates.VagrantMonster.Weapon;
using EntityStates.Vulture.Weapon;
using HarmonyLib;
using Mono.Cecil.Cil;
using MonoMod.Cil;
using RoR2;
using RoR2.Projectile;
using UnityEngine;

namespace AdvancedPrediction.Prediction
{
    /// <summary>
    /// literally just rewrote this to demonstrate how concise it could have been
    /// </summary>
    public class PredictionHooks
    {
        public static void Init()
        {
            On.RoR2.CharacterAI.BaseAI.FindEnemyHurtBox += (orig, self, maxDist, _, filterByLoS) => orig(self, maxDist, true/*360*/, filterByLoS);

            // special cases
            IL.EntityStates.BeetleGuardMonster.FireSunder.FixedUpdate += (il) => FireSunder_FixedUpdate(new(il));
            IL.EntityStates.Bell.BellWeapon.ChargeTrioBomb.FixedUpdate += (il) => ChargeTrioBomb_FixedUpdate(new(il));

            // generic type T
            IL.EntityStates.GenericProjectileBaseState.FireProjectile += (il) => FireProjectile<GenericProjectileBaseState>(new(il));
            IL.EntityStates.GreaterWispMonster.FireCannons.OnEnter += (il) => FireProjectile<FireCannons>(new(il));
            IL.EntityStates.RoboBallBoss.Weapon.FireEyeBlast.FixedUpdate += (il) => FireProjectile<FireEyeBlast>(new(il));
            IL.EntityStates.Vulture.Weapon.FireWindblade.OnEnter += (il) => FireProjectile<FireWindblade>(new(il));
            IL.EntityStates.GravekeeperBoss.FireHook.OnEnter += (il) => FireProjectile<FireHook>(new(il));
            IL.EntityStates.LemurianMonster.FireFireball.OnEnter += (il) => FireProjectile<FireFireball>(new(il));
            IL.EntityStates.LemurianBruiserMonster.FireMegaFireball.FixedUpdate += (il) => FireProjectile<FireMegaFireball>(new(il));
            IL.EntityStates.ScavMonster.FireEnergyCannon.OnEnter += (il) => FireProjectile<FireEnergyCannon>(new(il));
            IL.EntityStates.ClayBoss.ClayBossWeapon.FireBombardment.FireGrenade += (il) => FireProjectile<FireBombardment>(new(il));
            IL.EntityStates.ClayBoss.FireTarball.FireSingleTarball += (il) => FireProjectile<FireTarball>(new(il));
            IL.EntityStates.ImpMonster.FireSpines.FixedUpdate += (il) => FireProjectile<FireSpines>(new(il));

            //fire many
            IL.EntityStates.VagrantMonster.Weapon.JellyBarrage.FixedUpdate += (il) => FireProjectileGroup<JellyBarrage>(new(il));
            IL.EntityStates.ImpBossMonster.FireVoidspikes.FixedUpdate += (il) => FireProjectileGroup<FireVoidspikes>(new(il));
        }

        #region Generics
        public static void FireProjectile<T>(ILCursor c, string prefabName)
        {
            if (c.TryGotoNext(MoveType.After, x => x.MatchCall<BaseState>(nameof(BaseState.GetAimRay))))
                c.EmitPredictAimray<T>(prefabName);
            else Log.Error("AccurateEnemies: Generic OnEnter IL Hook failed ");
        }

        public static void FireProjectile<T>(ILCursor c)
        {
            if (c.TryGotoNext(MoveType.After, x => x.MatchCall<BaseState>(nameof(BaseState.GetAimRay))))
                c.EmitPredictAimray<T>();
            else Log.Error("AccurateEnemies: Generic OnEnter IL Hook failed ");
        }

        public static void FireProjectileGroup<T>(ILCursor c)
        {
            while (c.TryGotoNext(MoveType.After, x => x.MatchCall<BaseState>(nameof(BaseState.GetAimRay))))
                c.EmitPredictAimray<T>();
        }
        #endregion

        private static void FireSunder_FixedUpdate(ILCursor c)
        {
            int loc = -1;

            if (c.TryGotoNext(x => x.MatchCall<BaseState>(nameof(BaseState.GetAimRay))) &&
                c.TryGotoNext(x => x.MatchStloc(out loc)) && loc != -1 &&
                c.TryGotoNext(x => x.MatchCall(AccessTools.PropertyGetter(typeof(ProjectileManager), nameof(ProjectileManager.instance)))))
            {
                c.Emit(OpCodes.Ldloc, loc);
                c.EmitPredictAimray<FireSunder>();
                c.Emit(OpCodes.Stloc, loc);
            }
            else Log.Error("AccurateEnemies: EntityStates.BeetleGuardMonster.FireSunder.FixedUpdate IL Hook failed");
        }

        private static void ChargeTrioBomb_FixedUpdate(ILCursor c)
        {
            int rayLoc = -1, transformLoc = -1;

            if (c.TryGotoNext(x => x.MatchCall<BaseState>(nameof(BaseState.GetAimRay))) &&
                c.TryGotoNext(x => x.MatchStloc(out rayLoc)) &&
                c.TryGotoNext(x => x.MatchCall<ChargeTrioBomb>(nameof(ChargeTrioBomb.FindTargetChildTransformFromBombIndex))) &&
                c.TryGotoNext(x => x.MatchStloc(out transformLoc)) &&
                c.TryGotoNext(x => x.MatchCall(AccessTools.PropertyGetter(typeof(ProjectileManager), nameof(ProjectileManager.instance)))))
            {
                //aimray
                c.Emit(OpCodes.Ldloc, rayLoc);

                //transform
                c.Emit(OpCodes.Ldloc, transformLoc);

                //body
                c.Emit(OpCodes.Ldarg_0);
                c.Emit(OpCodes.Call, AccessTools.PropertyGetter(typeof(ChargeTrioBomb), nameof(ChargeTrioBomb.characterBody)));

                // prefab
                c.Emit(OpCodes.Ldsfld, AccessTools.DeclaredField(typeof(ChargeTrioBomb), nameof(ChargeTrioBomb.bombProjectilePrefab)));

                // aimRay, transform, characterBody, projectilePrefab
                c.EmitDelegate<Func<Ray, Transform, CharacterBody, GameObject, Ray>>((aimRay, transform, body, prefab) =>
                {
                    aimRay.origin = transform.position;
                    return PredictionUtils.PredictAimray(aimRay, body, prefab);
                });
                //aimray
                c.Emit(OpCodes.Stloc, rayLoc);
            }
            else Log.Error("AccurateEnemies: EntityStates.Bell.BellWeapon.ChargeTrioBomb.FixedUpdate IL Hook failed");
        }
    }
}
