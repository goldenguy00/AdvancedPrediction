using System;
using System.Linq;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using EntityStates;
using HarmonyLib;
using R2API.Utils;
using RoR2;

using SYS = System.Reflection.Emit;

using RIFT = RiftTitansMod.Modules;
using RIFT_SKILLS = RiftTitansMod.SkillStates;
using EPC = EnemiesPlus.Content;
using MonoMod.Cil;

namespace AdvancedPrediction.Prediction
{
    public static class HarmonyHooks
    {
        public static Harmony Patcher;

        public static void Init()
        {
            Patcher = new Harmony(AdvPredictionPlugin.PluginGUID);
            if (AdvPredictionPlugin.LeagueOfLiteralGays)
            {
                League();
            }
            if (AdvPredictionPlugin.EnemiesPlusInstalled)
            {
                Enemies();
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining | MethodImplOptions.NoOptimization)]
        private static void League()
        {
            Patcher.CreateClassProcessor(typeof(RaptorPrediction)).Patch();
        }

        [MethodImpl(MethodImplOptions.NoInlining | MethodImplOptions.NoOptimization)]
        private static void Enemies()
        {
            Patcher.CreateClassProcessor(typeof(EnemiesPlusBeetle)).Patch();
            Patcher.CreateClassProcessor(typeof(EnemiesPlusImp)).Patch();
        }
    }

    [HarmonyPatch(typeof(RIFT_SKILLS.Chicken.Shoot), nameof(RIFT_SKILLS.Chicken.Shoot.Fire))]
    public class RaptorPrediction
    {
        [HarmonyTranspiler]
        private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
        {
            foreach (var code in instructions)
            {
                yield return code;

                if (code.Calls(AccessTools.Method(typeof(BaseState), nameof(BaseState.GetAimRay))))
                {
                    yield return new CodeInstruction(SYS.OpCodes.Ldarg_0);
                    yield return new CodeInstruction(SYS.OpCodes.Call, AccessTools.PropertyGetter(typeof(EntityState), nameof(EntityState.characterBody)));
                    yield return new CodeInstruction(SYS.OpCodes.Ldsfld, AccessTools.DeclaredField(typeof(RIFT.Projectiles), nameof(RIFT.Projectiles.chickenProjectilePrefab)));
                    yield return new CodeInstruction(SYS.OpCodes.Call, typeof(PredictionUtils).GetMethodCached(nameof(PredictionUtils.PredictAimray)));
                }
            }
        }
    }

    [HarmonyPatch(typeof(EPC.Beetle.BeetleSpit), nameof(EPC.Beetle.BeetleSpit.Fire))]
    public class EnemiesPlusBeetle
    {
        [HarmonyILManipulator]
        public static void ILBeetle(ILContext il)
        {
            PredictionHooks.FireProjectile<EPC.Beetle.BeetleSpit>(new(il));
        }
    }

    [HarmonyPatch(typeof(EPC.Imp.ImpVoidSpike), nameof(EPC.Imp.ImpVoidSpike.HandleSlash))]
    public class EnemiesPlusImp
    {
        [HarmonyILManipulator]
        public static void ILImp(ILContext il)
        {
            PredictionHooks.FireProjectile<EPC.Imp.ImpVoidSpike>(new(il));
        }
    }
}
