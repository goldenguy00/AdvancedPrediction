using System.Runtime.CompilerServices;
using System.Security;
using System.Security.Permissions;
using BepInEx;
using BepInEx.Bootstrap;
using AdvancedPrediction.Prediction;
using RoR2;

[module: UnverifiableCode]
#pragma warning disable CS0618 // Type or member is obsolete
[assembly: SecurityPermission(SecurityAction.RequestMinimum, SkipVerification = true)]
#pragma warning restore CS0618 // Type or member is obsolete

namespace AdvancedPrediction
{
    [BepInDependency("com.phreel.TitansOfTheRiftSOTV", BepInDependency.DependencyFlags.SoftDependency)]
    [BepInDependency("com.score.EnemiesPlusPlus", BepInDependency.DependencyFlags.SoftDependency)]
    [BepInDependency("com.rune580.riskofoptions", BepInDependency.DependencyFlags.SoftDependency)]
    [BepInPlugin(PluginGUID, PluginName, PluginVersion)]
    public class AdvPredictionPlugin : BaseUnityPlugin
    {
        public const string PluginGUID = $"com.{PluginAuthor}.{PluginName}";
        public const string PluginAuthor = "score";
        public const string PluginName = "AdvancedPrediction";
        public const string PluginVersion = "1.0.4";

        public static AdvPredictionPlugin Instance { get; private set; }

        public static bool RooInstalled => Chainloader.PluginInfos.ContainsKey("com.rune580.riskofoptions");
        public static bool InfernoInstalled => Chainloader.PluginInfos.ContainsKey("HIFU.Inferno");
        public static bool RiskyArtifactsInstalled => Chainloader.PluginInfos.ContainsKey("com.Moffein.RiskyArtifacts");
        public static bool LeagueOfLiteralGays => Chainloader.PluginInfos.ContainsKey("com.phreel.TitansOfTheRiftSOTV");
        public static bool EnemiesPlusInstalled => Chainloader.PluginInfos.ContainsKey("com.score.EnemiesPlusPlus");

        public void Awake()
        {
            Instance = this;
            Log.Init(Logger);
            AdvPredictionConfig.Init(Config);

            PredictionHooks.Init();
            HarmonyHooks.Init();
        }


        [MethodImpl(MethodImplOptions.NoInlining | MethodImplOptions.NoOptimization)]
        private static float GetRiskyArtifactsWarfareProjectileSpeedMult()
        {
            if (RunArtifactManager.instance && RunArtifactManager.instance.IsArtifactEnabled(Risky_Artifacts.Artifacts.Warfare.artifact))
            {
                return Risky_Artifacts.Artifacts.Warfare.projSpeed;
            }
            return 1f;
        }

        [MethodImpl(MethodImplOptions.NoInlining | MethodImplOptions.NoOptimization)]
        private static float GetInfernoProjectileSpeedMult()
        {
            if (Run.instance && Run.instance.selectedDifficulty == Inferno.Main.InfernoDiffIndex)
            {
                return Inferno.Main.ProjectileSpeed.Value;
            }
            return 1f;
        }

        [MethodImpl(MethodImplOptions.NoInlining | MethodImplOptions.NoOptimization)]
        public static void GetProjectileSpeedModifiers(ref float speed)
        {
            if (InfernoInstalled)
                speed *= GetInfernoProjectileSpeedMult();

            if (RiskyArtifactsInstalled)
                speed *= GetRiskyArtifactsWarfareProjectileSpeedMult();
        }
    }
}
