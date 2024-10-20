using BepInEx.Logging;

namespace AdvancedPrediction
{
    public static class Log
    {
        private static ManualLogSource _logSource;

        internal static void Init(ManualLogSource logSource)
        {
            _logSource = logSource;
        }

        public static void Debug(string data) => _logSource.LogDebug(data);
        public static void Error(string data) => _logSource.LogError(data);
        public static void Fatal(string data) => _logSource.LogFatal(data);
        public static void Info(string data) => _logSource.LogInfo(data);
        public static void Message(string data) => _logSource.LogMessage(data);
        public static void Warning(string data) => _logSource.LogWarning(data);
    }
}