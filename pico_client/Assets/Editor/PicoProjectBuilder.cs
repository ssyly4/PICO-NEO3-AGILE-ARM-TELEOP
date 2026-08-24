using System;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;
using UnityEditor.XR.OpenXR.Features;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Management;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR.OpenXR.Features;
using UnityEngine.XR.OpenXR.Features.Interactions;
using Unity.XR.PXR;
using Unity.XR.OpenXR.Features.PICOSupport;

public static class PicoProjectBuilder
{
    private const string ScenePath = "Assets/Scenes/PicoControllerProbe.unity";
    private const string DefaultDestinationHost = "127.0.0.1";

    private static string ApkPath
    {
        get
        {
            string configured = Environment.GetEnvironmentVariable("NERO_PICO_APK");
            if (!string.IsNullOrWhiteSpace(configured))
            {
                return configured;
            }

            string projectRoot = Directory.GetParent(Application.dataPath)?.FullName
                ?? throw new InvalidOperationException("Could not resolve the Unity project root.");
            return Path.GetFullPath(Path.Combine(
                projectRoot, "..", "artifacts", "builds", "NeroPicoControllerProbe.apk"));
        }
    }

    public static void Configure()
    {
        if (!EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, BuildTarget.Android))
        {
            throw new InvalidOperationException("Failed to switch the active build target to Android.");
        }

        PlayerSettings.productName = "NERO PICO Controller Probe";
        PlayerSettings.companyName = "NERO";
        PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Android, "com.nero.teleop.picocontrollerprobe");
        PlayerSettings.SetScriptingBackend(NamedBuildTarget.Android, ScriptingImplementation.IL2CPP);
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
        PlayerSettings.Android.targetSdkVersion = AndroidSdkVersions.AndroidApiLevelAuto;
        PlayerSettings.Android.forceInternetPermission = true;
        PlayerSettings.defaultInterfaceOrientation = UIOrientation.LandscapeLeft;
        PlayerSettings.colorSpace = ColorSpace.Linear;
        PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new[] { GraphicsDeviceType.OpenGLES3 });

        ConfigureOpenXR();
        ConfigurePlatformChecks();
        CreateProbeScene();
        AssetDatabase.SaveAssets();
        Debug.Log("NERO_PICO_CONFIGURED");
    }

    private static void ConfigurePlatformChecks()
    {
        const string settingPath = "Assets/Resources/PXR_PlatformSetting.asset";
        PXR_PlatformSetting platform = AssetDatabase.LoadAssetAtPath<PXR_PlatformSetting>(settingPath);
        if (platform == null)
        {
            Directory.CreateDirectory("Assets/Resources");
            platform = ScriptableObject.CreateInstance<PXR_PlatformSetting>();
            AssetDatabase.CreateAsset(platform, settingPath);
        }

        platform.startTimeEntitlementCheck = false;
        platform.entitlementCheckSimulation = false;
        platform.appID = string.Empty;
        PXR_PlatformSetting.Instance = platform;
        EditorUtility.SetDirty(platform);
    }

    public static void ConfigureAndBuild()
    {
        Configure();
        Directory.CreateDirectory(Path.GetDirectoryName(ApkPath) ?? throw new InvalidOperationException());

        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = new[] { ScenePath },
            locationPathName = ApkPath,
            target = BuildTarget.Android,
            options = BuildOptions.Development,
        });

        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new InvalidOperationException(
                $"PICO APK build failed: result={report.summary.result} errors={report.summary.totalErrors}");
        }

        Debug.Log($"NERO_PICO_APK={ApkPath} size={report.summary.totalSize}");
    }

    private static void ConfigureOpenXR()
    {
        const BuildTargetGroup group = BuildTargetGroup.Android;
        MethodInfo getOrCreate = typeof(XRGeneralSettingsPerBuildTarget).GetMethod(
            "GetOrCreate", BindingFlags.Static | BindingFlags.NonPublic);
        var settingsStore = getOrCreate?.Invoke(null, null) as XRGeneralSettingsPerBuildTarget;
        if (settingsStore == null)
        {
            throw new InvalidOperationException("XR settings store could not be created.");
        }

        if (!settingsStore.HasManagerSettingsForBuildTarget(group))
        {
            settingsStore.CreateDefaultManagerSettingsForBuildTarget(group);
        }

        XRGeneralSettings targetSettings = settingsStore.SettingsForBuildTarget(group);
        XRManagerSettings manager = settingsStore.ManagerSettingsForBuildTarget(group);
        if (!XRPackageMetadataStore.AssignLoader(manager, typeof(OpenXRLoader).FullName, group) &&
            !XRPackageMetadataStore.IsLoaderAssigned(typeof(OpenXRLoader).FullName, group))
        {
            throw new InvalidOperationException("Failed to assign the OpenXR loader to Android.");
        }

        FeatureHelpers.RefreshFeatures(group);
        OpenXRSettings settings = OpenXRSettings.GetSettingsForBuildTargetGroup(group);
        if (settings == null)
        {
            throw new InvalidOperationException("Android OpenXR settings were not created.");
        }

        foreach (OpenXRInteractionFeature feature in settings.GetFeatures<OpenXRInteractionFeature>())
        {
            feature.enabled = feature is PICONeo3ControllerProfile;
            EditorUtility.SetDirty(feature);
        }

        PICOFeature picoSupport = EnableFeature<PICOFeature>(settings);
        picoSupport.isPicoSupport = true;
        EnableFeature<PICONeo3ControllerProfile>(settings);

        EditorUtility.SetDirty(settings);
        EditorUtility.SetDirty(manager);
        EditorUtility.SetDirty(targetSettings);
    }

    private static T EnableFeature<T>(OpenXRSettings settings)
        where T : OpenXRFeature
    {
        T feature = settings.GetFeatures<T>().OfType<T>().FirstOrDefault();
        if (feature == null)
        {
            throw new InvalidOperationException($"OpenXR feature is unavailable: {typeof(T).FullName}");
        }

        feature.enabled = true;
        EditorUtility.SetDirty(feature);
        Debug.Log($"NERO_PICO_FEATURE enabled={typeof(T).FullName}");
        return feature;
    }

    private static void CreateProbeScene()
    {
        Directory.CreateDirectory("Assets/Scenes");
        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var root = new GameObject("PICO Controller Probe");
        PicoControllerProbe probe = root.AddComponent<PicoControllerProbe>();
        PicoUdpSender sender = root.AddComponent<PicoUdpSender>();
        sender.source = probe;
        sender.destinationHost = Environment.GetEnvironmentVariable("NERO_PICO_HOST")
            ?? DefaultDestinationHost;
        EditorSceneManager.SaveScene(scene, ScenePath);
        EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
    }
}
