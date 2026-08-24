using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public sealed class PicoControllerProbe : MonoBehaviour
{
    public bool enableDiagnosticLogging = false;
    public float statusUpdateRateHz = 2.0f;

    private static readonly List<InputDevice> NodeDevices = new List<InputDevice>();

    private Camera xrCamera;
    private GameObject leftMarker;
    private GameObject rightMarker;
    private TextMesh statusText;
    private float nextLogTime;
    private float nextStatusUpdateTime;

    public PicoInputPacket LatestPacket { get; } = new PicoInputPacket();

    private void Start()
    {
        xrCamera = CreateCamera();
        leftMarker = CreateMarker("Left controller", new Color(0.1f, 0.7f, 1.0f));
        rightMarker = CreateMarker("Right controller", new Color(1.0f, 0.35f, 0.15f));
        statusText = CreateStatusText(xrCamera.transform);
    }

    private void Update()
    {
        bool captureDiagnostics =
            enableDiagnosticLogging && Time.unscaledTime >= nextLogTime;
        bool headTracked = UpdateTrackedNode(
            XRNode.Head, xrCamera.gameObject, false,
            out float headTrigger, out float headGrip,
            out Vector2 headStick, out bool headStickClick,
            out bool headPrimary, out bool headSecondary,
            out Vector3 headPosition, out Quaternion headRotation,
            out Vector3 headVelocity, out Vector3 headAngularVelocity, captureDiagnostics,
            out string headDevice);
        bool leftTracked = UpdateTrackedNode(
            XRNode.LeftHand, leftMarker, true,
            out float leftTrigger, out float leftGrip,
            out Vector2 leftStick, out bool leftStickClick,
            out bool leftPrimary, out bool leftSecondary,
            out Vector3 leftPosition, out Quaternion leftRotation,
            out Vector3 leftVelocity, out Vector3 leftAngularVelocity,
            captureDiagnostics, out string leftDevice);
        bool rightTracked = UpdateTrackedNode(
            XRNode.RightHand, rightMarker, true,
            out float rightTrigger, out float rightGrip,
            out Vector2 rightStick, out bool rightStickClick,
            out bool rightPrimary, out bool rightSecondary,
            out Vector3 rightPosition, out Quaternion rightRotation,
            out Vector3 rightVelocity, out Vector3 rightAngularVelocity,
            captureDiagnostics, out string rightDevice);

        LatestPacket.head.Set(
            headTracked, headPosition, headRotation, headVelocity, headAngularVelocity,
            headTrigger, headGrip, headStick, headStickClick, headPrimary, headSecondary);
        LatestPacket.left.Set(
            leftTracked, leftPosition, leftRotation, leftVelocity, leftAngularVelocity,
            leftTrigger, leftGrip, leftStick, leftStickClick, leftPrimary, leftSecondary);
        LatestPacket.right.Set(
            rightTracked, rightPosition, rightRotation, rightVelocity, rightAngularVelocity,
            rightTrigger, rightGrip, rightStick, rightStickClick, rightPrimary, rightSecondary);

        if (statusUpdateRateHz > 0.0f && Time.unscaledTime >= nextStatusUpdateTime)
        {
            nextStatusUpdateTime = Time.unscaledTime + 1.0f / statusUpdateRateHz;
            statusText.text =
                $"NERO PICO  H:{TrackingLabel(headTracked)} " +
                $"L:{TrackingLabel(leftTracked)} R:{TrackingLabel(rightTracked)}\n" +
                $"L  T:{leftTrigger:F1} G:{leftGrip:F1}  " +
                $"R  T:{rightTrigger:F1} G:{rightGrip:F1} " +
                $"S:{rightStick.x:F1},{rightStick.y:F1}";
        }

        if (captureDiagnostics)
        {
            nextLogTime = Time.unscaledTime + 0.2f;
            Debug.Log(
                $"NERO_XR_INPUT leftDevice=\"{leftDevice}\" leftTracked={leftTracked} " +
                $"leftPos={Format(leftPosition)} leftRot={Format(leftRotation)} " +
                $"leftVel={Format(leftVelocity)} leftAngVel={Format(leftAngularVelocity)} " +
                $"leftTrigger={leftTrigger:F3} leftGrip={leftGrip:F3} leftPrimary={leftPrimary} leftSecondary={leftSecondary} " +
                $"rightDevice=\"{rightDevice}\" rightTracked={rightTracked} " +
                $"rightPos={Format(rightPosition)} rightRot={Format(rightRotation)} " +
                $"rightVel={Format(rightVelocity)} rightAngVel={Format(rightAngularVelocity)} " +
                $"rightTrigger={rightTrigger:F3} rightGrip={rightGrip:F3} " +
                $"rightStick=({rightStick.x:F3},{rightStick.y:F3}) rightStickClick={rightStickClick} " +
                $"rightPrimary={rightPrimary} rightSecondary={rightSecondary}");
        }
    }

    private static bool UpdateTrackedNode(
        XRNode node,
        GameObject target,
        bool animateMarker,
        out float trigger,
        out float grip,
        out Vector2 stick,
        out bool stickClick,
        out bool primary,
        out bool secondary,
        out Vector3 position,
        out Quaternion rotation,
        out Vector3 velocity,
        out Vector3 angularVelocity,
        bool captureDiagnostics,
        out string deviceLabel)
    {
        InputDevice device = GetPreferredDevice(node);
        bool poseTracked = HasTrackedPose(device, out InputTrackingState trackingState);
        deviceLabel = captureDiagnostics && device.isValid
            ? $"{device.name} [{device.characteristics}] tracking={trackingState}"
            : "none";
        bool hasPosition = device.TryGetFeatureValue(CommonUsages.devicePosition, out position);
        bool hasRotation = device.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation);
        device.TryGetFeatureValue(CommonUsages.deviceVelocity, out velocity);
        device.TryGetFeatureValue(CommonUsages.deviceAngularVelocity, out angularVelocity);
        device.TryGetFeatureValue(CommonUsages.trigger, out trigger);
        device.TryGetFeatureValue(CommonUsages.grip, out grip);
        device.TryGetFeatureValue(CommonUsages.primary2DAxis, out stick);
        device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out stickClick);
        device.TryGetFeatureValue(CommonUsages.primaryButton, out primary);
        device.TryGetFeatureValue(CommonUsages.secondaryButton, out secondary);

        bool tracked = device.isValid && poseTracked && hasPosition && hasRotation;
        target.SetActive(tracked);
        if (!tracked)
        {
            return false;
        }

        target.transform.SetLocalPositionAndRotation(position, rotation);
        if (animateMarker)
        {
            float scale = 0.06f + 0.035f * grip;
            target.transform.localScale = new Vector3(scale, scale, 0.12f + 0.05f * trigger);
            Renderer renderer = target.GetComponent<Renderer>();
            Color baseColor = node == XRNode.LeftHand
                ? new Color(0.1f, 0.7f, 1.0f)
                : new Color(1.0f, 0.35f, 0.15f);
            renderer.material.color = Color.Lerp(baseColor, Color.white, trigger);
        }

        return true;
    }

    private static InputDevice GetPreferredDevice(XRNode node)
    {
        NodeDevices.Clear();
        InputDevices.GetDevicesAtXRNode(node, NodeDevices);

        InputDevice best = default;
        int bestScore = -1;
        foreach (InputDevice device in NodeDevices)
        {
            if (!device.isValid)
            {
                continue;
            }

            int score = 0;
            if ((device.characteristics & InputDeviceCharacteristics.Controller) != 0)
            {
                score += 10;
            }
            if (HasTrackedPose(device, out _))
            {
                score += 100;
            }
            if (score > bestScore)
            {
                best = device;
                bestScore = score;
            }
        }

        return best;
    }

    private static bool HasTrackedPose(InputDevice device, out InputTrackingState trackingState)
    {
        trackingState = InputTrackingState.None;
        if (!device.isValid)
        {
            return false;
        }

        bool hasTrackedFlag = device.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked);
        bool hasTrackingState = device.TryGetFeatureValue(
            CommonUsages.trackingState, out trackingState);
        if (hasTrackingState)
        {
            bool hasPositionTracking =
                (trackingState & InputTrackingState.Position) != 0;
            bool hasRotationTracking =
                (trackingState & InputTrackingState.Rotation) != 0;
            return hasPositionTracking && hasRotationTracking;
        }

        return !hasTrackedFlag || isTracked;
    }

    private static string Format(Vector3 value)
    {
        return $"({value.x:F4},{value.y:F4},{value.z:F4})";
    }

    private static string Format(Quaternion value)
    {
        return $"({value.x:F4},{value.y:F4},{value.z:F4},{value.w:F4})";
    }

    private static string TrackingLabel(bool tracked)
    {
        return tracked ? "OK" : "--";
    }

    private static Camera CreateCamera()
    {
        var cameraObject = new GameObject("XR Camera");
        cameraObject.AddComponent<AudioListener>();
        Camera camera = cameraObject.AddComponent<Camera>();
        camera.nearClipPlane = 0.05f;
        camera.farClipPlane = 100.0f;
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.backgroundColor = new Color(0.015f, 0.015f, 0.02f, 1.0f);
        return camera;
    }

    private static GameObject CreateMarker(string name, Color color)
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
        marker.name = name;
        marker.GetComponent<Renderer>().material.color = color;
        return marker;
    }

    private static TextMesh CreateStatusText(Transform cameraTransform)
    {
        var textObject = new GameObject("Status");
        textObject.transform.SetParent(cameraTransform, false);
        textObject.transform.localPosition = new Vector3(-0.58f, 0.34f, 1.8f);
        textObject.transform.localRotation = Quaternion.identity;
        TextMesh text = textObject.AddComponent<TextMesh>();
        text.fontSize = 28;
        text.characterSize = 0.009f;
        text.anchor = TextAnchor.UpperLeft;
        text.color = Color.white;
        return text;
    }
}
