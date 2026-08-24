using System;
using UnityEngine;

[Serializable]
public sealed class PicoControllerState
{
    public bool tracked;
    public float px;
    public float py;
    public float pz;
    public float qx;
    public float qy;
    public float qz;
    public float qw = 1.0f;
    public float vx;
    public float vy;
    public float vz;
    public float avx;
    public float avy;
    public float avz;
    public float trigger;
    public float grip;
    public float stickX;
    public float stickY;
    public bool stickClick;
    public bool primary;
    public bool secondary;

    public void Set(
        bool isTracked,
        Vector3 position,
        Quaternion rotation,
        Vector3 velocity,
        Vector3 angularVelocity,
        float triggerValue,
        float gripValue,
        Vector2 stickValue,
        bool stickClicked,
        bool primaryPressed,
        bool secondaryPressed)
    {
        tracked = isTracked;
        px = position.x;
        py = position.y;
        pz = position.z;
        qx = rotation.x;
        qy = rotation.y;
        qz = rotation.z;
        qw = rotation.w;
        vx = velocity.x;
        vy = velocity.y;
        vz = velocity.z;
        avx = angularVelocity.x;
        avy = angularVelocity.y;
        avz = angularVelocity.z;
        trigger = triggerValue;
        grip = gripValue;
        stickX = stickValue.x;
        stickY = stickValue.y;
        stickClick = stickClicked;
        primary = primaryPressed;
        secondary = secondaryPressed;
    }
}

[Serializable]
public sealed class PicoInputPacket
{
    public string schema = "nero.quest.input.v1";
    public long sequence;
    public long unixTimeMs;
    public long unixTimeNs;
    public double monotonicTimeSec;
    public PicoControllerState head = new PicoControllerState();
    public PicoControllerState left = new PicoControllerState();
    public PicoControllerState right = new PicoControllerState();
}

[Serializable]
public sealed class PicoSyncPing
{
    public string schema;
    public string kind;
    public long sequence;
    public long hostSendUnixNs;
    public long hostSendMonotonicNs;
}

[Serializable]
public sealed class PicoSyncPong
{
    public string schema = "nero.quest.sync.v1";
    public string kind = "pong";
    public long sequence;
    public long hostSendUnixNs;
    public long hostSendMonotonicNs;
    public long questReceiveUnixNs;
    public long questReceiveMonotonicNs;
    public long questSendUnixNs;
    public long questSendMonotonicNs;
}
