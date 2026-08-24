using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public sealed class PicoUdpSender : MonoBehaviour
{
    public PicoControllerProbe source;
    public string destinationHost = "127.0.0.1";
    public int destinationPort = 50150;
    public float sendRateHz = 60.0f;
    public bool enableDiagnosticLogging = false;

    private UdpClient client;
    private IPEndPoint destination;
    private Thread senderThread;
    private volatile bool senderRunning;
    private long sequence;
    private readonly byte[] binaryPayload = new byte[244];
    private readonly byte[] stateSnapshot = new byte[216];
    private readonly byte[] stateStaging = new byte[216];
    private readonly object stateLock = new object();
    private readonly System.Diagnostics.Stopwatch monotonicClock =
        new System.Diagnostics.Stopwatch();

    private const long UnixEpochTicks = 621355968000000000L;
    private const int BinaryHeaderSize = 28;
    private const int BinaryStateSize = 72;

    private void Start()
    {
        if (source == null)
        {
            source = GetComponent<PicoControllerProbe>();
        }

        client = new UdpClient();
        client.EnableBroadcast = destinationHost == "255.255.255.255" || destinationHost.EndsWith(".255");
        destination = new IPEndPoint(IPAddress.Parse(destinationHost), destinationPort);
        Application.runInBackground = true;
        monotonicClock.Start();
        senderRunning = true;
        senderThread = new Thread(SenderLoop)
        {
            IsBackground = true,
            Name = "nero-pico-udp",
            Priority = System.Threading.ThreadPriority.AboveNormal,
        };
        senderThread.Start();
        Debug.Log($"NERO_UDP_READY destination={destination} rateHz={sendRateHz:F1}");
    }

    private void Update()
    {
        if (source == null || client == null)
        {
            return;
        }
        WriteStateSnapshot(source.LatestPacket, stateStaging);
        lock (stateLock)
        {
            Buffer.BlockCopy(stateStaging, 0, stateSnapshot, 0, stateSnapshot.Length);
        }
    }

    private void SenderLoop()
    {
        double nextSendSec = monotonicClock.Elapsed.TotalSeconds;
        while (senderRunning)
        {
            double nowSec = monotonicClock.Elapsed.TotalSeconds;
            double waitSec = nextSendSec - nowSec;
            if (waitSec > 0.002)
            {
                Thread.Sleep(Math.Max(1, (int)((waitSec - 0.001) * 1000.0)));
                continue;
            }
            if (waitSec > 0.0)
            {
                Thread.Yield();
                continue;
            }

            nextSendSec = Math.Max(nextSendSec + 1.0 / Math.Max(sendRateHz, 1.0f), nowSec);
            long unixTimeNs = UnixTimeNs();
            WriteBinaryHeader(binaryPayload, sequence++, unixTimeNs, nowSec);
            lock (stateLock)
            {
                Buffer.BlockCopy(
                    stateSnapshot, 0, binaryPayload, BinaryHeaderSize, stateSnapshot.Length);
            }
            try
            {
                client.Send(binaryPayload, binaryPayload.Length, destination);
                ProcessSyncRequests();
            }
            catch (SocketException)
            {
                Thread.Sleep(2);
            }
            catch (ObjectDisposedException)
            {
                break;
            }
        }
    }

    private static void WriteBinaryHeader(
        byte[] buffer, long packetSequence, long unixTimeNs, double monotonicTimeSec)
    {
        buffer[0] = (byte)'N';
        buffer[1] = (byte)'Q';
        buffer[2] = (byte)'0';
        buffer[3] = (byte)'1';
        int offset = 4;
        WriteInt64(buffer, ref offset, packetSequence);
        WriteInt64(buffer, ref offset, unixTimeNs);
        WriteDouble(buffer, ref offset, monotonicTimeSec);
    }

    private static void WriteStateSnapshot(PicoInputPacket packet, byte[] buffer)
    {
        int offset = 0;
        WriteState(buffer, ref offset, packet.head);
        WriteState(buffer, ref offset, packet.left);
        WriteState(buffer, ref offset, packet.right);
    }

    private static void WriteState(byte[] buffer, ref int offset, PicoControllerState state)
    {
        byte flags = 0;
        if (state.tracked) flags |= 0x01;
        if (state.stickClick) flags |= 0x02;
        if (state.primary) flags |= 0x04;
        if (state.secondary) flags |= 0x08;
        buffer[offset++] = flags;
        buffer[offset++] = 0;
        buffer[offset++] = 0;
        buffer[offset++] = 0;
        WriteFloat(buffer, ref offset, state.px);
        WriteFloat(buffer, ref offset, state.py);
        WriteFloat(buffer, ref offset, state.pz);
        WriteFloat(buffer, ref offset, state.qx);
        WriteFloat(buffer, ref offset, state.qy);
        WriteFloat(buffer, ref offset, state.qz);
        WriteFloat(buffer, ref offset, state.qw);
        WriteFloat(buffer, ref offset, state.vx);
        WriteFloat(buffer, ref offset, state.vy);
        WriteFloat(buffer, ref offset, state.vz);
        WriteFloat(buffer, ref offset, state.avx);
        WriteFloat(buffer, ref offset, state.avy);
        WriteFloat(buffer, ref offset, state.avz);
        WriteFloat(buffer, ref offset, state.trigger);
        WriteFloat(buffer, ref offset, state.grip);
        WriteFloat(buffer, ref offset, state.stickX);
        WriteFloat(buffer, ref offset, state.stickY);
    }

    private static void WriteFloat(byte[] buffer, ref int offset, float value)
    {
        WriteInt32(buffer, ref offset, BitConverter.SingleToInt32Bits(value));
    }

    private static void WriteDouble(byte[] buffer, ref int offset, double value)
    {
        WriteInt64(buffer, ref offset, BitConverter.DoubleToInt64Bits(value));
    }

    private static void WriteInt32(byte[] buffer, ref int offset, int value)
    {
        buffer[offset++] = (byte)value;
        buffer[offset++] = (byte)(value >> 8);
        buffer[offset++] = (byte)(value >> 16);
        buffer[offset++] = (byte)(value >> 24);
    }

    private static void WriteInt64(byte[] buffer, ref int offset, long value)
    {
        for (int index = 0; index < 8; index++)
        {
            buffer[offset++] = (byte)(value >> (8 * index));
        }
    }

    private void ProcessSyncRequests()
    {
        for (int count = 0; count < 8 && client.Available > 0; count++)
        {
            var remote = new IPEndPoint(IPAddress.Any, 0);
            byte[] payload = client.Receive(ref remote);
            long receiveUnixNs = UnixTimeNs();
            long receiveMonotonicNs = MonotonicTimeNs();
            string json = Encoding.UTF8.GetString(payload);
            PicoSyncPing ping = JsonUtility.FromJson<PicoSyncPing>(json);
            if (ping == null || ping.schema != "nero.quest.sync.v1" || ping.kind != "ping")
            {
                continue;
            }

            var pong = new PicoSyncPong
            {
                sequence = ping.sequence,
                hostSendUnixNs = ping.hostSendUnixNs,
                hostSendMonotonicNs = ping.hostSendMonotonicNs,
                questReceiveUnixNs = receiveUnixNs,
                questReceiveMonotonicNs = receiveMonotonicNs,
                questSendUnixNs = UnixTimeNs(),
                questSendMonotonicNs = MonotonicTimeNs(),
            };
            byte[] response = Encoding.UTF8.GetBytes(JsonUtility.ToJson(pong));
            client.Send(response, response.Length, remote);
        }
    }

    private static long UnixTimeNs()
    {
        return (DateTime.UtcNow.Ticks - UnixEpochTicks) * 100L;
    }

    private static long MonotonicTimeNs()
    {
        return System.Diagnostics.Stopwatch.GetTimestamp() * 1_000_000_000L
            / System.Diagnostics.Stopwatch.Frequency;
    }

    private void OnDestroy()
    {
        senderRunning = false;
        senderThread?.Join(500);
        senderThread = null;
        client?.Close();
        client = null;
    }
}
