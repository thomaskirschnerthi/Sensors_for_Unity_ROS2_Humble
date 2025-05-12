using UnityEngine;
using System.Net.Sockets;

public class KinectRaycastSensorFreauenzy : MonoBehaviour
{
    [Header("Kameraeinstellungen")]
    public Camera sensorCamera;

    [Header("Auflösung")]
    public int width = 512;
    public int height = 424;

    [Header("ROS TCP-Ziel")]
    public string rosIp = "127.0.0.1";
    public int depthPort = 5007;
    public int rgbPort = 5008;

    [Header("Abtastfrequenz")]
    [Tooltip("Abtastfrequenz in Hertz (z. B. 10 = 10 Abtastungen/Sekunde)")]
    public float abtastfrequenzHz = 10f;

    private float[] depthData;
    private byte[] rgbData;

    private float nextSampleTime = 0f;

    void Start()
    {
        if (sensorCamera == null)
        {
            sensorCamera = Camera.main;
        }

        sensorCamera.enabled = false;
        depthData = new float[width * height];
        rgbData = new byte[width * height * 3];

        nextSampleTime = Time.time;
    }

    void Update()
    {
        if (Time.time >= nextSampleTime)
        {
            nextSampleTime = Time.time + (1f / abtastfrequenzHz);

            CaptureDepthAndColorWithRaycast();
            SendDepthData(rosIp, depthPort);
            SendRgbData(rosIp, rgbPort);
        }
    }

    void CaptureDepthAndColorWithRaycast()
    {
        for (int y = 0; y < height; y++)
        {
            float v = (float)y / (height - 1);
            for (int x = 0; x < width; x++)
            {
                float u = (float)x / (width - 1);
                int mirroredX = width - x - 1;
                int index = y * width + mirroredX;

                Ray ray = sensorCamera.ViewportPointToRay(new Vector3(u, v, 0));
                if (Physics.Raycast(ray, out RaycastHit hit, 10f))
                {
                    depthData[index] = hit.distance;

                    Color color = Color.black;
                    Renderer renderer = hit.collider.GetComponent<Renderer>();
                    if (renderer != null && renderer.material != null)
                    {
                        color = renderer.material.color;
                    }

                    rgbData[index * 3 + 0] = (byte)(Mathf.Clamp01(color.r) * 255f);
                    rgbData[index * 3 + 1] = (byte)(Mathf.Clamp01(color.g) * 255f);
                    rgbData[index * 3 + 2] = (byte)(Mathf.Clamp01(color.b) * 255f);
                }
                else
                {
                    depthData[index] = 0f;
                    rgbData[index * 3 + 0] = 0;
                    rgbData[index * 3 + 1] = 0;
                    rgbData[index * 3 + 2] = 0;
                }
            }
        }
    }

    void SendDepthData(string ip, int port)
    {
        byte[] bytes = new byte[depthData.Length * sizeof(float)];
        System.Buffer.BlockCopy(depthData, 0, bytes, 0, bytes.Length);
        SendTcp(ip, port, bytes);
    }

    void SendRgbData(string ip, int port)
    {
        SendTcp(ip, port, rgbData);
    }

    void SendTcp(string ip, int port, byte[] data)
    {
        try
        {
            using (TcpClient client = new TcpClient(ip, port))
            using (NetworkStream stream = client.GetStream())
            {
                stream.Write(data, 0, data.Length);
            }
        }
        catch (SocketException ex)
        {
            Debug.LogWarning($"TCP-Verbindung fehlgeschlagen (Port {port}): {ex.Message}");
        }
    }
}
