using UnityEngine;
using System.Net.Sockets;
using System.Net;

public class KinectRaycastSensor : MonoBehaviour
{
    [Header("Kameraeinstellungen")]
    public Camera sensorCamera;

    [Header("Auflösung")]
    public int width = 512;
    public int height = 424;

    [Header("ROS TCP-Ziel")]
    public string rosIp = "127.0.0.1";
    public int depthPort = 5007;

    private float[] depthData;

    void Start()
    {
        if (sensorCamera == null)
        {
            sensorCamera = Camera.main;
        }

        sensorCamera.enabled = false; // Kamera nicht rendern
        depthData = new float[width * height];
    }

    void Update()
    {
        CaptureDepthWithRaycast();
        SendDepthData(rosIp, depthPort);
    }

    void CaptureDepthWithRaycast()
    {
        for (int y = 0; y < height; y++)
        {
            float v = (float)y / (height - 1);

            for (int x = 0; x < width; x++)
            {
                float u = (float)x / (width - 1);

                Ray ray = sensorCamera.ViewportPointToRay(new Vector3(u, v, 0));
                if (Physics.Raycast(ray, out RaycastHit hit, 10f)) // Max. Entfernung z. B. 10 m
                {
                    depthData[y * width + x] = hit.distance;
                }
                else
                {
                    depthData[y * width + x] = 0f; // Kein Treffer = 0
                }
            }
        }
    }

    void SendDepthData(string ip, int port)
    {
        byte[] bytes = new byte[depthData.Length * sizeof(float)];
        System.Buffer.BlockCopy(depthData, 0, bytes, 0, bytes.Length);

        try
        {
            using (TcpClient client = new TcpClient(ip, port))
            using (NetworkStream stream = client.GetStream())
            {
                stream.Write(bytes, 0, bytes.Length);
            }
        }
        catch (SocketException ex)
        {
            Debug.LogWarning($"TCP-Verbindung fehlgeschlagen: {ex.Message}");
        }
    }
}
