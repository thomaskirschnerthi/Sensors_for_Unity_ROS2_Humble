using UnityEngine;
using System.Net.Sockets;
using System.Net;

public class KinectSensor : MonoBehaviour
{
    [Header("Kameraeinstellungen")]
    public Camera depthCamera;
    public Camera rgbCamera;

    [Header("Material mit Tiefenshader")]
    public Material depthMaterial;

    [Header("Auflösung")]
    public int width = 512;
    public int height = 424;

    [Header("ROS TCP-Ziel")]
    public string rosIp = "127.0.0.1";
    public int depthPort = 5007;
    public int rgbPort = 5008;

    private RenderTexture depthTexture;
    private RenderTexture rgbTexture;

    private Texture2D depthReadTex;
    private Texture2D rgbReadTex;

    void Start()
    {
        // RenderTextures erzeugen
        depthTexture = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);
        depthTexture.Create();

        rgbTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        rgbTexture.Create();

        // Kameras konfigurieren
        depthCamera.targetTexture = depthTexture;
        rgbCamera.targetTexture = rgbTexture;

        depthCamera.SetReplacementShader(depthMaterial.shader, "RenderType");

        // Texturen für Datenextraktion
        depthReadTex = new Texture2D(width, height, TextureFormat.RFloat, false);
        rgbReadTex = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void Update()
    {
        SendImage(depthTexture, depthReadTex, rosIp, depthPort);
        SendImage(rgbTexture, rgbReadTex, rosIp, rgbPort);
    }

    void SendImage(RenderTexture rt, Texture2D tex, string ip, int port)
    {
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();

        byte[] data = tex.GetRawTextureData();

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
            Debug.LogWarning($"TCP-Verbindung fehlgeschlagen: {ex.Message}");
        }
    }
}
