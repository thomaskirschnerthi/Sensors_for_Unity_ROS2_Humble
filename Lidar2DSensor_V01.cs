using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Globalization;
using System.Collections;

/// <summary>
/// Simuliert kontinuierliche 2D-LiDAR-Daten (XZ-Ebene, Rotation um lokale Y-Achse).
/// Sendet bei jedem Winkel eine UDP-Nachricht im Format: "Winkel,Reichweite\n"
/// </summary>
public class Lidar2DSensorROS : MonoBehaviour
{
    [Header("Sensor-Einstellungen")]
    public float rotationSpeed = 3600f;
    public float angleStep = 0.72f;
    public float maxRange = 10f;
    public LayerMask detectionLayers;

    public delegate void OnRayMeasured(float angleDeg, float range);
    public event OnRayMeasured OnMeasurement;

    private float accumulatedRotation = 0f;
    private float currentScanAngle = 0f;
    private UdpClient udpClient;

    void Start()
    {
        udpClient = new UdpClient();
        udpClient.Connect("127.0.0.1", 5005); // IP des ROS-PCs
        //Application.targetFrameRate = 240;

        StartCoroutine(LidarLoop()); 
    }

    private IEnumerator LidarLoop()
{
    float angleStepLocal = angleStep;  // sicherstellen, dass Zwischenschritte exakt laufen
    float scanAngle = 0f;

    WaitForSecondsRealtime wait = new WaitForSecondsRealtime(0.0002f); // 5000 Hz

    while (true)
    {
        MeasureAndSend(scanAngle);
        scanAngle += angleStepLocal;
        if (scanAngle >= 360f) scanAngle -= 360f;

        yield return wait;
    }
}



    void Update()
    {
  
    }



    void MeasureAndSend(float angleDeg)
    {
        Vector3 origin = transform.position;
        Quaternion localRotation = Quaternion.Euler(0f, angleDeg, 0f);
        Vector3 localDirection = localRotation * Vector3.forward;
        Vector3 worldDirection = transform.TransformDirection(localDirection);

        RaycastHit hit;
        float range = maxRange;
        if (Physics.Raycast(origin, worldDirection, out hit, maxRange, detectionLayers))
        {
            range = hit.distance;
        }

        string data = string.Format(CultureInfo.InvariantCulture, "{0:F1},{1:F3}\n", angleDeg, range);
        byte[] bytes = Encoding.ASCII.GetBytes(data);
        udpClient.Send(bytes, bytes.Length);

        Debug.DrawLine(origin, origin + worldDirection * range, Color.green, 0.05f);

        // Hier wird das Event ausgelöst
        OnMeasurement?.Invoke(angleDeg, range);
    }

    void OnApplicationQuit()
    {
        udpClient.Close();
    }
}
