using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;


/// <summary>
/// Simuliert kontinuierliche 2D-LiDAR-Daten (XZ-Ebene, Rotation um lokale Y-Achse).
/// Sendet bei 360* eine UDP-Nachricht im Format: "Winkel,Reichweite\n"
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
    private List<string> batchData = new List<string>();

    void Start()
    {
        udpClient = new UdpClient();
        udpClient.Connect("127.0.0.1", 5005); // IP des ROS-PCs
        //Application.targetFrameRate = 240;

        //StartCoroutine(LidarLoop());         das wäre der alternativweg
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
  	//float deltaRotation = rotationSpeed * Time.deltaTime;
  	float deltaRotation = 5000 * 0.72f * Time.deltaTime;  // 5000 Hz * 0,72°
        Debug.Log($"Frame Time: {Time.deltaTime:F4} s");
        accumulatedRotation += deltaRotation;

        while (accumulatedRotation >= angleStep)
        {
            MeasureAndSend(currentScanAngle);
            currentScanAngle += angleStep;
            if (currentScanAngle >= 360f) currentScanAngle -= 360f;
            accumulatedRotation -= angleStep;
        }
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

    	// Statt direkter UDP-Sendung:
    	string data = $"{angleDeg:F1},{range:F3}";
        Debug.Log($"Gesendeter Winkel: {angleDeg:F2}°");
    	batchData.Add(data);

    	// Bei vollständiger Umdrehung senden
    	if (angleDeg >= 360f - angleStep)
    	{
        	string combinedData = string.Join("\n", batchData) + "\n";
        	byte[] bytes = Encoding.ASCII.GetBytes(combinedData);
            
            IPEndPoint rosEndpoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 5005);
            udpClient.Send(bytes, bytes.Length, rosEndpoint);
        	
            
            //udpClient.Send(bytes, bytes.Length);
        	batchData.Clear();
    	}

        Debug.DrawLine(origin, origin + worldDirection * range, Color.green, 0.05f);

        // Hier wird das Event ausgelöst
        OnMeasurement?.Invoke(angleDeg, range);
    }

    void OnApplicationQuit()
    {
        udpClient.Close();
    }
}
