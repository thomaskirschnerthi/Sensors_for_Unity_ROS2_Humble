using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Globalization;

/// <summary>
/// Empfängt LiDAR-Einzelmessungen und sendet sie bei jeder neuen Messung als Klartext-String an ROS.
/// Zeigt in der Unity-Konsole, welche Daten gesendet werden.
/// </summary>
public class RosLidarPublisher : MonoBehaviour
{
    public int raysPerRotation = 360;
    public string rosIp = "127.0.0.1";     // IP des ROS-Rechners
    public int rosPort = 5005;             // Port muss mit ROS-Empfangsskript übereinstimmen

    public Lidar2DSensorROS lidar;  // Per Inspector zuweisen
    private UdpClient udpClient;

    void Start()
{
    if (lidar == null)
    {
        lidar = GetComponent<Lidar2DSensorROS>();
        if (lidar == null)
        {
            Debug.LogError("Kein Lidar2DSensorROS gefunden oder zugewiesen.");
            return;
        }
    }

    lidar.OnMeasurement += HandleRay;
    Debug.Log("Event-Handler erfolgreich registriert.");

    udpClient = new UdpClient();
}

        
    

    // Hier wird jeder neue Messwert direkt gesendet
    void HandleRay(float angleDeg, float range)
    {
        Debug.Log($"Empfangene Messung: Winkel={angleDeg}, Reichweite={range}");
        SendMeasurementAsText(angleDeg, range);
    }

    // Sendet die Messung sofort als Text-String über UDP an ROS
    void SendMeasurementAsText(float angleDeg, float range)
    {
        // Formatiert die Messung als Text-String, z. B. "0.45, 2.3"
        string textData = string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", angleDeg, range);
        byte[] bytes = Encoding.UTF8.GetBytes(textData + "\n");  // \n für den Abschluss der Zeile

        udpClient.Send(bytes, bytes.Length, rosIp, rosPort);

        // In der Unity-Konsole anzeigen, welche Daten gesendet werden
        Debug.Log($"[ROS-Text] Einzelne Messung (Text) gesendet: {textData}");
    }

    void OnDestroy()
    {
        if (lidar != null)
        {
            lidar.OnMeasurement -= HandleRay;
        }

        udpClient?.Close();
    }
}
