using System;
using UnityEngine;
using ROS2;
using std_msgs.msg;

public class FahrbefehlAnzeigen : MonoBehaviour
{
    // ROS-Komponenten
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<Float32MultiArray> fahrbefehlSubscription;

    // Interne Variablen für die beiden Werte
    private float bewegung = 0f; // Index 0 → vor/zurück
    private float rotation = 0f; // Index 1 → rechts/links

    // Anzeigen im Inspector zuweisen
    [Header("Steueranzeigen (Ziehen im Inspector)")]
    public GameObject AnzeigeVorwärts;
    public GameObject AnzeigeRückwärts;
    public GameObject AnzeigeRechts;
    public GameObject AnzeigeLinks;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("ROS2UnityComponent fehlt!");
        }
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity != null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("fahrbefehl_listener_node");

            fahrbefehlSubscription = ros2Node.CreateSubscription<Float32MultiArray>(
                "/freigegebener_fahrbefehl",
                msg =>
                {
                    if (msg.Data.Length >= 2)
                    {
                    bewegung = msg.Data[0];
                    rotation = msg.Data[1];
                    Debug.Log($"[ROS] Fahrbefehl empfangen: Bewegung = {bewegung}, Rotation = {rotation}");
                    }
                    else
                    {
                    Debug.LogWarning("Empfangene Float32MultiArray hat weniger als 2 Werte.");
                    }

                });
        }

        UpdateAnzeigen();
    }

    void UpdateAnzeigen()
    {
        // Anzeige Vorwärts (bewegung > 0)
        if (AnzeigeVorwärts != null)
        {
            SetAnzeigenFarbe(AnzeigeVorwärts, bewegung > 0f);
        }

        // Anzeige Rückwärts (bewegung < 0)
        if (AnzeigeRückwärts != null)
        {
            SetAnzeigenFarbe(AnzeigeRückwärts, bewegung < 0f);
        }

        // Anzeige Rechts (rotation > 0)
        if (AnzeigeRechts != null)
        {
            SetAnzeigenFarbe(AnzeigeRechts, rotation > 0f);
        }

        // Anzeige Links (rotation < 0)
        if (AnzeigeLinks != null)
        {
            SetAnzeigenFarbe(AnzeigeLinks, rotation < 0f);
        }
    }

    void SetAnzeigenFarbe(GameObject anzeige, bool aktiv)
    {
        Renderer renderer = anzeige.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = aktiv ? Color.green : Color.red;
        }
    }
}
