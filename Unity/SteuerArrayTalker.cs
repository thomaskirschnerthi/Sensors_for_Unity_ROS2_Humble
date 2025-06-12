using UnityEngine;
using ROS2;
using std_msgs.msg;

public class SteuerArrayTalker : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<Float32MultiArray> commandPublisher;

    private Rollstuhl rollstuhlScript;

    // Zum Speichern des vorherigen Zustands
    private int[] letzterZustand = new int[2];

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        rollstuhlScript = GetComponent<Rollstuhl>();

        if (rollstuhlScript == null)
        {
            Debug.LogError("Rollstuhl-Komponente nicht gefunden!");
        }

        // Initialzustand auf ungültige Werte setzen, um beim Start direkt zu senden
        letzterZustand[0] = int.MinValue;
        letzterZustand[1] = int.MinValue;
    }

    void Update()
    {
        if (!ros2Unity.Ok()) return;

        if (ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("steuer_array_talker_node");
            commandPublisher = ros2Node.CreatePublisher<Float32MultiArray>("/controller_commands");
        }

        if (rollstuhlScript != null)
        {
            int[] aktuellerZustand = rollstuhlScript.Steuerbefehle_Unity;

            // Nur senden, wenn sich ein Wert geändert hat
            if (aktuellerZustand[0] != letzterZustand[0] || aktuellerZustand[1] != letzterZustand[1])
            {
                letzterZustand[0] = aktuellerZustand[0];
                letzterZustand[1] = aktuellerZustand[1];

                Float32MultiArray msg = new Float32MultiArray
                {
                    Data = new float[2]
                };

                msg.Data[0] = aktuellerZustand[0];
                msg.Data[1] = aktuellerZustand[1];

                commandPublisher.Publish(msg);

                Debug.Log($"ROS2 Publish: [Bewegung: {msg.Data[0]}, Rotation: {msg.Data[1]}]");
            }
        }
    }
}
