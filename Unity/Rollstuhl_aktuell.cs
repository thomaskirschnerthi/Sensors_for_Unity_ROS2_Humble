using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

public class Rollstuhl : MonoBehaviour
{
    public float Speed;
    public float SpeedRotation;

    private ROS2MultiBoolListener rosListener;

    [Header("TCP Odometry Einstellungen")]
    public string rosIp = "127.0.0.1";
    public int odomPort = 5005;
    public float odomSendRateHz = 10f;

    private float odomSendInterval;
    private float odomTimer;

     // Array für 16 GameObjects zum Einfärben im Editor
    public GameObject[] visualIndicators = new GameObject[16];

    // Neues Array zur Speicherung der Steuerbefehle
    public int[] Steuerbefehle_Unity = new int[2];


    void Start()
    {
        rosListener = GetComponent<ROS2MultiBoolListener>();
        if (rosListener == null)
        {
            Debug.LogError("ROS2MultiBoolListener-Komponente nicht gefunden!");
        }

        odomSendInterval = 1f / odomSendRateHz;
        odomTimer = 0f;
    }

    void Update()
    {
        
        Movement();
        UpdateVisualIndicators();
        UpdateSteuerbefehle();

        odomTimer += Time.deltaTime;
        if (odomTimer >= odomSendInterval)
        {
            SendOdometryToROS();
            odomTimer = 0f;
        }
    }

    void Movement()
    {
        if (rosListener == null) return;

        // Prüfen, ob die Bedingungen für Bewegung / Drehung erfüllt sind

        // Vorwärts: GetShouldRotate0 bis 3 müssen alle true sein
        //  bool canMoveForward = rosListener.GetShouldRotate0() && rosListener.GetShouldRotate1() &&
        //                        rosListener.GetShouldRotate2() && rosListener.GetShouldRotate3();

        // Rückwärts: GetShouldRotate8 bis 11 alle true
        // bool canMoveBackward = rosListener.GetShouldRotate8() && rosListener.GetShouldRotate9() &&
        //                       rosListener.GetShouldRotate10() && rosListener.GetShouldRotate11();

        // Rechts drehen: GetShouldRotate4 bis 7 alle true
        //  bool canRotateRight = rosListener.GetShouldRotate4() && rosListener.GetShouldRotate5() &&
        //                    rosListener.GetShouldRotate6() && rosListener.GetShouldRotate7();

        // Links drehen: GetShouldRotate12 bis 15 alle true
        //  bool canRotateLeft = rosListener.GetShouldRotate12() && rosListener.GetShouldRotate13() &&
        //                 rosListener.GetShouldRotate14() && rosListener.GetShouldRotate15();

        // Diese Booleans dienen nur für den Lösungsfindungsprozess 
        bool canMoveForward = true;
        bool canMoveBackward = true;
        bool canRotateLeft = true;
        bool canRotateRight = true;

        if (canMoveForward && Input.GetAxis("Vertical") > 0)
        {
            Vector3 movement = Vector3.back * Input.GetAxis("Vertical") * Speed * Time.deltaTime;
            transform.Translate(movement);
        }

        if (canMoveBackward && Input.GetAxis("Vertical") < 0)
        {
            Vector3 movement = Vector3.back * Input.GetAxis("Vertical") * Speed * Time.deltaTime;
            transform.Translate(movement);
        }

        if (canRotateRight && Input.GetAxis("Horizontal") > 0)
        {
            if (Input.GetAxis("Vertical") < 0)
                transform.Rotate(Vector3.down, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
            else
                transform.Rotate(Vector3.up, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
        }

        if (canRotateLeft && Input.GetAxis("Horizontal") < 0)
        {
            if (Input.GetAxis("Vertical") < 0)
                transform.Rotate(Vector3.down, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
            else
                transform.Rotate(Vector3.up, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
        }

    }

    void UpdateSteuerbefehle()
{
    // Bewegung (Pfeil nach vorne/hinten)
    if (Input.GetKey(KeyCode.UpArrow))
    {
        Steuerbefehle_Unity[0] = 1;
    }
    else if (Input.GetKey(KeyCode.DownArrow))
    {
        Steuerbefehle_Unity[0] = -1;
    }
    else
    {
        Steuerbefehle_Unity[0] = 0;
    }

    // Rotation (Pfeil rechts/links)
    if (Input.GetKey(KeyCode.RightArrow))
    {
        Steuerbefehle_Unity[1] = 1;
    }
    else if (Input.GetKey(KeyCode.LeftArrow))
    {
        Steuerbefehle_Unity[1] = -1;
    }
    else
    {
        Steuerbefehle_Unity[1] = 0;
    }
}


void UpdateVisualIndicators()
    {
        if (rosListener == null) return;

        bool[] values = new bool[16]
        {
            rosListener.GetShouldRotate0(),
            rosListener.GetShouldRotate1(),
            rosListener.GetShouldRotate2(),
            rosListener.GetShouldRotate3(),
            rosListener.GetShouldRotate4(),
            rosListener.GetShouldRotate5(),
            rosListener.GetShouldRotate6(),
            rosListener.GetShouldRotate7(),
            rosListener.GetShouldRotate8(),
            rosListener.GetShouldRotate9(),
            rosListener.GetShouldRotate10(),
            rosListener.GetShouldRotate11(),
            rosListener.GetShouldRotate12(),
            rosListener.GetShouldRotate13(),
            rosListener.GetShouldRotate14(),
            rosListener.GetShouldRotate15()
        };

        for (int i = 0; i < visualIndicators.Length && i < values.Length; i++)
        {
            if (visualIndicators[i] != null)
            {
                Renderer renderer = visualIndicators[i].GetComponent<Renderer>();
                if (renderer != null)
                {   // grün bei Wert false -> kein Hindernis in Sektor
                    renderer.material.color = values[i] ? Color.red : Color.green;
                }
            }
        }
    }

    void SendOdometryToROS()
    {
        OdometryData odom = new OdometryData
        {
            x = transform.position.x,
            y = transform.position.y,
            z = transform.position.z,
            qx = transform.rotation.x,
            qy = transform.rotation.y,
            qz = transform.rotation.z,
            qw = transform.rotation.w
        };

        string json = JsonUtility.ToJson(odom);
        byte[] data = Encoding.UTF8.GetBytes(json);
        SendTcpAsync(rosIp, odomPort, data);
    }

    void SendTcpAsync(string ip, int port, byte[] data)
    {
        _ = Task.Run(() =>
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
//                Debug.LogWarning($"TCP-Verbindung fehlgeschlagen: {ex.Message}");
            }
        });
    }

    [System.Serializable]
    public class OdometryData
    {
        public float x, y, z;
        public float qx, qy, qz, qw;
    }

    public void OnTriggerEnter(Collider other)
    {
        Debug.Log("Zusammenstoß mit Objekt");
    }
}
