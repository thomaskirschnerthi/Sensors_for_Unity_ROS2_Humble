using UnityEngine;

public class Rollstuhl : MonoBehaviour
{
    public float Speed;
    public float SpeedRotation;

    private ROS2MultiBoolListener rosListener;

    void Start()
    {
        rosListener = GetComponent<ROS2MultiBoolListener>();
        if (rosListener == null)
        {
            Debug.LogError("ROS2MultiBoolListener-Komponente nicht gefunden!");
        }
    }

    void Update()
    {
        Movement();
    }

    void Movement()
    {
        if (rosListener == null) return;

        // Prüfen, ob die Bedingungen für Bewegung / Drehung erfüllt sind

        // Vorwärts: GetShouldRotate0 bis 3 müssen alle true sein
        bool canMoveForward = rosListener.GetShouldRotate0() && rosListener.GetShouldRotate1() &&
                              rosListener.GetShouldRotate2() && rosListener.GetShouldRotate3();

        // Rückwärts: GetShouldRotate8 bis 11 alle true
        bool canMoveBackward = rosListener.GetShouldRotate8() && rosListener.GetShouldRotate9() &&
                               rosListener.GetShouldRotate10() && rosListener.GetShouldRotate11();

        // Rechts drehen: GetShouldRotate4 bis 7 alle true
        bool canRotateRight = rosListener.GetShouldRotate4() && rosListener.GetShouldRotate5() &&
                              rosListener.GetShouldRotate6() && rosListener.GetShouldRotate7();

        // Links drehen: GetShouldRotate12 bis 15 alle true
        bool canRotateLeft = rosListener.GetShouldRotate12() && rosListener.GetShouldRotate13() &&
                             rosListener.GetShouldRotate14() && rosListener.GetShouldRotate15();


        // Bewegung nach vorne (Input.GetAxis("Vertical") > 0)
        if (canMoveForward && Input.GetAxis("Vertical") > 0)
        {
            Vector3 movement = Vector3.forward * Input.GetAxis("Vertical") * Speed * Time.deltaTime;
            transform.Translate(movement);
        }

        // Bewegung nach hinten (Input.GetAxis("Vertical") < 0)
        if (canMoveBackward && Input.GetAxis("Vertical") < 0)
        {
            Vector3 movement = Vector3.forward * Input.GetAxis("Vertical") * Speed * Time.deltaTime;
            transform.Translate(movement);
        }

        // Drehung nach rechts (Input.GetAxis("Horizontal") > 0)
        if (canRotateRight && Input.GetAxis("Horizontal") > 0)
        {
            // Wenn Rückwärts (Vertical < 0), drehe andersrum
            if (Input.GetAxis("Vertical") < 0)
                transform.Rotate(Vector3.down, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
            else
                transform.Rotate(Vector3.up, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
        }

        // Drehung nach links (Input.GetAxis("Horizontal") < 0)
        if (canRotateLeft && Input.GetAxis("Horizontal") < 0)
        {
            // Wenn Rückwärts (Vertical < 0), drehe andersrum
            if (Input.GetAxis("Vertical") < 0)
                transform.Rotate(Vector3.down, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
            else
                transform.Rotate(Vector3.up, Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime);
        }
    }

    public void OnTriggerEnter(Collider other)
    {
        Debug.Log("Zusammenstoß mit Objekt");
    }
}
