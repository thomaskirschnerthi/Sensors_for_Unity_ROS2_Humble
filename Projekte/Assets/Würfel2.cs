using UnityEngine;

public class Würfel2 : MonoBehaviour
{
    public float distance = 40f;   // Distanz der Bewegung entlang der X-Achse
    public float speed = 5f;       // Geschwindigkeit in Einheiten pro Sekunde

    private Vector3 startPosition;
    private Vector3 targetPosition;
    private bool movingForward = true;

    void Start()
    {
        startPosition = transform.position;
        targetPosition = startPosition + Vector3.right * distance;
    }

    void LateUpdate()
    {
        Vector3 goal = movingForward ? targetPosition : startPosition;

        // Bewegung pro Frame basierend auf Zeit
        transform.position = Vector3.MoveTowards(transform.position, goal, speed * Time.deltaTime);

        // Wenn Ziel erreicht: Richtung umkehren
        if (Vector3.Distance(transform.position, goal) < 0.01f)
        {
            movingForward = !movingForward;
        }
    }
}
