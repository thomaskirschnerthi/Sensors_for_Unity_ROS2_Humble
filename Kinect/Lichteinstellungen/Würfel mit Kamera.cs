using UnityEngine;
using System.Collections;

public class WürfelmitKamera : MonoBehaviour
{
    public float moveDistance = 20f;           // Wie weit sich der Würfel bewegt
    public float moveDuration = 5f;            // Wie lange die Bewegung dauert
    public float updatesPerSecond = 30f;       // Wie oft die Position pro Sekunde aktualisiert wird

    private Vector3 startPos;
    private Vector3 targetPos;
    private bool movingRight = true;

    void Start()
    {
        startPos = transform.position;
        targetPos = startPos + Vector3.right * moveDistance;

        // Starte die Bewegungsschleife
        StartCoroutine(MoveLoop());
    }

    IEnumerator MoveLoop()
    {
        while (true)
        {
            Vector3 from = movingRight ? startPos : targetPos;
            Vector3 to = movingRight ? targetPos : startPos;
            float elapsed = 0f;

            // Berechne Intervall zwischen den Positionsupdates
            float updateInterval = 1f / updatesPerSecond;

            while (elapsed < moveDuration)
            {
                elapsed += updateInterval;
                float t = Mathf.Clamp01(elapsed / moveDuration);
                transform.position = Vector3.Lerp(from, to, t);

                yield return new WaitForSeconds(updateInterval);
            }

            movingRight = !movingRight;
        }
    }
}
