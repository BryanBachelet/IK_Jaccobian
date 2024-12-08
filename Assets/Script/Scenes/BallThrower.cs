using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallThrower : MonoBehaviour
{
    public Jacobian_IK jaccobianComponent;

    [Header("Ball Variables")]
    public GameObject ballGo;
    public float strenghOfLaunch;
    public float angleOfThrow;

    [Header("Thrower Variables")]
    public float frequencyOfLaunch;
    private float timerOfLaunch;
    public Vector3 spawnPoint;


    [Header("Debug Variables")]
    [SerializeField] private bool m_isDebugActive = true;
    [SerializeField] private float m_lengthOfDebugVector = 3.0f;
    [SerializeField] private float m_radiusOfSpawnPoint = 0.4f;


    // Update is called once per frame
    void Update()
    {
        if (timerOfLaunch > frequencyOfLaunch)
        {
            ThrowBall();
            timerOfLaunch = 0.0f;
        }
        else
        {
            timerOfLaunch += Time.deltaTime;
        }
    }

    public void ThrowBall()
    {
        GameObject instance = GameObject.Instantiate(ballGo, transform.position + spawnPoint, Quaternion.identity);
        Rigidbody rigidInstance = instance.GetComponent<Rigidbody>();
        rigidInstance.AddForce(Quaternion.Euler(angleOfThrow, 0, 0) * Vector3.forward * strenghOfLaunch, ForceMode.Impulse);
        if(jaccobianComponent != null)
        {
            jaccobianComponent.AddContextualTarget(instance.transform);
        }
    }

    public void OnDrawGizmosSelected()
    {   
        if (!m_isDebugActive) return;

        Gizmos.DrawRay(transform.position + spawnPoint, Quaternion.Euler(angleOfThrow, 0, 0) * Vector3.forward * m_lengthOfDebugVector);
        Gizmos.DrawSphere(transform.position + spawnPoint, m_radiusOfSpawnPoint);
    }
}
