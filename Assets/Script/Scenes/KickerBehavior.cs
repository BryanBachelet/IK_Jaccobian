using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KickerBehavior : MonoBehaviour
{
    [Header("Kicker Variables ")]
    public float kickStrength = 10;
    public Jacobian_IK jacobianComponent;
    public BoxCollider colliderBox;
    private Vector3 m_lastDirection;

    public void OnTriggerEnter(Collider other)
    {
        if (other.tag != "Ball" || !jacobianComponent.HasContextualTarget) return;

        Vector3 direction = transform.forward;
        m_lastDirection = direction;
        Rigidbody ballRigid = other.GetComponent<Rigidbody>();
        ballRigid.AddForce(direction.normalized * kickStrength, ForceMode.Impulse);

        jacobianComponent.RemoveContextualTarget();

        Debug.Log("teST");
    }

    public void OnDrawGizmosSelected()
    {
        Gizmos.DrawRay((transform.position), m_lastDirection.normalized * 5);
    }
}
