using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CamCloserBehavior : MonoBehaviour
{
    [SerializeField] private bool m_deactivateFeature;
    [SerializeField] private Jacobian_IK m_jaccobianIKSolver;
    [SerializeField] private Camera m_cameraClose;
    private bool hasBeenDeactivate;

    #region Unity Functions
    // Start is called before the first frame update
    void Start()
    {
        m_cameraClose.enabled = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (m_deactivateFeature)
        {
            m_cameraClose.enabled = false;
            return;
        }

        if (m_jaccobianIKSolver.HasContextualTarget)
        {
            if(m_jaccobianIKSolver.hasContextualTargetClose)
            {
                m_cameraClose.enabled = true;
                hasBeenDeactivate = false;

            }
           
        }
        else
        {
            if (hasBeenDeactivate) return;
            hasBeenDeactivate = true; 
            StartCoroutine(ActiveCamera(false, 0.5f));
        }
    }

    #endregion

    IEnumerator ActiveCamera(bool isActivate, float seconds)
    {

        yield return new WaitForSeconds(seconds);
        m_cameraClose.enabled = isActivate;
    }
}
