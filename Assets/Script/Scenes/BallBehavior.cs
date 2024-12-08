using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallBehavior : MonoBehaviour
{
    [Header("Ball Variables")]
    public float lifeTime;
    private float m_lifeTimer;


    // Update is called once per frame
    void Update()
    {

        if (m_lifeTimer > lifeTime)
        {
            Destroy(this.gameObject);
        }
        else
        {
            m_lifeTimer += Time.deltaTime;
        }
    }


}
