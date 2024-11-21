using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestMat : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
           Matrix4x4 mat = Matrix4x4.TRS(transform.localPosition, transform.localRotation, transform.localScale);
        transform.position = mat.inverse.MultiplyPoint( transform.position);
    }

    // Update is called once per frame
    void Update()
    {
        Rigidbody rigidbody;
    }
}
