using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum AxisState
{
    FREE = 0,
    LOCK = 1,
    CLAMP = 2,
}


public class JointContraint : MonoBehaviour
{
    public AxisState xState;
    public AxisState yState;
    public AxisState zState;


    [Range(0, 180)]  public float xPositifAngle;
    [Range(0, -180)] public float xNegatifAngle;
    [Range(0, 180)] public float yPositifAngle;
    [Range(0, -180)] public float yNegatifAngle;
    [Range(0, 180)] public float zPositifAngle;
    [Range(0, -180)] public float zNegatifAngle;
}
