using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum AxisState
{
    FREE = 0,
    LOCK = 1,
    CLAMP = 2,
}

public enum Axis
{
    X = 0,
    Y = 1,
    Z = 2,
}

[System.Serializable]
public class AxisContraintTEst
{
    public AxisState state;
    public double angleStart;
    public double storeAngle;
    public double totalAngle;
    public bool bIsReverse;
    [Range(0, 180)] public float PositifAngle;
    [Range(0, -180)] public float NegatifAngle;
}


public class JointContraint : MonoBehaviour
{

    public AxisContraintTEst XAxisContraint;
    public AxisContraintTEst YAxisContraint;
    public AxisContraintTEst ZAxisContraint;


    private JointContraint parentJoint;
    private bool hasParent;

    public void Awake()
    {

        if (transform.parent != null)
        {
            parentJoint = transform.parent.GetComponent<JointContraint>();
           if(parentJoint != null) hasParent = true;
        }

    }

    public double ClampJoint(double baseAngle, double angleDelta, Axis axisFlag)
    {


        AxisContraintTEst axisContraint = XAxisContraint;


        AxisContraintTEst axisParentContraint = new AxisContraintTEst();
       if(hasParent) axisParentContraint = parentJoint.XAxisContraint;

        switch (axisFlag)
        {
            case Axis.X:
                axisContraint = XAxisContraint;
             if(hasParent)   axisParentContraint = parentJoint.XAxisContraint;
                break;
            case Axis.Y:
                axisContraint = YAxisContraint;
                if (hasParent) axisParentContraint = parentJoint.YAxisContraint;
                break;
            case Axis.Z:
                axisContraint = ZAxisContraint;
                if (hasParent) axisParentContraint = parentJoint.ZAxisContraint;
                break;
            default:
                axisContraint = XAxisContraint;
                if (hasParent) axisParentContraint = parentJoint.XAxisContraint;
                break;
        }

        double parentAngle = 0;
        if (hasParent) parentAngle = axisParentContraint.storeAngle;
        
        double clampAngle = baseAngle + parentAngle % 360;

        int sign =  (-270 < clampAngle && clampAngle< -90) ? 1 : -1;


        if (axisContraint.state == AxisState.LOCK)
        {
            axisContraint.totalAngle = clampAngle;
            axisContraint.storeAngle = baseAngle;
            return baseAngle;
        }
        if (axisContraint.bIsReverse)
        {
            baseAngle += sign * angleDelta;
        }
        else
        {
            baseAngle += angleDelta;
        }

        double angleTest = baseAngle;
        if (axisContraint.state == AxisState.CLAMP)
        {
            angleTest = Math.Clamp(baseAngle, axisContraint.angleStart + axisContraint.NegatifAngle, axisContraint.angleStart + axisContraint.PositifAngle);
        }
        else if(axisContraint.state == AxisState.FREE)
        {
            angleTest = angleTest % 360;
            
        }

        
        axisContraint.storeAngle = angleTest;
        axisContraint.totalAngle = clampAngle;
        return angleTest;

    }


}
