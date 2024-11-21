using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using IK.Tools;


public class GenericJacobian : MonoBehaviour
{
    public static JacobianMatrix CreateGenericJacobian(int jointCount, Transform[] joints)
    {
        JacobianMatrix jacobianMatrix = new JacobianMatrix(6, 6 * jointCount);

        // Create Adjoint Block
        for (int i = 0; i < jointCount; i++)
        {
            AdjointBlock adjointBlock = new AdjointBlock();
            Matrix4x4 rotationJoint = Matrix4x4.Rotate(joints[i].transform.rotation);
            // Add the the two rotation bock
            adjointBlock.AddBlock(rotationJoint, 3, 3, 0, 0);
            adjointBlock.AddBlock(rotationJoint, 3, 3, 3, 3);

            // Calcul the cross rotation block and add it to the adjointBlock
            Matrix4x4 crossTranslation = Matrix4x4.zero;
            crossTranslation[0, 1] = -joints[i].transform.position.z;
            crossTranslation[0, 2] = joints[i].transform.position.y;
            crossTranslation[1, 2] = -joints[i].transform.position.x;

            crossTranslation[1, 0] = joints[i].transform.position.z;
            crossTranslation[2, 0] = -joints[i].transform.position.y;
            crossTranslation[2, 1] = joints[i].transform.position.x;

            Matrix4x4 crossRotation = crossTranslation * rotationJoint;
            adjointBlock.AddBlock(crossRotation, 3, 3, 0, 3);

            //adjointBlock.DebugMatrix();
            // Add the adjoin block to the jaccobian matrix
            jacobianMatrix.AddData(adjointBlock.values, adjointBlock.rows, adjointBlock.column, i * adjointBlock.column);
        }
        jacobianMatrix.Transpose();
        return jacobianMatrix;
    }
    
    /// <summary>
    /// Calcul the delta values of orientation and position
    /// </summary>
    /// <param name="linearVelocity"></param>
    /// <param name="angularVelocity"></param>
    /// <param name="jacobianMatrix"></param>
    /// <returns></returns>
    public static float[] GetDeltaValues(Vector3 linearVelocity, Vector3 angularVelocity, JacobianMatrix jacobianMatrix)
    {
        Vector6 velocity = new Vector6(linearVelocity, angularVelocity);

        return jacobianMatrix * velocity;
    }
}
