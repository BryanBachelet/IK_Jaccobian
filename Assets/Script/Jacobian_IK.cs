using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using IK.Tools;


public class Jacobian_IK : MonoBehaviour
{
    public int iterationCount = 20;
    
    private Transform m_contextualTargetTransform;
    public bool HasContextualTarget;
    public bool hasContextualTargetClose;
    public Transform generalTargetTransform;
    private Transform m_currentTargeTransform;


    [Header("Contextual Target Variable")]
    public bool isTargetVariationActive = true;
    public float distanceToDetect = 5;

    public Transform[] joints = new Transform[3];
    public Transform endEffectorTransform;

    public float maxDistance;
    public float m_minDistance = .2f;
    private const int m_minAngleOrientation = 10;

    private float m_iterations;

    private float[] m_anglesXArray;
    private float[] m_anglesYArray;
    private float[] m_anglesZArray;

    private float[] m_baseAnglesX;
    private float[] m_baseAnglesY;
    private float[] m_baseAnglesZ;

    /// <summary>
    /// Active the genenic jaccobian algorithm.
    /// </summary>
    public bool isGenericJacobianActive;
    [SerializeField] private bool isDebugActive;

    #region Unity Functions
    // Start is called before the first frame update


    void Start()
    {
        m_currentTargeTransform = generalTargetTransform;

        // Creation of array for angles delta for each joint
        m_anglesXArray = new float[joints.Length];
        m_anglesYArray = new float[joints.Length];
        m_anglesZArray = new float[joints.Length];

        // Creation of array for base angles of each joint
        m_baseAnglesX = new float[joints.Length];
        m_baseAnglesY = new float[joints.Length];
        m_baseAnglesZ = new float[joints.Length];

        // Make sure to have min distance in the joint
        maxDistance += 0.5f;


        // Register base angle the joint and store the total distance between jointss
        for (int i = 0; i < joints.Length; i++)
        {

            if (i < joints.Length - 1) maxDistance += Vector3.Distance(joints[i].transform.position, joints[i + 1].transform.position);
            if (i == joints.Length - 1) maxDistance += Vector3.Distance(joints[i].transform.position, endEffectorTransform.transform.position);

            m_anglesXArray[i] = joints[i].localRotation.eulerAngles.x;
            m_anglesYArray[i] = joints[i].localRotation.eulerAngles.y;
            m_anglesZArray[i] = joints[i].localRotation.eulerAngles.z;

            m_baseAnglesX[i] = joints[i].localRotation.eulerAngles.x;
            m_baseAnglesY[i] = joints[i].localRotation.eulerAngles.y;
            m_baseAnglesZ[i] = joints[i].localRotation.eulerAngles.z;
        }


    }

    // Update is called once per frame
    void Update()
    {

        if(m_contextualTargetTransform !=null)
        {
            float contextualTargetDistance = Vector3.Distance(transform.position, m_contextualTargetTransform.position);
            if(contextualTargetDistance < distanceToDetect)
            {
                m_currentTargeTransform = m_contextualTargetTransform;
                hasContextualTargetClose = true;
            }
            else
            {
                m_currentTargeTransform = generalTargetTransform;
                hasContextualTargetClose = false;
            }
            
        }
        else
        {
            m_currentTargeTransform = generalTargetTransform;
        }

        // Reset iterations count
        m_iterations = 0;

        // IK solver algorithm 
        if (!isGenericJacobianActive)
        {
            JacobianIKOnlySpace();
        }
        else
        {
            GenericJacobianAll();
        }

    }
    #endregion

    #region  IK Solver functions
    /// <summary>
    /// Base algorithm IK solver only setup the position of the end effector to the target
    /// </summary>
    private void JacobianIKOnlySpace()
    {

        while (IsEndEffectorIsTooFar() && IsEnoughIteration() && IsTargetCloseEnough())
        {
            // Calcul the delta orientation values
            float[] dO = GetDeltaOrientation();


            //Set the values to the joint
            for (int i = 0; i < joints.Length; i++)
            {

                int indexAngle = i * 3;
                JointContraint jointContraint = joints[i].GetComponent<JointContraint>();

                m_anglesXArray[i] = (float)jointContraint.ClampJoint(m_anglesXArray[i], dO[indexAngle] * Time.deltaTime, Axis.X);
                m_anglesYArray[i] = (float)jointContraint.ClampJoint(m_anglesYArray[i], dO[indexAngle + 1] * Time.deltaTime, Axis.Y);
                m_anglesZArray[i] = (float)jointContraint.ClampJoint(m_anglesZArray[i], dO[indexAngle + 2] * Time.deltaTime, Axis.Z);

                joints[i].localRotation = Quaternion.Euler(m_anglesXArray[i], m_anglesYArray[i], m_anglesZArray[i]);

            }

            m_iterations++;

        }
    }


    /// <summary>
    /// First attempt for IK solver algorithm that set the position and the orientation of the end effector.
    /// Warning : Not use and not working
    /// </summary>
    private void JacobianIKAll()
    {


        while (Mathf.Abs((endEffectorTransform.position - m_currentTargeTransform.position).magnitude) > m_minDistance && m_iterations < iterationCount && Vector3.Distance(joints[joints.Length - 1].transform.position, m_currentTargeTransform.position) < maxDistance)
        {
            float[] dO = GetDeltaOrientation();

            for (int i = 0; i < joints.Length; i++)
            {


                int indexAngle = i * 3;
                JointContraint jointContraint = joints[i].GetComponent<JointContraint>();

                m_anglesXArray[i] = (float)jointContraint.ClampJoint(m_anglesXArray[i], dO[indexAngle] * Time.deltaTime, Axis.X);
                m_anglesYArray[i] = (float)jointContraint.ClampJoint(m_anglesYArray[i], dO[indexAngle + 1] * Time.deltaTime, Axis.Y);
                m_anglesZArray[i] = (float)jointContraint.ClampJoint(m_anglesZArray[i], dO[indexAngle + 2] * Time.deltaTime, Axis.Z);
                joints[i].localRotation = Quaternion.Euler(m_anglesXArray[i], m_anglesYArray[i], m_anglesZArray[i]);

            }

            float[] dO1 = GetDeltaOrientation2();
            for (int i = 0; i < joints.Length; i++)
            {
                int indexAngle = i * 3;
                JointContraint jointContraint = joints[i].GetComponent<JointContraint>();


                m_anglesXArray[i] = (float)jointContraint.ClampJoint(m_anglesXArray[i], dO1[indexAngle] * Time.deltaTime, Axis.X);
                m_anglesYArray[i] = (float)jointContraint.ClampJoint(m_anglesYArray[i], dO1[indexAngle + 1] * Time.deltaTime, Axis.Y);
                m_anglesZArray[i] = (float)jointContraint.ClampJoint(m_anglesZArray[i], dO1[indexAngle + 2] * Time.deltaTime, Axis.Z);


            }
            m_iterations++;

        }
    }

    /// <summary>
    /// Second attempt after research on Jaccobian on IK solver algorithm
    /// Inspired by the work of Shahin (Amir H.) Rabbani : https://www.shahinrabbani.ca/jacobian/a-recipe-to-cook-jacobian
    /// </summary>
    private void GenericJacobianAll()
    {
        float angle = 0;
        Vector3 axis = Vector3.zero;
        float angleEndEff = 0;
        float angleTarget = 0;

        // Try to calcul the difference in angle between the two rotation
        Quaternion quat = Quaternion.Inverse(m_currentTargeTransform.rotation) * endEffectorTransform.rotation;
        quat.ToAngleAxis(out angle, out axis);

        endEffectorTransform.rotation.ToAngleAxis(out angleEndEff, out axis);
        m_currentTargeTransform.rotation.ToAngleAxis(out angleTarget, out axis);

        angle = angleTarget - angleEndEff;

        while ( (IsEndEffectorIsTooFar() || IsOrientationIsTooFar(angle)) && IsEnoughIteration() && IsTargetCloseEnough())
        {

            // Creation of Jaccobian matrix
            JacobianMatrix jacobianMatrix = GenericJacobian.CreateGenericJacobian(joints.Length, joints);

            // Not sure about the calcul of the linear and angular error
            quat = Quaternion.Inverse(m_currentTargeTransform.rotation) * endEffectorTransform.rotation;
            Matrix4x4 mat = Matrix4x4.Rotate(m_currentTargeTransform.rotation).transpose * Matrix4x4.Rotate(endEffectorTransform.rotation);
            Vector3 angularError = (quat.eulerAngles);
            Vector3 linearError = (m_currentTargeTransform.position - endEffectorTransform.position);

            // Multiply the jaccobian matrix by the twist (linear error and angular Error)
            double[] dO = GenericJacobian.GetDeltaValues(linearError, angularError, jacobianMatrix);


            // Apply the orientation for each joint
            for (int i = 0; i < joints.Length; i++)
            {
                int indexAngle = i * 6;
                JointContraint jointContraint = joints[i].GetComponent<JointContraint>();

                float deltaTime = Time.deltaTime / iterationCount;

                m_anglesXArray[i] = (float)jointContraint.ClampJoint(m_anglesXArray[i], dO[indexAngle] * deltaTime + dO[indexAngle + 3] * deltaTime, Axis.X);
                m_anglesYArray[i] = (float)jointContraint.ClampJoint(m_anglesYArray[i], dO[indexAngle + 1] * deltaTime + dO[indexAngle + 4] * deltaTime, Axis.Y);
                m_anglesZArray[i] = (float)jointContraint.ClampJoint(m_anglesZArray[i], dO[indexAngle + 2] * deltaTime + dO[indexAngle + 5] * deltaTime, Axis.Z);


                Vector3 angleVec = new Vector3(m_anglesXArray[i], m_anglesYArray[i], m_anglesZArray[i]);
                joints[i].localRotation = Quaternion.Euler(angleVec);
            }

            // Re-calcul the value to see if the orientation of the end effector is matching the target orientation
            endEffectorTransform.rotation.ToAngleAxis(out angleEndEff, out axis);
            m_currentTargeTransform.rotation.ToAngleAxis(out angleTarget, out axis);
            angle = angleTarget - angleEndEff;

            m_iterations++;
        }

    }





    #endregion

    #region Jaccobian Calcul Functions
    float[] GetDeltaOrientation()
    {
        SimpleJacobianMatrix Jt = GetJacobianTranspose();
        Vector3 V = m_currentTargeTransform.position - endEffectorTransform.position;

        float[] dO = Jt * V;

        return dO;
    }

    float[] GetDeltaOrientation2()
    {
        SimpleJacobianMatrix Jt = GetJacobianTranspose2();
        Vector3 directionDiff = joints[0].eulerAngles - joints[1].eulerAngles;

        float[] dO = Jt * directionDiff;
        return dO;
    }

    /* Step Three */
    SimpleJacobianMatrix GetJacobianTranspose()
    {
        Vector3 endEffectorPos = endEffectorTransform.position;
        Vector3 endEffectorDirection = endEffectorTransform.forward;

        SimpleJacobianMatrix J = new SimpleJacobianMatrix();
        for (int i = 0; i < joints.Length; i++)
        {
            // Location in spaces
            Vector3 J_A = Vector3.Cross(joints[i].transform.right, (endEffectorPos - joints[i].position).normalized);
            Vector3 J_B = Vector3.Cross(joints[i].transform.up, (endEffectorPos - joints[i].position).normalized);
            Vector3 J_C = Vector3.Cross(joints[i].transform.forward, (endEffectorPos - joints[i].position).normalized);

            J.AddColumn(J_A);
            J.AddColumn(J_B);
            J.AddColumn(J_C);


        }

        return J.Transpose();
    }


    SimpleJacobianMatrix GetJacobianTranspose2()
    {
        Vector3 endEffectorPos = endEffectorTransform.position;
        Vector3 endEffectorDirection = endEffectorTransform.forward;

        SimpleJacobianMatrix J = new SimpleJacobianMatrix();
        for (int i = 0; i < joints.Length; i++)
        {

            if (isGenericJacobianActive)
            {

                Vector3 J_D = Vector3.Cross(joints[i].transform.right, endEffectorTransform.right - joints[i].transform.right);
                Vector3 J_E = Vector3.Cross(joints[i].transform.up, endEffectorTransform.up - joints[i].transform.up);
                Vector3 J_F = Vector3.Cross(joints[i].transform.forward, endEffectorTransform.forward - joints[i].transform.forward);
                J.AddColumn(J_D);
                J.AddColumn(J_E);
                J.AddColumn(J_F);

            }
        }

        return J.Transpose();
    }
    #endregion

    #region Conditions Functions
    private bool IsEndEffectorIsTooFar()
    {
        return (m_currentTargeTransform.position - endEffectorTransform.position).magnitude > m_minDistance;
    }

    private bool IsEnoughIteration()
    {
        return m_iterations < iterationCount;
    }

    private bool IsTargetCloseEnough()
    {
        return Vector3.Distance(joints[joints.Length - 1].transform.position, m_currentTargeTransform.position) < maxDistance;
    }

    private bool IsOrientationIsTooFar(float angle)
    {
        return Mathf.Abs(angle) > m_minAngleOrientation;
    }

    #endregion



    public void AddContextualTarget(Transform newTarget)
    {
        m_contextualTargetTransform = newTarget;
        HasContextualTarget = true;
    }


    public void RemoveContextualTarget()
    {
        m_contextualTargetTransform = null;
        HasContextualTarget = false;
    }

   

    public void OnDrawGizmos()
    {

        if (!isDebugActive) return;
        float distance = maxDistance;

        Gizmos.DrawRay(joints[joints.Length - 1].position, (m_currentTargeTransform.position - joints[joints.Length - 1].transform.position) * distance);

        Gizmos.DrawCube(m_currentTargeTransform.position, Vector3.one * .3f);
        Gizmos.DrawCube(endEffectorTransform.position, Vector3.one * .3f);

    }
}
