using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Unity_IK : MonoBehaviour
{
    protected Animator animator;

    public bool ikActive = false;
    public Transform rightHandObj = null;
    public Transform lookObj = null;

    public float timeTransition;
    private float timerTransition;
    private bool isFirstTimeSwitch;

    void Start()
    {
        animator = GetComponent<Animator>();
    }

    //a callback for calculating IK
    void OnAnimatorIK()
    {
        if (animator)
        {

            if(ikActive != isFirstTimeSwitch)
            {
                timerTransition = 0.0f;
                isFirstTimeSwitch = ikActive;
            }

            timerTransition += Time.deltaTime;
            float ratio = Mathf.Clamp(timerTransition / timeTransition, 0.0f, 1f);
            //if the IK is active, set the position and rotation directly to the goal.
            if (ikActive)
            {

                // Set the look target position, if one has been assigned
                if (lookObj != null)
                {
                    animator.SetLookAtWeight(ratio);
                    animator.SetLookAtPosition(lookObj.position);
                }

                // Set the right hand target position and rotation, if one has been assigned
                if (rightHandObj != null)
                {
                    animator.SetIKPositionWeight(AvatarIKGoal.RightHand, ratio);
                    animator.SetIKRotationWeight(AvatarIKGoal.RightHand, ratio);
                    animator.SetIKPosition(AvatarIKGoal.RightHand, rightHandObj.position);
                    animator.SetIKRotation(AvatarIKGoal.RightHand, rightHandObj.rotation);
                }

            }

            //if the IK is not active, set the position and rotation of the hand and head back to the original position
            else
            {
                animator.SetIKPositionWeight(AvatarIKGoal.RightHand, 1.0f - ratio);
                animator.SetIKRotationWeight(AvatarIKGoal.RightHand, 1.0f - ratio);
                animator.SetIKPosition(AvatarIKGoal.RightHand, rightHandObj.position);
                animator.SetIKRotation(AvatarIKGoal.RightHand, rightHandObj.rotation);
                animator.SetLookAtWeight(1.0f - ratio);
                animator.SetLookAtPosition(lookObj.position);
            }
        }
    }
}
