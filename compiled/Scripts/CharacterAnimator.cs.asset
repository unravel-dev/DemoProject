using System;
using Unravel.Core;
[ScriptSourceFile]
class CharacterAnimator : ScriptComponent
{
    // Idle / Locomotion
    public AnimationClip idleClip;
    public AnimationClip walkClip;          // forward
    public AnimationClip walkBackwardsClip;
    public AnimationClip slowRunClip;
    public AnimationClip runClip;

    // Jump / Fall
    public AnimationClip jumpClip;
    public AnimationClip fallClip;

    // Special actions
    public AnimationClip danceClip;
    public AnimationClip leftTurnClip; 
    public AnimationClip rightTurnClip; 

    public bool phaseSync = true;

    // We'll reference the Movement script to read IsGrounded, CurrentSpeed, etc.
    private CharacterController cotroller_;
    private AnimationComponent anim;

    public override void OnStart()
    {
        // Locate the movement script on the same GameObject
        cotroller_ = owner.GetComponent<CharacterController>();
        anim = owner.GetComponent<AnimationComponent>();

        // Optionally define default clip
        if (idleClip != null)
        {
            anim.Blend(idleClip, 0.0f, true, false);
            anim.Play();
        }

    }

    public override void OnUpdate()
    {
        // 1) Check for special "dance" input (overrides everything else)
        if (Input.IsDown(KeyCode.R))
        {
            if (danceClip != null)
            {
                anim.Blend(danceClip, 0.2f, true, false);
                return; // no other state overrides dancing
            }
        }

        // 2) If not grounded, pick jump or fall
        if (!cotroller_.IsGrounded)
        {
            // If you want a more advanced check for "just jumped" vs. truly falling, 
            // you might expose that from CharacterMovement or store a jump timer. 
            // For simplicity, if we have jumpClip, use it, else fallClip:
            if (jumpClip != null)
            {
                // Just jumped this frame
                anim.Blend(jumpClip, 0.2f, false, false);
            }
            else if (fallClip != null)
            {
                // In mid-air and we have a fall clip
                anim.Blend(fallClip, 0.2f, true, false);
            }
            else
            {
                // fallback
                if (idleClip != null) anim.Blend(idleClip, 0.2f, true, false);
            }
            return;
        }

        // 3) If grounded, possibly turning in place
        if (cotroller_.IsRotatingInPlace)
        {
            // check direction
            if (cotroller_.TurningInPlaceDirection < -0.1f)
            {
                // left turn
                if (leftTurnClip != null)
                    anim.Blend(leftTurnClip, 0.2f, true, phaseSync);
                else if (idleClip != null)
                    anim.Blend(idleClip, 0.2f, true, false);
            }
            else if (cotroller_.TurningInPlaceDirection > 0.1f)
            {
                // right turn
                if (rightTurnClip != null)
                    anim.Blend(rightTurnClip, 0.2f, true, phaseSync);
                else if (idleClip != null)
                    anim.Blend(idleClip, 0.2f, true, false);
            }
            else
            {
                // fallback if direction = 0 for some reason
            }
            return;
        }

        // 4) Normal grounded locomotion logic
        float speed     = cotroller_.CurrentSpeed; 
        float walkSpeed = cotroller_.WalkSpeed;
        float runSpeed  = cotroller_.RunSpeed;

        // 4a) If hardly moving
        if (speed < 0.1f)
        {
            if (idleClip != null) anim.Blend(idleClip, 0.2f, true, false);
        }
        else
        {
            // 4b) If backwards
            if (cotroller_.IsBackwards && walkBackwardsClip != null)
            {
                anim.Blend(walkBackwardsClip, 0.3f, true, phaseSync);
            }
            // 4c) If below or near walk speed
            else if (speed < walkSpeed + 0.1f)
            {
                if (walkClip != null) anim.Blend(walkClip, 0.5f, true, phaseSync);
            }
            // 4d) If above walk speed but not fully running
            else if (speed < runSpeed - 0.1f)
            {
                if (slowRunClip != null) anim.Blend(slowRunClip, 0.3f, true, phaseSync);
            }
            // 4e) Full run
            else
            {
                if (runClip != null) anim.Blend(runClip, 0.5f, true, phaseSync);
            }
        }
    }
}
