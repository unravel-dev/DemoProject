using System;
using Unravel.Core;

/// <summary>
/// Drives the animation component from the <see cref="CharacterController"/>'s
/// runtime state. Designed to produce smooth, non-ping-ponging transitions even when
/// the player spams move input or oscillates around threshold speeds.
///
/// Key ideas:
///   - The animator is a small state machine. A single cached <see cref="LocoState"/>
///     is compared against the desired state every frame, and <see cref="AnimationComponent.Blend"/>
///     is only invoked when the state actually changes. This eliminates the previous
///     behaviour of re-issuing Blend every frame and is the root fix for most pops.
///   - Idle &lt;-&gt; locomotion is INTENT-DRIVEN: we use
///     <see cref="CharacterController.IsMoving"/> (which reflects forward/back input)
///     as the authoritative signal. Speed is used only for tier selection and a short
///     optional decel tail. This is critical when root motion is enabled: if we gated
///     idle on <see cref="CharacterController.CurrentSpeed"/> alone, any residual
///     logical velocity (axis smoothing, a frame where <see cref="CharacterController.IsGrounded"/>
///     flickers and PreserveAirMomentum skips decel, etc.) would pin the animator in
///     a locomotion clip, whose root motion keeps sliding the character forever.
///   - Walk / jog / run tiers use both a speed hysteresis band and an optional time
///     debounce (<see cref="TierDebounce"/>) so micro speed wobbles don't switch clips.
///   - Jump vs Fall is decided by vertical velocity instead of "any time I'm airborne
///     I'm jumping", so the correct clip plays on the way down.
///   - Blend times are chosen per transition class (idle&lt;-&gt;move, within-locomotion,
///     jump, fall, land, turn, dance). Phase-sync is only enabled between states that
///     share a stride phase (walk/jog/run/backwards) so entering a locomotion clip
///     from idle/air/turn doesn't snap to a mid-stride pose.
/// </summary>
[ScriptSourceFile]
class CharacterAnimator : ScriptComponent
{
    // ---------------------------------------------------------------------
    // Clip slots
    // ---------------------------------------------------------------------
    [Header("Locomotion Clips")]
    public AnimationClip idleClip;
    public AnimationClip walkClip;            // forward walk
    public AnimationClip walkBackwardsClip;
    public AnimationClip slowRunClip;         // jog tier
    public AnimationClip runClip;             // sprint tier

    [Header("Air Clips")]
    public AnimationClip jumpClip;
    public AnimationClip fallClip;

    [Header("Special Clips")]
    public AnimationClip danceClip;
    public AnimationClip leftTurnClip;
    public AnimationClip rightTurnClip;

    // ---------------------------------------------------------------------
    // Thresholds (hysteresis)
    // ---------------------------------------------------------------------
    [Header("Locomotion Thresholds")]
    [Tooltip("Speed (m/s) at which the character commits to *starting* a locomotion clip. " +
             "Below this we still hold Idle even if input is held, so a one-frame tap doesn't " +
             "flash a walk step. Leave at 0 to enter locomotion the instant input is held.")]
    public float MoveStartSpeed = 0.6f;

    [Tooltip("Maximum time (s) to keep the locomotion clip playing AFTER the player releases " +
             "input, as a graceful decel tail. 0 = snap to Idle on release (safest with root motion). " +
             "If you see the character sliding because a locomotion clip keeps playing, lower this. " +
             "If releases feel abrupt, raise it. The clip is always forced to Idle once the decel " +
             "tail expires, regardless of logical speed.")]
    [Min(0)]
    public float MoveReleaseTail = 0.0f;

    [Tooltip("Hysteresis band (m/s) around the walk->jog and jog->run tier boundaries. " +
             "You must overshoot the boundary by this much to enter the higher tier, and " +
             "undershoot by the same margin to fall back down.")]
    [Min(0)]
    public float TierHysteresis = 0.3f;

    // ---------------------------------------------------------------------
    // Blend durations
    // ---------------------------------------------------------------------
    [Header("Blend Durations (seconds)")]
    [Tooltip("Blend length entering a locomotion clip from Idle (no stride phase to match).")]
    public float BlendIdleToMove = 0.20f;

    [Tooltip("Blend length returning to Idle from a locomotion clip.")]
    public float BlendMoveToIdle = 0.25f;

    [Tooltip("Blend length between locomotion tiers (walk <-> jog <-> run). Phase-synced, " +
             "so a longer blend just lingers in the crossfade.")]
    public float BlendWithinLocomotion = 0.25f;

    [Tooltip("Blend length entering the jump clip when leaving the ground.")]
    public float BlendToJump = 0.10f;

    [Tooltip("Blend length from jump to fall once vertical velocity drops below FallThreshold.")]
    public float BlendJumpToFall = 0.20f;

    [Tooltip("Blend length from an air clip back into a grounded clip (the 'landing' blend).")]
    public float BlendLand = 0.15f;

    [Tooltip("Blend length entering a turn-in-place clip.")]
    public float BlendToTurn = 0.18f;

    [Tooltip("Blend length entering the dance clip.")]
    public float BlendToDance = 0.25f;

    [Tooltip("Blend length entering the walk-backwards clip.")]
    public float BlendBackwards = 0.25f;

    // ---------------------------------------------------------------------
    // Air / anti-ping-pong
    // ---------------------------------------------------------------------
    [Header("Air Detection")]
    [Tooltip("Vertical velocity (m/s) below which we switch from the jump clip to the fall clip. " +
             "A small negative value (e.g. -0.5) gives a natural apex hang before the fall pose.")]
    public float FallThreshold = -0.5f;

    [Header("Anti-ping-pong")]
    [Tooltip("A locomotion tier change (walk/jog/run) must persist for this many seconds before " +
             "actually switching clips. Idle<->move already uses speed-level hysteresis, so this " +
             "only affects within-locomotion tier flicker. 0 = disabled.")]
    [Min(0)]
    public float TierDebounce = 0.08f;

    [Tooltip("Enable stride phase synchronisation between matching locomotion clips (walk/jog/run/back). " +
             "Phase sync is always disabled when one side has no stride (idle, turn, jump, fall, dance).")]
    public bool phaseSync = true;

    [Header("Debug")]
    [Tooltip("Log every state transition (From -> To) with the driving signals. Useful for diagnosing " +
             "'animation never stops' issues: if you release input and don't see a 'Walk -> Idle' entry " +
             "in the log, either IsMoving is pinned true on the controller or idleClip is unassigned.")]
    public bool LogTransitions = false;

    // ---------------------------------------------------------------------
    // Internal state
    // ---------------------------------------------------------------------
    private enum LocoState
    {
        None,
        Idle,
        Walk,
        WalkBackwards,
        Jog,
        Run,
        TurnLeft,
        TurnRight,
        Jump,
        Fall,
        Dance,
    }

    private CharacterController controller_;
    private AnimationComponent anim;
    private CharacterControllerComponent cc;

    private LocoState state = LocoState.None;

    // Debounce: which tier we *want* to switch to, and how long it has persisted.
    private LocoState pendingTier = LocoState.None;
    private float pendingTimer = 0.0f;

    // Seconds since the player last RELEASED locomotion input. Used to drive the
    // optional MoveReleaseTail so we can keep the locomotion clip playing briefly
    // after release while velocity bleeds off, then force Idle.
    private float timeSinceRelease = float.MaxValue;

    // ---------------------------------------------------------------------
    // Lifecycle
    // ---------------------------------------------------------------------
    public override void OnStart()
    {
        controller_ = owner.GetComponent<CharacterController>();
        anim = owner.GetComponent<AnimationComponent>();
        cc = owner.GetComponent<CharacterControllerComponent>();

        if (idleClip != null && anim != null)
        {
            anim.Blend(idleClip, 0.0f, true, false);
            anim.Play();
            state = LocoState.Idle;
        }
    }

    public override void OnUpdate()
    {
        if (controller_ == null || anim == null)
            return;

        float dt = Time.deltaTime;

        // 1) Dance override - trumps everything else.
        if (danceClip != null && Input.IsDown(KeyCode.R))
        {
            CommitState(LocoState.Dance, danceClip, BlendToDance, true, false);
            return;
        }

        // 2) Airborne: jump vs fall based on vertical velocity.
        if (!controller_.IsGrounded)
        {
            float vy = cc != null ? cc.velocity.y : 0.0f;

            if (vy <= FallThreshold && fallClip != null)
            {
                CommitState(LocoState.Fall, fallClip, BlendJumpToFall, true, false);
            }
            else if (jumpClip != null)
            {
                // Going up, at the apex, or we simply don't have a fall clip.
                CommitState(LocoState.Jump, jumpClip, BlendToJump, false, false);
            }
            else if (fallClip != null)
            {
                CommitState(LocoState.Fall, fallClip, BlendJumpToFall, true, false);
            }
            else if (idleClip != null)
            {
                CommitState(LocoState.Idle, idleClip, BlendLand, true, false);
            }
            return;
        }

        // 3) Turn-in-place. Phase-sync only when swapping between the two turn clips
        //    so a left-turn-that-reverses-to-right doesn't reset mid-pose.
        if (controller_.IsRotatingInPlace)
        {
            if (controller_.TurningInPlaceDirection < -0.1f && leftTurnClip != null)
            {
                CommitState(LocoState.TurnLeft, leftTurnClip, BlendToTurn, true,
                            phaseSync && state == LocoState.TurnRight);
            }
            else if (controller_.TurningInPlaceDirection > 0.1f && rightTurnClip != null)
            {
                CommitState(LocoState.TurnRight, rightTurnClip, BlendToTurn, true,
                            phaseSync && state == LocoState.TurnLeft);
            }
            else if (idleClip != null)
            {
                CommitState(LocoState.Idle, idleClip, BlendMoveToIdle, true, false);
            }
            return;
        }

        // 4) Idle <-> moving, driven primarily by INPUT INTENT, not logical speed.
        //
        //    This is the critical safety rule when root motion is enabled: the
        //    clip itself is what translates the character, so if we ever fail to
        //    blend back to Idle the character will keep sliding forever - there
        //    is no independent "velocity" that decays to stop them. Gating on
        //    CurrentSpeed alone leaves us vulnerable to any source of residual
        //    logical velocity (input-axis smoothing, a frame where
        //    CharacterController.PreserveAirMomentum skips the decel, etc).
        //
        //    Rules:
        //      - Input held + above MoveStartSpeed -> enter locomotion.
        //        (One-frame taps that never reach MoveStartSpeed are ignored so
        //         we don't flash a walk step.)
        //      - Input released                    -> start the decel tail timer.
        //        We keep playing the current locomotion clip for up to
        //        MoveReleaseTail seconds so the stop looks natural, then we
        //        FORCE Idle regardless of logical speed. Setting MoveReleaseTail
        //        to 0 makes release snap to Idle immediately - the safest
        //        setting with root motion.
        float speed = controller_.CurrentSpeed;
        bool wasIdle = state == LocoState.Idle || state == LocoState.None
                    || state == LocoState.TurnLeft || state == LocoState.TurnRight;
        bool landing = state == LocoState.Jump || state == LocoState.Fall;

        if (controller_.IsMoving)
        {
            timeSinceRelease = 0.0f;
        }
        else
        {
            timeSinceRelease += dt;
        }

        bool shouldIdle;
        if (controller_.IsMoving)
        {
            // While input is held, only refuse to enter locomotion if we're
            // currently idle AND haven't ramped up past MoveStartSpeed yet.
            // Once we're in a locomotion clip, stay there while input is held.
            shouldIdle = wasIdle && speed < MoveStartSpeed;
        }
        else
        {
            // Input released: allow a brief decel tail, then always go Idle.
            shouldIdle = timeSinceRelease >= MoveReleaseTail;
        }

        if (shouldIdle)
        {
            if (idleClip != null)
            {
                float blend = landing ? BlendLand : BlendMoveToIdle;
                CommitState(LocoState.Idle, idleClip, blend, true, false);
            }
            else if (LogTransitions && state != LocoState.Idle && state != LocoState.None)
            {
                Log.Warning($"[CharacterAnimator] shouldIdle=true but idleClip is null - cannot leave state {state}. " +
                            $"Assign idleClip in the inspector or the character will be stuck in the current clip.");
            }
            return;
        }

        // 5) Moving. Backwards is its own state and doesn't phase-sync with walk
        //    (different stride direction).
        if (controller_.IsBackwards && walkBackwardsClip != null)
        {
            CommitState(LocoState.WalkBackwards, walkBackwardsClip, BlendBackwards, true, false);
            return;
        }

        // 5a) Tier selection (walk / jog / run) with a symmetric hysteresis band.
        float walkSpeed   = controller_.WalkSpeed;
        float runSpeed    = controller_.RunSpeed;
        float sprintSpeed = controller_.SprintSpeed;
        float walkToJog   = 0.5f * (walkSpeed + runSpeed);
        float jogToRun    = 0.5f * (runSpeed + sprintSpeed);

        int currentTier = state == LocoState.Jog ? 1
                        : state == LocoState.Run ? 2
                        : 0;

        int desiredTier;
        if (speed < walkToJog - (currentTier >= 1 ? TierHysteresis : 0.0f))
            desiredTier = 0;
        else if (speed < jogToRun - (currentTier >= 2 ? TierHysteresis : 0.0f))
            desiredTier = 1;
        else
            desiredTier = 2;

        LocoState desired = desiredTier == 0 ? LocoState.Walk
                          : desiredTier == 1 ? LocoState.Jog
                                             : LocoState.Run;
        AnimationClip desiredClip = desiredTier == 0 ? walkClip
                                  : desiredTier == 1 ? slowRunClip
                                                     : runClip;
        if (desiredClip == null)
            return;

        // 5b) Debounce tier changes only. Idle<->move and air transitions are
        //     handled by other mechanisms and stay responsive.
        bool isTierSwitch = (state == LocoState.Walk || state == LocoState.Jog || state == LocoState.Run)
                         && (desired != state);
        if (TierDebounce > 0.0f && isTierSwitch)
        {
            if (pendingTier != desired)
            {
                pendingTier = desired;
                pendingTimer = 0.0f;
            }
            pendingTimer += dt;
            if (pendingTimer < TierDebounce)
                return; // keep current tier clip for a bit longer
        }
        pendingTier = LocoState.None;
        pendingTimer = 0.0f;

        // 5c) Entering from a non-locomotion clip has no stride to sync with,
        //     and should use the idle->move blend (longer) instead of the
        //     within-locomotion crossfade.
        bool enteringLocomotion = state != LocoState.Walk
                               && state != LocoState.Jog
                               && state != LocoState.Run;
        bool sync  = phaseSync && !enteringLocomotion;
        float blendSec = landing             ? BlendLand
                       : enteringLocomotion  ? BlendIdleToMove
                                             : BlendWithinLocomotion;

        CommitState(desired, desiredClip, blendSec, true, sync);
    }

    // ---------------------------------------------------------------------
    // State transition primitive. Only actually issues a Blend when the
    // state changes, so spamming the same state from OnUpdate is free and
    // never restarts the blend timer.
    // ---------------------------------------------------------------------
    private void CommitState(LocoState next, AnimationClip clip, float blendSeconds, bool loop, bool sync)
    {
        if (clip == null)
        {
            if (LogTransitions)
                Log.Warning($"[CharacterAnimator] CommitState({next}) skipped: clip is null.");
            return;
        }
        if (next == state) return;

        if (LogTransitions)
        {
            float speed = controller_ != null ? controller_.CurrentSpeed : 0.0f;
            bool moving = controller_ != null && controller_.IsMoving;
            bool back   = controller_ != null && controller_.IsBackwards;
            Log.Info($"[CharacterAnimator] {state} -> {next}  (blend={blendSeconds:0.00}s loop={loop} sync={sync}) " +
                     $"| IsMoving={moving} IsBackwards={back} speed={speed:0.00} tail={timeSinceRelease:0.00}s");
        }

        anim.Blend(clip, blendSeconds, loop, sync);
        state = next;
        pendingTier = LocoState.None;
        pendingTimer = 0.0f;
    }
}
