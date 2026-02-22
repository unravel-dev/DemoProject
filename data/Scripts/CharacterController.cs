using System;
using Unravel.Core;


[ScriptSourceFile]
class CharacterController : ScriptComponent
{
    [Tooltip("Entity for the left foot used by inverse kinematics.")]
    public Entity LeftFoot;

    [Tooltip("Entity for the right foot used by inverse kinematics.")]
    public Entity RightFoot;

    [Tooltip("Entity representing the character's hips for IK-based vertical offset.")]
    public Entity Hips;

    [Tooltip("Entity representing the spine root for spine IK (aiming adjustments)." )]
    public Entity Spine;

    [Tooltip("Entity for the left hand used by inverse kinematics when aiming/shooting.")]
    public Entity LeftHand;

    [Tooltip("Entity for the right hand used by inverse kinematics when aiming/shooting.")]
    public Entity RightHand;

    [Tooltip("Prefab for the projectile spawned when shooting.")]
    public Prefab Projectice;

    [Tooltip("Controls whether inverse kinematics is applied to feet.")]
    public bool ApplyIK = true;

    [Tooltip("Minimum foot height above the hips before IK blends out (in world units).")]
    public float FootHeightMin = 0.14f;

    [Tooltip("Maximum foot height above the hips before IK is disabled (in world units).")]
    public float FootHeightMax = 0.22f;

    [Tooltip("Vertical offset above the foot from which to start the ground raycast (in world units).")]
    public float FootRaycastOffset = 0.5f;

    [Tooltip("Additional vertical offset to apply to the foot IK target for foot thickness (in world units).")]
    public float FootTargetYOffset = 0.1f;

    [Range(0,5)]
    [Tooltip("Speed at which the spine IK weight ramps up/down when aiming.")]
    public float SpineIKSpeed = 5f;

    [Range(0,5)]
    [Tooltip("Speed at which hand IK weights ramp up/down when aiming.")]
    public float HandIKSpeed = 5f;

    [Range(0,20)]
    [Tooltip("How quickly left/right hand IK weights swap based on aim angle.")]
    public float HandSideSpeed = 15f;

    [Range(0,90)]
    [Tooltip("Aim cone angle (degrees) at which the off-hand IK begins to fade out.")]
    public float HandSwitchStartAngle = 20f;

    [Range(0,90)]
    [Tooltip("Aim cone angle (degrees) at which the off-hand IK has fully faded out.")]
    public float HandSwitchEndAngle = 30f;

    [Tooltip("Maximum allowed deviation (degrees) from forward direction for spine/hand IK.")]
    public float ForwardAngle = 110f;

    // --- Movement configuration ---
    [Tooltip("Walking speed of the character (units per second).")]
    public float WalkSpeed = 2f;

    [Tooltip("Running speed of the character (units per second).")]
    public float RunSpeed = 5f;

    [Tooltip("Rate at which the character accelerates to target speed (units/s^2).")]
    public float Acceleration = 8f;

    [Tooltip("Rate at which the character decelerates to target speed (units/s^2).")]
    public float DeAcceleration = 8f;

    [Tooltip("Base rotation speed (degrees per second) when walking.")]
    public float RotateSpeed = 1.0f;

    [Tooltip("Rotation speed (degrees per second) when running.")]
    public float RunRotateSpeed = 2.0f;

    // --- Jump configuration ---
    [Tooltip("Upward impulse force applied when the character jumps.")]
    public float JumpForce = 5f;

    [Tooltip("Radius of the sphere used for ground checking.")]
    public float SphereCastRadius = 0.2f;

    [Tooltip("Distance below the character from which to check for ground.")]
    public float GroundCheckDistance = 0.5f;

    [Tooltip("Maximum slope angle (degrees) from vertical that is considered walkable.")]
    public float SlopeLimitAngle = 60f;

    [Tooltip("Toggle enabling manual gravity instead of using the physics engine's gravity.")]
    public bool UseManualGravity = false;

    [Tooltip("Magnitude of the custom gravity applied when UseManualGravity is true.")]
    public float GravityMagnitude = 9.81f;

    [Tooltip("Maximum downward speed when using custom gravity (units per second).")]
    public float MaxFallSpeed = 20f;

    [Tooltip("Duration (seconds) after leaving ground during which a jump is still allowed (coyote time).")]
    public float CoyoteTimeDuration = 0.2f;

    [Tooltip("Duration (seconds) to buffer a jump input before landing (jump buffer).")]
    public float JumpBufferDuration = 0.1f;

    // --- Public properties (for Animations or other systems) ---
    public float CurrentSpeed { get; private set; } = 0f;
    public float CurrentRotateSpeed { get; private set; } = 0f;
    public bool  IsGrounded         { get; private set; } = true;
    public bool  IsBackwards        { get; private set; } = false;
    public bool  IsMoving           { get; private set; } = false;
    public bool  IsRotatingInPlace  { get; private set; } = false;
    public float TurningInPlaceDirection { get; private set; } = 0f; // negative=left, positive=right
    public bool  IsRunning          { get; private set; } = false;

    private PhysicsComponent rb;
    private float righthandIKWeight = 0f;
    private float lefthandIKWeight  = 0f;
    private float handIKWeight = 0f;    // current blend (0 = off, 1 = full)
    private float spineIKWeight = 0f;   // 0 = off, 1 = full;
    // IK state fields for foot blending and Hips offset
    private float leftFootIKBlend = 1.0f;
    private float rightFootIKBlend = 1.0f;
    private float hipsIKOffset = 0f;
    private float timeSinceUngrounded = 0f;
    private float timeSinceJumpPressed= float.MaxValue; 
    private Entity ProjecticeContainer;

    public override void OnStart()
    {
        rb = owner.GetComponent<PhysicsComponent>();

        LeftFoot = transform.FindChild("mixamorig:LeftFoot", true);
        RightFoot = transform.FindChild("mixamorig:RightFoot", true);

        LeftHand = transform.FindChild("mixamorig:LeftHand", true);
        RightHand = transform.FindChild("mixamorig:RightHand", true);
        Hips = transform.FindChild("mixamorig:Hips", true);
        Spine = transform.FindChild("mixamorig:Spine", true);

        ProjecticeContainer = Scene.CreateEntity("Projectices");

    }

    public void Test()
    {
        PhysicsComponent s = new  PhysicsComponent();;
        s.angularVelocity = Vector3.one;
    }

    public override void OnUpdate()
    {

        float dt = Time.deltaTime;

        // 1) Update coyote/jump-buffer timers
        if (IsGrounded)
        {
            timeSinceUngrounded = 0f;
        }
        else
        {
            timeSinceUngrounded += dt;
        }

        if (Input.IsPressed("Jump"))
        {
            timeSinceJumpPressed = 0f;
        }
        else
        {
            timeSinceJumpPressed += dt;
        }


        // 2) Ground check
        UpdateGroundedState();

        if(ApplyIK)
        {
            // After updating ground check and movement, update IK:
            ApplyFeetIK(dt);

        }
        ApplySpineIK(dt);

        // 3) Read input
        float inputX = Input.GetAxis("Horizontal"); // left/right
        float inputZ = Input.GetAxis("Vertical");   // forward/back
        float runHeld = Input.GetAxis("Run");       // 0..1

        if(IsBackwards)
        {
            runHeld = 0.0f;
        }
        // Decide if we are "running"
        IsRunning = (runHeld > 0.5f);

        // 4) Movement Speed / Accel
        float targetSpeed       = Mathf.Lerp(WalkSpeed,  RunSpeed,  runHeld);
        float targetRotateSpeed = Mathf.Lerp(RotateSpeed, RunRotateSpeed, runHeld);

        // Current velocity
        Vector3 vel = rb.velocity;
        float horizontalSpeed = new Vector2(vel.x, vel.z).magnitude;
        float velocityY = vel.y;

        // 5) Handle gravity if using manual
        if (UseManualGravity && !IsGrounded)
        {
            velocityY -= GravityMagnitude * dt;
            velocityY = Mathf.Max(velocityY, -MaxFallSpeed);
        }

        // Are we moving forward/back enough to call it "movement"?
        // We'll define "IsMoving" if magnitude of (inputX, inputZ) > 0.2f,
        // but specifically checking forward/back ignoring small horizontal only input
        // or just do standard magnitude if you want diagonal as moving
        Vector2 planar = new Vector2(inputX, inputZ);
        float planeMag = planar.magnitude;
        IsMoving = (inputZ > 0.01f) || (inputZ < -0.01f);  // threshold

        float accelValue = (CurrentSpeed < targetSpeed) ? Acceleration : DeAcceleration;
        // Accelerate or decelerate to targetSpeed
        CurrentSpeed = Mathf.MoveTowards(CurrentSpeed, targetSpeed, accelValue * dt);

        CurrentRotateSpeed = Mathf.MoveTowards(CurrentRotateSpeed, targetRotateSpeed, accelValue * dt);



        // Jump logic
        bool canJump = (IsGrounded || timeSinceUngrounded < CoyoteTimeDuration);
        bool jumpJustPressed = (timeSinceJumpPressed < JumpBufferDuration);
        if (canJump && jumpJustPressed)
        {
            velocityY  = JumpForce;
            IsGrounded = false;
            timeSinceJumpPressed = float.MaxValue;
        }

        // 6) Horizontal movement
        if (!IsMoving)
        {
            // No forward/back movement => decelerate horizontally
            CurrentSpeed = Mathf.MoveTowards(horizontalSpeed, 0f, DeAcceleration * dt);

            if (horizontalSpeed > 0.01f)
            {
                Vector2 horizontalDir = new Vector2(vel.x, vel.z).normalized;
                Vector2 newHoriz = horizontalDir * CurrentSpeed;
                rb.velocity = new Vector3(newHoriz.x, velocityY, newHoriz.y);
            }
            else
            {
                rb.velocity = new Vector3(0f, velocityY, 0f);
            }

            // We'll handle rotation in a separate step
        }
        else
        {


            // Build final horizontal velocity
            // Full direction
            Vector3 moveDir = new Vector3(inputX, 0, inputZ);
            if (moveDir.magnitude > 1f)
                moveDir.Normalize();


            Vector3 horizontal = transform.TransformDirection(moveDir) * CurrentSpeed;
            Vector3 finalVel = new Vector3(horizontal.x, velocityY, horizontal.z);
            rb.velocity = finalVel;

            // behind check
            Vector3 inputDir = new Vector3(finalVel.x, 0f, finalVel.z).normalized;
            float dotForward = Vector3.Dot(transform.forward, inputDir);
            float dotRight = Vector3.Dot(transform.right, inputDir);

            bool purelyBehind = (dotForward < 0f && Mathf.Abs(dotRight) < 0.2f);
            IsBackwards = purelyBehind;
        }

        // 7) Rotation step (outside the isMoving check)
        // Even if not "moving forward," we could rotate in place.
        HandleRotation(inputX, inputZ);
    }

    private void HandleRotation(float inputX, float inputZ)
    {
        float dt = Time.deltaTime;

        // By default, not rotating in place
        IsRotatingInPlace = false;
        TurningInPlaceDirection = 0f;

        // 1) Build a direction from input for rotation
        Vector3 inputVec = new Vector3(inputX, 0, inputZ);

        bool hasHorizontalInput = Mathf.Abs(inputX) > 0.2f;
        // If magnitude is small, maybe we are purely rotating left/right
        bool hasHorizontalOnly = (hasHorizontalInput && Mathf.Abs(inputZ) < 0.2f);

        if (hasHorizontalOnly)
        {
            // We are rotating in place
            IsRotatingInPlace = true;
            TurningInPlaceDirection = Mathf.Sign(inputX); // negative=left, positive=right

            // Let's do an in-place rotation
            // Typically, you'd set a rotation speed. We can use CurrentRotateSpeed or a base
            float rotationAngle = TurningInPlaceDirection * CurrentRotateSpeed * dt * 50f;
            // 50f is an arbitrary multiplier so you can tune the turn speed

   
            transform.RotateByEuler(Vector3.up * rotationAngle);
        }
        else
        {
            // We might have forward/back movement or be idle with no horizontal input
            // If we have forward movement, we might do the Slerp approach
            Vector3 velocityXZ = new Vector3(rb.velocity.x, 0, rb.velocity.z);
            if (!IsBackwards && velocityXZ.sqrMagnitude > 0.0001f)        
            {
                Quaternion targetRot = Quaternion.LookRotation(velocityXZ);
                transform.rotation = Quaternion.Slerp(
                    transform.rotation,
                    targetRot,
                    CurrentRotateSpeed * dt
                );
            }
        }
    }

    private void UpdateGroundedState()
    {
        Ray ray;
        ray.origin = transform.position + Vector3.up * 0.1f;
        ray.direction = Vector3.down;

        var hit = Physics.SphereCast(ray, SphereCastRadius, GroundCheckDistance, rb.collisionLayers, querySensors: false);
        if (hit.HasValue)
        {
            float angle = Vector3.Angle(hit.Value.normal, Vector3.up);
            bool isSteep = (angle > SlopeLimitAngle);

            bool isWalkable = Vector3.Dot(hit.Value.normal, Vector3.up) > 0.0f;

            IsGrounded = isWalkable && !isSteep;
        }
        else
        {
            //Gizmos.AddSphere(Color.red, ray.origin, SphereCastRadius);
            //Gizmos.AddSphere(Color.red, ray.origin + ray.direction * GroundCheckDistance, SphereCastRadius);

            IsGrounded = false;
        }
    }


    /// <summary>
    /// Processes IK for an individual foot:
    /// - Updates the foot IK blend value based on whether the character is grounded.
    /// - Computes an effective blend factoring in a height blend (to fade out IK when the foot is raised).
    /// - Raycasts from above the animated foot position to find the ground.
    /// - If a hit is found, computes the target position and applies IK with the appropriate blend.
    /// Returns true if a valid IK target was found.
    /// </summary>
    /// <param name="foot">The foot Entity to process IK for.</param>
    /// <param name="footIKBlend">A reference to that foot’s blend value.</param>
    /// <param name="dt">Delta time for the update.</param>
    /// <param name="ikTarget">The computed IK target in world space.</param>
    /// <param name="effectiveBlend">The effective IK blend (weight) after factoring in height differences.</param>
    /// <param name="footOffset">The vertical offset computed for hip adjustment.</param>
    /// <returns>True if the raycast hit (and thus IK is applied), false otherwise.</returns>
    private bool ProcessFootIK(Entity foot, ref float footIKBlend, float dt, 
                                 out Vector3 ikTarget, out float effectiveBlend, out float footOffset)
    {
        // Update the IK blend based on ground state.
        if (IsGrounded)
            footIKBlend += dt;
        else
            footIKBlend -= dt;


        footIKBlend = Mathf.Clamp(footIKBlend, 0f, 1f);

        // Get the current foot position.
        Vector3 footPos = foot.transform.position;

        float offsy = transform.position.y;
        float height = footPos.y - offsy;
                // Map the height between FootHeightMin and FootHeightMax to 0..1.
        float heightBlend = MapRange(height, FootHeightMin, FootHeightMax, 0.0f, 1.0f);
        // We want full IK (blend = 1) when the foot is low, so invert it.
        heightBlend = 1.0f - Mathf.Clamp01(heightBlend);
        // Compute a height blend: when the foot is raised, the IK influence fades out.
        // // InverseLerp returns 0 at FootHeightMin and 1 at FootHeightMax.
        // float heightBlend = 1f - Mathf.Clamp01(Mathf.InverseLerp(offsy + FootHeightMin, offsy + FootHeightMax, footPos.y));

        // // Combine the animation blend and the height blend.
        effectiveBlend = footIKBlend * heightBlend;

        // Prepare defaults.
        ikTarget = Vector3.zero;
        footOffset = 0f;

        // Raycast from above the foot downward to find the ground.
        Vector3 rayStart = footPos + Vector3.up * FootRaycastOffset;
        Ray ray = new Ray{origin = rayStart, direction = Vector3.down};
        var hit = Physics.Raycast(ray, 1.0f, rb.collisionLayers, false);

        if (hit.HasValue)
        {

            // Adjust target position upward to account for foot thickness.
            ikTarget = hit.Value.point + Vector3.up * FootTargetYOffset;
            
            // Compute the vertical offset that would adjust the Hips.
            footOffset = (ikTarget.y - footPos.y) * effectiveBlend;

            //Gizmos.AddSphere(Color.green, ikTarget, 0.1f * effectiveBlend );

            return true;
        }
        return false;
    }

    /// <summary>
    /// Calls ProcessFootIK for each foot and then computes a Hips offset from the results.
    /// </summary>
    private void ApplyFeetIK(float dt)
    {
        // Process left foot.
        bool leftHit = ProcessFootIK(LeftFoot, ref leftFootIKBlend, dt, 
                                     out Vector3 leftIKTarget, out float leftEffectiveBlend, out float leftOffset);

        // Process right foot.
        bool rightHit = ProcessFootIK(RightFoot, ref rightFootIKBlend, dt, 
                                      out Vector3 rightIKTarget, out float rightEffectiveBlend, out float rightOffset);

        // Compute the Hips vertical offset: take the minimum offset from the feet (to pull Hips down when needed).
        float offsetY = 0f;
        if (leftHit)
        {
            offsetY = Mathf.Min(offsetY, leftOffset);
        }
        if (rightHit)
        {
            offsetY = Mathf.Min(offsetY, rightOffset);
        }
        // Smoothly blend the Hips offset.
        hipsIKOffset = Mathf.Lerp(hipsIKOffset, offsetY, 0.1f);

        // Apply the Hips offset to adjust the Hips position.
        Vector3 HipsPos = Hips.transform.position;
        Hips.transform.position = new Vector3(HipsPos.x, HipsPos.y + hipsIKOffset, HipsPos.z);

        if(leftHit)
        {
            IK.SetIKPositionFabrik(LeftFoot, leftIKTarget, 2, 0.001f, 10);
            // IK.SetIKPositionTwoBone(LeftFoot, leftIKTarget, transform.forward);
        }

        if(rightHit)
        {
            IK.SetIKPositionFabrik(RightFoot, rightIKTarget, 2, 0.001f, 10);
            // IK.SetIKPositionTwoBone(RightFoot, rightIKTarget, transform.forward);

        }
    }

    /// <summary>
    /// (Optional) Adjusts the Spine orientation when the character is aiming.
    /// This mimics the Python callback that aligns the Spine to the camera forward.
    /// </summary>
    private void ApplySpineIK(float dt)
    {
        // 1) ramp global Spine & hand IK on/off
        bool isAiming = Input.IsDown(MouseButton.Right);
        float tgtSpineW = isAiming ? 1f : 0f;
        float tgtHandW  = isAiming ? 1f : 0f;

        spineIKWeight = Mathf.MoveTowards(spineIKWeight, tgtSpineW, SpineIKSpeed * dt);
        handIKWeight  = Mathf.MoveTowards(handIKWeight,  tgtHandW,  HandIKSpeed  * dt);

        if (spineIKWeight <= 0f && handIKWeight <= 0f)
            return;

        // 2) raycast from camera → mouse
        var camE = Scene.FindEntityByName("Main Camera");
        if (!camE.IsValid()) return;
        var cam = camE.GetComponent<CameraComponent>();
        cam.ScreenPointToRay(Input.mousePosition, out Ray ray);

        var hit = Physics.Raycast(ray, 500f, rb.collisionLayers, false);
        if (!hit.HasValue) return;

        Vector3 hitPoint = hit.Value.point;
        Vector3 lookDir  = (hitPoint - transform.position).normalized;

        // 3) forward‐cone check
        float fwdDot = Vector3.Dot(transform.forward, lookDir);
        if (fwdDot < Mathf.Cos(ForwardAngle * Mathf.Deg2Rad))
        {
            spineIKWeight = handIKWeight = 0f;
            return;
        }

        // 4) Spine IK
        if (spineIKWeight > 0f)
            IK.SetIKLookAtPosition(Spine, hitPoint, spineIKWeight);

        // 5) compute the signed side‐angle
        float sideAngle = Vector3.SignedAngle(transform.forward, lookDir, transform.up);
        float absAngle  = Mathf.Abs(sideAngle);

        // remap absAngle ∈ [start, end] → t ∈ [0, 1]
        float t = 0f;
        if (absAngle >= HandSwitchStartAngle)
            t = Mathf.InverseLerp(HandSwitchStartAngle, HandSwitchEndAngle, absAngle);

        // target weights: full on the “leading” hand, the other goes from 1→0 as t goes 0→1
        float targetR, targetL;
        if (sideAngle <  0f)  // target is to our right
        {
            targetR = handIKWeight;
            targetL = handIKWeight * (1f - t);
        }
        else  // target is to our left
        {
            targetL = handIKWeight;
            targetR = handIKWeight * (1f - t);
        }

        // 6) smooth toward those per‐hand targets
        righthandIKWeight = Mathf.MoveTowards(
            righthandIKWeight, targetR, HandIKSpeed * dt
        );
        lefthandIKWeight  = Mathf.MoveTowards(
            lefthandIKWeight,  targetL, HandIKSpeed * dt
        );

        // apply FABRIK if weight > 0
        if (righthandIKWeight > 0f)
        {
            Vector3 curr = RightHand.transform.position;
            Vector3 targ = Vector3.Lerp(curr, hitPoint, righthandIKWeight);
            IK.SetIKPositionFabrik(RightHand, targ, 2, 0.001f, 10);
        }
        if (lefthandIKWeight > 0f)
        {
            Vector3 curr = LeftHand.transform.position;
            Vector3 targ = Vector3.Lerp(curr, hitPoint, lefthandIKWeight);
            IK.SetIKPositionFabrik(LeftHand, targ, 2, 0.001f, 10);
        }

        if (Input.IsDown(KeyCode.E))
        {
            bool shootRight = righthandIKWeight >= lefthandIKWeight;
            Vector3 source = (shootRight ? RightHand : LeftHand).transform.position;
            Vector3 dir = (hitPoint - source).normalized;
            Shoot(source, dir, 2.4f, 10);
        }

        if (Input.IsDown(KeyCode.Q))
        {
            float radius = 5.0f;

            LayerMask layerMask = rb.collisionLayers; ;
            layerMask += LayerMask.NameToLayer("Projectiles");
            var hits = Physics.SphereOverlap(hitPoint,radius, layerMask);

            foreach (var e in hits)
            {
                var ph = e.GetComponent<PhysicsComponent>();
                if (ph != null)
                {
                    ph.ApplyExplosionForce(1.0f, hitPoint, radius, 0.0f, ForceMode.Impulse);
                }
            }
            // bool shootRight = righthandIKWeight >= lefthandIKWeight;
            // Vector3 source = (shootRight ? RightHand : LeftHand).transform.position;
            // Vector3 dir = (hitPoint - source).normalized;
            // Shoot(source, dir, 4.4f, 200);
        }

        // 7) shooting: pick whichever hand is more “on”
        if (Input.IsDown(MouseButton.Left))
        {
            bool shootRight = righthandIKWeight >= lefthandIKWeight;
            Vector3 source = (shootRight ? RightHand : LeftHand).transform.position;
            Vector3 dir = (hitPoint - source).normalized;
            Shoot(source, dir, 0.4f, 1);
        }
    }

    private float MapRange(float value, float inMin, float inMax, float outMin, float outMax)
    {
        return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
    }

    private void Shoot(Vector3 source, Vector3 shootDir, float spread, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            var instance = Scene.Instantiate(Projectice);
            instance.transform.parent = ProjecticeContainer;
            instance.transform.position = source + Random.insideUnitSphere * spread + shootDir * 0.5f;
            instance.transform.forward = shootDir;


            var iphysics = instance.GetComponent<PhysicsComponent>();
            iphysics.excludeLayers = owner.layers;
            iphysics.ApplyForce(shootDir * 25.0f, ForceMode.Impulse);
            Scene.DestroyEntity(instance, 5);
        }
        
    }

}
