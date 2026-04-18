using System;
using Unravel.Core;

/// <summary>
/// Handles aim-based projectile firing and explosion abilities for a character.
///
/// Independent of the locomotion/IK controller: performs its own mouse raycast
/// to resolve the aim point, and picks which hand to fire from based on the
/// horizontal angle between the character's forward and the aim direction
/// (matches the convention used by the controller's aim IK).
///
/// Inputs (while right-mouse aim is held):
///   - Left mouse  -> Primary single-shot projectile (tight spread).
///   - E           -> Burst / shotgun-style spread (many projectiles, wide spread).
///   - Q           -> Explosive pulse at the aim hit point (applies impulse to
///                    every physics body in a radius, including projectiles).
///
/// The projectile prefab is expected to carry its own <see cref="PhysicsComponent"/>;
/// the shooter sets its <see cref="PhysicsComponent.excludeLayers"/> to the owner's
/// layers so the character never collides with its own shots, and applies an
/// impulse forward. Projectiles are auto-despawned after
/// <see cref="ProjectileLifetimeSeconds"/>.
/// </summary>
[ScriptSourceFile]
public class CharacterShooter : ScriptComponent
{
    // ---------------------------------------------------------------------
    // Projectile
    // ---------------------------------------------------------------------
    [Header("Projectile")]
    [Tooltip("Prefab spawned for each shot. Must carry a PhysicsComponent.")]
    public Prefab Projectile;

    [Tooltip("Seconds before a spawned projectile is destroyed.")]
    public int ProjectileLifetimeSeconds = 5;

    [Tooltip("Forward impulse applied to each spawned projectile.")]
    public float ProjectileImpulse = 25.0f;

    [Tooltip("Distance in front of the firing hand at which the projectile is spawned. " +
             "Keeps the initial position clear of the character's own capsule.")]
    public float ProjectileSpawnOffset = 0.5f;

    // ---------------------------------------------------------------------
    // Aim
    // ---------------------------------------------------------------------
    [Header("Aim")]
    [Tooltip("When true, right mouse button is required to enable all firing modes. " +
             "Set to false to allow 'hip-fire' without aiming.")]
    public bool RequireAimHeld = true;

    [Tooltip("Maximum raycast distance used to resolve the aim point.")]
    public float AimMaxDistance = 500.0f;

    [Tooltip("Maximum deviation (degrees) from the character's forward within which aim is allowed. " +
             "Aiming further to the rear disables fire to avoid over-twisted poses.")]
    [Range(0, 180)]
    public float MaxAimForwardAngle = 110.0f;

    // ---------------------------------------------------------------------
    // Fire modes
    // ---------------------------------------------------------------------
    [Header("Primary (Left Mouse)")]
    [Tooltip("Spread radius around the shoot direction for the primary fire. Smaller = tighter.")]
    public float PrimarySpread = 0.4f;

    [Tooltip("Number of projectiles spawned per primary shot.")]
    public int PrimaryCount = 1;

    [Header("Burst (E)")]
    [Tooltip("Spread radius for the burst / shotgun mode.")]
    public float BurstSpread = 2.4f;

    [Tooltip("Number of projectiles spawned per burst input.")]
    public int BurstCount = 10;

    [Header("Explosion (Q)")]
    [Tooltip("Radius (world units) of the explosion applied at the aim hit point.")]
    public float ExplosionRadius = 5.0f;

    [Tooltip("Explosion force magnitude, applied as an impulse to every affected body.")]
    public float ExplosionForce = 1.0f;

    [Tooltip("Name of the layer projectiles live on - included in the explosion sweep so " +
             "in-flight shots also get knocked around. Leave blank to skip.")]
    public string ProjectileLayerName = "Projectiles";

    // ---------------------------------------------------------------------
    // Hands
    // ---------------------------------------------------------------------
    [Header("Hands (auto-resolved for Mixamo rigs)")]
    public Entity LeftHand;
    public Entity RightHand;

    // ---------------------------------------------------------------------
    // Internal
    // ---------------------------------------------------------------------
    private CameraComponent mainCamera;
    private Entity projectileContainer;
    private LayerMask aimLayerMask;
    private bool hasAimLayerMask;

    public override void OnStart()
    {
        projectileContainer = Scene.CreateEntity("Projectiles");

        if (!LeftHand.IsValid())  LeftHand  = FindBone("mixamorig:LeftHand",  "LeftHand",  "hand.L");
        if (!RightHand.IsValid()) RightHand = FindBone("mixamorig:RightHand", "RightHand", "hand.R");

        ResolveCamera();

        // Prefer the CC's collision layers for aim raycasts (so we hit whatever the
        // character itself can collide with). Fallback handled in TryGetAimRay.
        CharacterControllerComponent cc = owner.GetComponent<CharacterControllerComponent>();
        if (cc != null)
        {
            aimLayerMask = cc.collisionLayers;
            hasAimLayerMask = true;
        }
    }

    public override void OnUpdate()
    {
        // Gate all firing on the aim input. Early-out if not aiming, to avoid
        // paying for a raycast every frame.
        if (RequireAimHeld && !Input.IsDown(MouseButton.Right))
            return;

        if (mainCamera == null)
        {
            ResolveCamera();
            if (mainCamera == null) return;
        }

        if (!mainCamera.ScreenPointToRay(Input.mousePosition, out Ray ray))
            return;

        LayerMask mask = hasAimLayerMask ? aimLayerMask : new LayerMask();
        RaycastHit? hit = Physics.Raycast(ray, AimMaxDistance, mask, false);
        if (!hit.HasValue)
            return;

        Vector3 hitPoint = hit.Value.point;

        // Forward cone gate: firing should match the aim IK's acceptance cone
        // so we never fire from a hand that isn't reaching the target.
        Vector3 lookDir = hitPoint - transform.position;
        lookDir.y = 0.0f;
        if (lookDir.sqrMagnitude < 0.0001f)
            return;
        lookDir = lookDir.normalized;
        float forwardDot = Vector3.Dot(transform.forward, lookDir);
        if (forwardDot < Mathf.Cos(MaxAimForwardAngle * Mathf.Deg2Rad))
            return;

        if (Input.IsDown(MouseButton.Left))
            FireFromLeadingHand(hitPoint, PrimarySpread, PrimaryCount);

        if (Input.IsDown(KeyCode.E))
            FireFromLeadingHand(hitPoint, BurstSpread, BurstCount);

        if (Input.IsDown(KeyCode.Q))
            Detonate(hitPoint, mask);
    }

    // ---------------------------------------------------------------------
    // Firing
    // ---------------------------------------------------------------------
    private void FireFromLeadingHand(Vector3 aimPoint, float spread, int count)
    {
        Entity hand = PickLeadingHand(aimPoint);
        if (!hand.IsValid())
            return;

        Vector3 source = hand.transform.position;
        Vector3 dir = aimPoint - source;
        if (dir.sqrMagnitude < 0.0001f) return;
        dir = dir.normalized;
        Shoot(source, dir, spread, count);
    }

    /// <summary>
    /// Picks the hand that is on the same side as the aim target. Mirrors the
    /// controller's aim-IK side convention: SignedAngle(forward, look, up) &lt; 0
    /// means the target is to the character's right.
    /// </summary>
    private Entity PickLeadingHand(Vector3 aimPoint)
    {
        Vector3 lookDir = aimPoint - transform.position;
        lookDir.y = 0.0f;
        if (lookDir.sqrMagnitude < 0.0001f)
            return RightHand.IsValid() ? RightHand : LeftHand;

        float sideAngle = Vector3.SignedAngle(transform.forward, lookDir.normalized, transform.up);
        Entity pick = sideAngle < 0.0f ? RightHand : LeftHand;
        if (!pick.IsValid())
            pick = RightHand.IsValid() ? RightHand : LeftHand;
        return pick;
    }

    private void Shoot(Vector3 source, Vector3 shootDir, float spread, int count)
    {
        if (Projectile == null)
            return;

        for (int i = 0; i < count; ++i)
        {
            Entity instance = Scene.Instantiate(Projectile);
            instance.transform.parent = projectileContainer;
            instance.transform.position = source + Random.insideUnitSphere * spread + shootDir * ProjectileSpawnOffset;
            instance.transform.forward = shootDir;

            PhysicsComponent physics = instance.GetComponent<PhysicsComponent>();
            if (physics != null)
            {
                physics.excludeLayers = owner.layers;
                physics.ApplyForce(shootDir * ProjectileImpulse, ForceMode.Impulse);
            }

            Scene.DestroyEntity(instance, ProjectileLifetimeSeconds);
        }
    }

    private void Detonate(Vector3 center, LayerMask baseMask)
    {
        LayerMask mask = baseMask;
        if (!string.IsNullOrEmpty(ProjectileLayerName))
            mask += LayerMask.NameToLayer(ProjectileLayerName);

        Entity[] hits = Physics.SphereOverlap(center, ExplosionRadius, mask);
        if (hits == null) return;

        foreach (Entity e in hits)
        {
            PhysicsComponent ph = e.GetComponent<PhysicsComponent>();
            if (ph != null)
                ph.ApplyExplosionForce(ExplosionForce, center, ExplosionRadius, 0.0f, ForceMode.Impulse);
        }
    }

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------
    private Entity FindBone(params string[] names)
    {
        foreach (string name in names)
        {
            Entity e = transform.FindChild(name, true);
            if (e.IsValid())
                return e;
        }
        return Entity.Invalid;
    }

    private void ResolveCamera()
    {
        Entity camEntity = Scene.FindEntityByName("Main Camera");
        if (!camEntity.IsValid()) return;
        mainCamera = camEntity.GetComponent<CameraComponent>();
    }
}
