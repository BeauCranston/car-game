using Godot;

/// <summary>
///  Car class that drives from a top down view. When the car goes forward it's Y position is negative, and when it goes backwards the Y position is positive
/// </summary>
/// <param name="parameterName">Parameter description.</param>
/// <returns>Type and description of the returned object.</returns>
/// <example>Write me later.</example>
public partial class Car2 : CharacterBody2D
{
    [ExportGroup("Effects")]
    [Export]
    public PackedScene CarSmokeEffect;

    [Export]
    public PackedScene TireTrackEffect;

    [Export]
    public PackedScene ExhaustEffect;

    [Export]
    public NodePath EffectsParentPath;

    [Export]
    public float TireTrackEffectInterval = 0.2f;

    [Export]
    public float ExhaustEffectInterval = 0.15f;

    [Export]
    public float CarSmokeEffectInterval = 10f;

    [ExportGroup("Units")]
    [Export]
    public float PixelsPerMeter = 100f;

    [ExportGroup("Vehicle")]
    [Export]
    public float Mass = 1200f;

    [Export]
    public float EngineTorque = 9000f;

    [Export]
    public float BrakeTorque = 12000f;

    [Export]
    public float MaxSpeed = 900f;

    [ExportSubgroup("Steering")]
    [Export]
    public float SteerSpeed = 2.5f;

    [Export]
    public float MaxSteerAngle = 0.65f;

    [ExportSubgroup("Tire")]
    [Export]
    public float TireRadius = 0.34f;

    [Export]
    public float WheelInertia = 1.2f;

    [Export]
    public float WheelAngularDrag = 0.5f;

    [ExportGroup("Game Feel")]
    [Export]
    public float LateralGrip = 8f;

    [Export]
    public float RollingResistance = 0.99f;

    [Export]
    public float MaxSpeedPixels = 1200f;

    private float _gravity = 9.81f;

    private float Weight
    {
        get => Mass * _gravity;
    }
    private float NominalLoadOnTire
    {
        get => Weight / 4;
    }
    private float _tireTrackTimer = 0f;
    private float _carSmokeTimer = 0f;
    private float _exhaustTimer = 0f;

    private Node2D _effectsParent;
    private Marker2D _carSmokeSpawnLeft;
    private Marker2D _carSmokeSpawnRight;
    private Marker2D _exhaustSpawn;
    private Marker2D _tireTrackSpawnLeft;
    private Marker2D _tireTrackSpawnRight;

    private float _wheelAngularVelocity = 0;
    private float _wheelRadius = 30f;
    private float _slipDenominatorLowerBoundary = 0.5f;

    private float _wheelSlipLowerBoundary = -1;
    private float _wheelSlipUpperBoundary = 1;

    // private float WheelSurfaceSpeed
    // {
    //     get => _wheelRadius * _wheelAngularVelocity;
    // }

    private float VerticalLoadOnTire { get; set; }

    [Signal]
    public delegate void VelocityChangedEventHandler(Vector2 speed);

    [Export]
    public AnimationPlayer AnimPlayer { get; set; }

    private Gear _gear = Gear.Forward;
    private int GearDirection => _gear == Gear.Forward ? 1 : -1;

    public override void _Ready()
    {
        ZIndex = 10;

        if (AnimPlayer == null)
        {
            GD.PrintErr("AnimationPlayer is not assigned in the Inspector!");
        }

        AnimPlayer.Play("normal");
        _exhaustSpawn = GetNode<Marker2D>("ExhaustSpawn");
        _carSmokeSpawnLeft = GetNode<Marker2D>("CarSmokeSpawnLeft");
        _carSmokeSpawnRight = GetNode<Marker2D>("CarSmokeSpawnRight");
        _tireTrackSpawnLeft = GetNode<Marker2D>("TireTrackSpawnLeft");
        _tireTrackSpawnRight = GetNode<Marker2D>("TireTrackSpawnRight");
        _effectsParent = GetNodeOrNull<Node2D>(EffectsParentPath);
        _exhaustTimer = ExhaustEffectInterval;
        _tireTrackTimer = TireTrackEffectInterval;
        _carSmokeTimer = CarSmokeEffectInterval;

        if (_effectsParent == null)
        {
            _effectsParent = GetTree().CurrentScene as Node2D;
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        float dt = (float)delta;

        Vector2 forward = -Transform.Y;
        Vector2 right = Transform.X;

        // 1. Apply user input as torque to the wheel.
        float throttle = Input.IsActionPressed("accelerate") ? 1f : 0f;
        float brake = Input.IsActionPressed("brake") ? 1f : 0f;
        float steerInput =
            (Input.IsActionPressed("steer_right") ? 1f : 0f)
            - (Input.IsActionPressed("steer_left") ? 1f : 0f);
        float carForwardSpeedPixels = Velocity.Dot(forward);
        float carForwardSpeed = carForwardSpeedPixels / PixelsPerMeter;

        GD.Print($"carForwardSpeed: {carForwardSpeed}");

        GD.Print($"wheelAngularVelocity: {_wheelAngularVelocity}");
        ApplyWheelInput(throttle, brake, dt);
        float wheelSurfaceSpeed = _wheelAngularVelocity * TireRadius;
        // 3. Calculate tire slip.
        float slip = CalculateWheelSlip(carForwardSpeed, wheelSurfaceSpeed);
        // 4. Calculate tire force from Magic Formula.
        float forcePerTire = GetLongitudinalForce(
            slip,
            NominalLoadOnTire,
            SurfaceConditions.DryTarmac
        );
        // 5. Apply tire force to the car.
        Vector2 accelerationMeters = forward * (forcePerTire / Mass);
        Vector2 accelerationPixels = accelerationMeters * PixelsPerMeter;
        Velocity += accelerationPixels * dt;
        // 6. Apply reaction torque back to the wheel.
        //
        // The ground force pushes the car forward, but it also resists
        // the wheel spin. This prevents the wheel from spinning forever.
        // float reactionTorque = forcePerTire * TireRadius;
        // _wheelAngularVelocity -= reactionTorque / WheelInertia * dt;
        // 7. Simple lateral grip so the car does not slide sideways forever.
        float lateralSpeed = Velocity.Dot(right);
        Velocity += -right * lateralSpeed * LateralGrip * dt;

        // 8. Rolling resistance / general damping.
        Velocity *= Mathf.Pow(RollingResistance, dt * 60f);

        // 9. Simple steering.
        float speedFactor = Mathf.Clamp(Mathf.Abs(carForwardSpeedPixels) / 400f, 0f, 1f);
        Rotation += steerInput * speedFactor * 2.5f * dt;

        MoveAndSlide();
        GD.Print(
            $"Vx: {carForwardSpeed}, wheel: {wheelSurfaceSpeed}, slip: {slip}, tireForce: {forcePerTire}"
        );
    }

    private void UpdateGear()
    {
        if (Input.IsActionJustPressed("gear_reverse"))
        {
            _gear = _gear == Gear.Forward ? Gear.Reverse : Gear.Forward;
        }
    }

    private void ApplyWheelInput(float throttle, float brake, float dt)
    {
        int gearDirection = GearDirection;
        float driveTorque = throttle * EngineTorque * gearDirection;

        _wheelAngularVelocity += driveTorque / WheelInertia * dt;

        // Brake slows wheel rotation toward zero.
        // It does NOT reverse the wheel.
        if (brake > 0f)
        {
            float brakeAngularDecel = brake * BrakeTorque / WheelInertia * dt;

            _wheelAngularVelocity = Mathf.MoveToward(_wheelAngularVelocity, 0f, brakeAngularDecel);
        }

        // Rotational drag.
        _wheelAngularVelocity = Mathf.MoveToward(_wheelAngularVelocity, 0f, WheelAngularDrag * dt);
    }

    private float CalculateWheelSlip(float carForwardSpeed, float wheelSurfaceSpeed)
    {
        float slipVelocity = wheelSurfaceSpeed - carForwardSpeed;

        GD.Print($"Slip Velocity: {slipVelocity}");
        // Protect against division by zero at low speed.
        float denominator = Mathf.Max(Mathf.Abs(carForwardSpeed), 0.5f);

        float slip = slipVelocity / denominator;

        return Mathf.Clamp(slip, -1f, 1f);
    }

    private float GetLongitudinalForce(float slip, float loadOnTire, SurfaceCondition surface)
    {
        float bk = surface.B * slip;

        return loadOnTire
            * surface.D
            * Mathf.Sin(surface.C * Mathf.Atan(bk - surface.E * (bk - Mathf.Atan(bk))));
    }

    private float GetSpeedChange(float delta, float currentSpeed, float newSpeed)
    {
        return Mathf.Abs(newSpeed) - Mathf.Abs(currentSpeed);
    }

    private void SpawnCarExhaust(float delta)
    {
        if (_exhaustTimer <= 0)
        {
            SpawnEffect(ExhaustEffect, _exhaustSpawn.GlobalPosition, GlobalRotation);
            _exhaustTimer = ExhaustEffectInterval;
        }
        _exhaustTimer -= delta;
    }

    private void SpawnCarSmoke(float delta)
    {
        if (_carSmokeTimer <= 0)
        {
            SpawnEffect(CarSmokeEffect, _carSmokeSpawnLeft.GlobalPosition, GlobalRotation);
            SpawnEffect(CarSmokeEffect, _carSmokeSpawnRight.GlobalPosition, GlobalRotation);
            _carSmokeTimer = CarSmokeEffectInterval;
        }
        _carSmokeTimer -= delta;
    }

    private void SpawnTireTracks(float delta)
    {
        // if (_speedChange < -15)
        // {
        //     SpawnEffect(TireTrackEffect, _tireTrackSpawnLeft.GlobalPosition, GlobalRotation);
        //     SpawnEffect(TireTrackEffect, _tireTrackSpawnRight.GlobalPosition, GlobalRotation);
        //     _tireTrackTimer = TireTrackEffectInterval;
        // }
        // _tireTrackTimer -= delta;
    }

    private void SpawnEffect(PackedScene scene, Vector2 position, float rotation)
    {
        if (scene == null)
            return;

        Node2D effect = scene.Instantiate<Node2D>();

        effect.GlobalPosition = position;
        effect.GlobalRotation = rotation;
        _effectsParent.AddChild(effect);
    }

    private void CalculateSteering(float delta) { }
}
