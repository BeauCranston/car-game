using Godot;

/// <summary>
///  Car class that drives from a top down view. When the car goes forward it's Y position is negative, and when it goes backwards the Y position is positive
/// </summary>
/// <param name="parameterName">Parameter description.</param>
/// <returns>Type and description of the returned object.</returns>
/// <example>Write me later.</example>
public partial class Car : CharacterBody2D
{
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

    private float _tireTrackTimer = 0f;
    private float _carSmokeTimer = 0f;
    private float _exhaustTimer = 0f;

    private Node2D _effectsParent;
    private Marker2D _carSmokeSpawnLeft;
    private Marker2D _carSmokeSpawnRight;
    private Marker2D _exhaustSpawn;
    private Marker2D _tireTrackSpawnLeft;
    private Marker2D _tireTrackSpawnRight;

    [Signal]
    public delegate void VelocityChangedEventHandler(Vector2 speed);

    [Export]
    public float BrakeForce = -400f;

    [Export]
    public float MaxReverseSpeed = 250f;

    [Export]
    public float EnginePower = 900f;

    [Export]
    public float Friction = -0.7f;

    [Export]
    public float Drag = -0.0001f;

    [Export]
    public float WheelBase = 100f;

    [Export]
    public float SteerAngle = 15f;

    [Export]
    public float SlipSpeed = 400f;

    [Export]
    public float TractionFast = 0.5f;

    [Export]
    public float TractionSlow = 1.0f;

    [Export]
    public AnimationPlayer AnimPlayer { get; set; }

    private Vector2 _acceleration = Vector2.Zero;
    private Vector2 Forward => -Transform.Y;

    private float _speedChange = 0;

    private float _steerDirection;

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
        _acceleration = Vector2.Zero;
        HandleInput(dt);
        ApplyFriction();
        Velocity += _acceleration * dt;
        CalculateSteering(dt);
        MoveAndSlide();
        EmitSignal(SignalName.VelocityChanged, Velocity);
        GD.Print(Velocity.Length());
        // GD.Print(GlobalRotationDegrees);
    }

    private void ApplyFriction()
    {
        if (Velocity.Length() < 5)
            Velocity = Vector2.Zero;

        var frictionForce = Velocity * Friction;
        var dragForce = Velocity * Velocity.Length() * Drag;

        _acceleration += dragForce + frictionForce;
    }

    private void HandleInput(float delta)
    {
        var turn = 0;

        if (Input.IsActionPressed("steer_right"))
        {
            turn += 1;
        }
        if (Input.IsActionPressed("steer_left"))
        {
            turn -= 1;
        }
        _steerDirection = turn * Mathf.DegToRad(SteerAngle);
        if (Input.IsActionPressed("accelerate"))
        {
            _acceleration = Forward * EnginePower;
            SpawnCarExhaust(delta);
        }
        if (Input.IsActionPressed("brake"))
        {
            _acceleration = (Forward * BrakeForce);
            AnimPlayer.Play("braking");
            SpawnTireTracks(delta);
        }
        if (Input.IsActionJustReleased("brake"))
        {
            AnimPlayer.Play("normal");
        }
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
        if (_tireTrackTimer <= 0)
        {
            SpawnEffect(TireTrackEffect, _tireTrackSpawnLeft.GlobalPosition, GlobalRotation);
            SpawnEffect(TireTrackEffect, _tireTrackSpawnRight.GlobalPosition, GlobalRotation);
            _tireTrackTimer = TireTrackEffectInterval;
        }
        _tireTrackTimer -= delta;
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

    private void CalculateSteering(float delta)
    {
        // Save the current speed BEFORE changing heading.
        var speedBeforeSteering = Velocity.Length();
        if (Velocity.Length() < 0.1f)
            return;

        var forward = Forward;

        var rearWheel = Position - forward * (WheelBase / 2);
        var frontWheel = Position + forward * (WheelBase / 2);

        // Signed speed tells us whether we are moving forward or reverse.
        var signedSpeed = Velocity.Dot(forward);

        rearWheel += forward * signedSpeed * delta;

        // Keep the negative sign if this fixed your mirrored steering.
        frontWheel += forward.Rotated(_steerDirection) * signedSpeed * delta;

        var newHeading = (frontWheel - rearWheel).Normalized();

        var traction = speedBeforeSteering > SlipSpeed ? TractionFast : TractionSlow;

        if (signedSpeed >= 0)
        {
            // Forward: keep the same speed, only steer the direction.
            Velocity = Velocity.Lerp(newHeading * speedBeforeSteering, traction);
        }
        else
        {
            // Reverse: keep same speed, but move opposite the car's heading.
            Velocity = Velocity.Lerp(
                -newHeading * Mathf.Min(speedBeforeSteering, MaxReverseSpeed),
                traction
            );
        }
        // Important: steering is not allowed to increase speed.
        if (Velocity.Length() > 0.001f)
        {
            Velocity = Velocity.Normalized() * speedBeforeSteering;
        }

        Rotation = newHeading.Angle() + Mathf.Pi / 2f;
    }
}
