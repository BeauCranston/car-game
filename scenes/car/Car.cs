using Godot;

public partial class Car : CharacterBody2D
{
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
    public float SteerAngle = 25f;

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

    private float SteerDirection;

    public override void _Ready()
    {
        if (AnimPlayer == null)
        {
            GD.PrintErr("AnimationPlayer is not assigned in the Inspector!");
        }

        AnimPlayer.Play("normal");
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
    }

    private void ApplyFriction()
    {
        if (Velocity.Length() < 5)
            Velocity = Vector2.Zero;

        var frictionForce = Velocity * Friction;
        var dragForce = Velocity * Velocity.Length() * Drag;

        _acceleration += dragForce + frictionForce;
    }

    private void HandleInput(float dt)
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
        SteerDirection = turn * Mathf.DegToRad(SteerAngle);
        if (Input.IsActionPressed("accelerate"))
        {
            _acceleration = Forward * EnginePower;
        }
        if (Input.IsActionPressed("brake"))
        {
            _acceleration = (Forward * BrakeForce);
            AnimPlayer.Play("braking");
        }
        else
        {
            AnimPlayer.Play("normal");
        }
    }

    private void CalculateSteering(float dt)
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

        rearWheel += forward * signedSpeed * dt;

        // Keep the negative sign if this fixed your mirrored steering.
        frontWheel += forward.Rotated(SteerDirection) * signedSpeed * dt;

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
