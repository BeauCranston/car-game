using Godot;

/// <summary>
///  Car class that drives from a top down view. When the car goes forward it's Y position is negative, and when it goes backwards the Y position is positive
/// </summary>
/// <param name="parameterName">Parameter description.</param>
/// <returns>Type and description of the returned object.</returns>
/// <example>Write me later.</example>
public partial class Car3 : RigidBody2D
{
    [Export]
    public float PixelsPerMeter { get; set; } = 100f;

    [Export]
    public AnimationPlayer AnimPlayer { get; set; }

    // [ExportGroup("Vehicle")]
    [Export]
    public float Mass = 1500f;

    //
    // [Export]
    // public float EngineTorque = 9000f;
    //
    // [Export]
    // public float BrakeTorque = 12000f;
    //
    // [Export]
    // public float MaxSpeed = 900f;
    //
    [ExportSubgroup("Steering")]
    // [Export]
    // public float SteerSpeed = 2.5f;

    [Export]
    public float MaxSteerAngle = 0.5f;

    [ExportSubgroup("Tire")]
    [Export]
    public float TireRadius = 0.34f;

    [Export]
    public float WheelInertia = 1.2f;

    // [Export]
    // public float WheelAngularDrag = 0.5f;

    // [ExportGroup("Game Feel")]
    // [Export]
    // public float LateralGrip = 8f;

    // [Export]
    // public float RollingResistance = 0.99f;

    // [Export]
    // public float MaxSpeedPixels = 1200f;

    private float _gravity = 9.81f;

    private float Weight
    {
        get => Mass * _gravity;
    }
    private float NominalLoadOnTire
    {
        get => Weight / 4;
    }

    [ExportGroup("WheelPositions")]
    [Export]
    private Marker2D _frontLeftWheel;

    [Export]
    private Marker2D _frontRightWheel;

    [Export]
    private Marker2D _rearLeftWheel;

    [Export]
    private Marker2D _rearRightWheel;

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

    private Gear _gear = Gear.Forward;
    private int GearDirection => _gear == Gear.Forward ? 1 : -1;

    [ExportGroup("Vehicle")]
    [Export]
    private float _maxEngineTorque = 300f; // Newton-meters

    [Export]
    private float _gearRatio = 3.5f; // Current gear ratio

    [Export]
    private float _finalDriveRatio = 4.1f; // Differential ratio

    [Export]
    private float _brakeTorque = 1500f; // Max braking power

    // Track the independent angular velocities (rad/s) of each wheel
    private float _omegaFL,
        _omegaFR,
        _omegaRL,
        _omegaRR;
    private float _steeringAngle = 0f;

    public override void _Ready()
    {
        ZIndex = 10;

        if (AnimPlayer == null)
        {
            GD.PrintErr("AnimationPlayer is not assigned in the Inspector!");
        }

        AnimPlayer.Play("normal");
        // _frontLeftWheel = GetNode<Marker2D>("TireFrontLeft");
        // _frontRightWheel = GetNode<Marker2D>("TireFrontRight");
        // _rearLeftWheel = GetNode<Marker2D>("TireBackLeft");
        // _rearRightWheel = GetNode<Marker2D>("TireBackRight");
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
        _steeringAngle = steerInput * MaxSteerAngle;

        // 2. Calculate Drivetrain Torques
        // Total torque multiplier through transmission
        float driveTorque = throttle * _maxEngineTorque * _gearRatio * _finalDriveRatio;

        // Split torque across the drive wheels (Rear-Wheel Drive layout used here)
        float driveTorquePerWheel = driveTorque / 2f;

        // 3. Calculate Static Vertical Load (Per-wheel allocation of vehicle weight)
        // GD.Print(
        //     $"throttle:{throttle}, driveTorque:{driveTorque}, totalWeight:${Weight}, loadOnTire:{NominalLoadOnTire}"
        // );

        // 4. Process Each Wheel Individually
        ProcessWheel(
            _frontLeftWheel,
            ref _omegaFL,
            _steeringAngle,
            0f,
            brake,
            NominalLoadOnTire,
            dt
        );
        ProcessWheel(
            _frontRightWheel,
            ref _omegaFR,
            _steeringAngle,
            0f,
            brake,
            NominalLoadOnTire,
            dt
        );
        ProcessWheel(
            _rearLeftWheel,
            ref _omegaRL,
            0f,
            driveTorquePerWheel,
            brake,
            NominalLoadOnTire,
            dt
        );
        ProcessWheel(
            _rearRightWheel,
            ref _omegaRR,
            0f,
            driveTorquePerWheel,
            brake,
            NominalLoadOnTire,
            dt
        );
    }

    private void ProcessWheel(
        Marker2D wheel,
        ref float wheelOmega,
        float steerAngle,
        float driveTorque,
        float brakeInput,
        float fz,
        float dt
    )
    {
        // 1. Orient wheel vectors natively (0 rad points UP -Y)
        float totalAngle = GlobalRotation + steerAngle - (Mathf.Pi / 2f);
        Vector2 wheelForward = Vector2.FromAngle(totalAngle);

        // 2. Manage free-rolling wheels
        if (driveTorque == 0f && brakeInput == 0f)
        {
            Vector2 wVel = GetVelocityAtWheel(wheel);
            float groundSpeed = wVel.Dot(wheelForward);
            wheelOmega = groundSpeed / TireRadius;
        }

        // 3. Handle Prospective Braking Lockup
        float prospectiveAppliedBrakeTorque = 0f;
        bool isWheelLockedByBrakes = false;

        if (brakeInput > 0f && Mathf.Abs(wheelOmega) > 0.01f)
        {
            float brakeDirection = -Mathf.Sign(wheelOmega);
            prospectiveAppliedBrakeTorque = brakeInput * _brakeTorque * brakeDirection;

            // Calculate if this frame's torque will push rotation past zero
            Vector2 preVel = GetVelocityAtWheel(wheel);
            var preResult = TirePhysics.CalculateTireForces(
                preVel,
                wheelForward,
                wheelOmega,
                TireRadius,
                fz
            );
            float preFeedback = preResult.forces.Dot(wheelForward) * TireRadius;

            float netTorque = driveTorque - preFeedback + prospectiveAppliedBrakeTorque;
            float prospectiveDeltaOmega = (netTorque / WheelInertia) * dt;

            if (Mathf.Sign(wheelOmega) != Mathf.Sign(wheelOmega + prospectiveDeltaOmega))
            {
                wheelOmega = 0f;
                isWheelLockedByBrakes = true; // Flag to skip manual torque integration later
            }
        }

        // 4. Gather true ground velocity & calculate final tire forces
        Vector2 wheelVelocity = GetVelocityAtWheel(wheel);
        var result = TirePhysics.CalculateTireForces(
            wheelVelocity,
            wheelForward,
            wheelOmega,
            TireRadius,
            fz
        );

        // 5. CRITICAL FIX: Always pass forces to Godot's engine, even if wheel is locked!
        Vector2 pixelForce = result.forces * PixelsPerMeter;
        ApplyForce(pixelForce, wheel.Position);

        // If the wheel locked up this frame, stop here. The skidding force was already applied above.
        if (isWheelLockedByBrakes || (brakeInput > 0f && Mathf.Abs(wheelOmega) <= 0.01f))
        {
            wheelOmega = 0f;
            return;
        }

        // 6. Normal Wheel Rotational Physics Integration
        float fxTire = result.forces.Dot(wheelForward);
        float tireFeedbackTorque = fxTire * TireRadius;

        float totalTorque = driveTorque - tireFeedbackTorque + prospectiveAppliedBrakeTorque;
        float angularAcceleration = totalTorque / WheelInertia;

        wheelOmega += angularAcceleration * dt;

        // Prevent numeric explosion clamp
        wheelOmega = Mathf.Clamp(wheelOmega, -300f, 300f);
    }

    private Vector2 GetVelocityAtWheel(Marker2D wheelMarker)
    {
        Vector2 localPos = wheelMarker.Position;
        Vector2 tangentialVelocity = new Vector2(-localPos.Y, localPos.X) * AngularVelocity;

        //magic forumla expects meters per second so need to convert from pixel velocity to real velocity
        return (LinearVelocity + tangentialVelocity) / PixelsPerMeter;
    }

    private void UpdateGear()
    {
        if (Input.IsActionJustPressed("gear_reverse"))
        {
            _gear = _gear == Gear.Forward ? Gear.Reverse : Gear.Forward;
        }
    }
}
