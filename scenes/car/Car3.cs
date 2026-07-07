using Godot;

/// <summary>
///  Car class that drives from a top down view. When the car goes forward it's Y position is negative, and when it goes backwards the Y position is positive
/// </summary>
/// <param name="parameterName">Parameter description.</param>
/// <returns>Type and description of the returned object.</returns>
/// <example>Write me later.</example>
public partial class Car3 : RigidBody2D
{
    [ExportGroup("Visuals & Chassis")]
    [Export]
    private Node2D _carVisual; // Assign the main car Sprite2D/Node2D here!

    [Export]
    private float _bodyRollStiffness = 0.05f; // Controls how stiff the suspension resists roll

    [Export]
    private float _bodyRollDamping = 0.15f; // Dampens body bounce vibrations

    private float _currentBodyRollAngle = 0f;
    private float _bodyRollVelocity = 0f;

    // Variable to track longitudinal load shift for accurate spin-outs
    private float _longitudinalWeightTransfer = 0f;

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
    public float MaxSteerAngle = 0.65f;

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

    private float Wheelbase =>
        Mathf.Abs(_frontLeftWheel.Position.Y - _rearLeftWheel.Position.Y) / PixelsPerMeter;
    private float TrackWidth =>
        Mathf.Abs(_frontLeftWheel.Position.X - _frontRightWheel.Position.X) / PixelsPerMeter;
    private float CenterOfMassHeight = 0.5f; // Estimated height of car's CG in meters
    private Vector2 _previousLinearVelocity = Vector2.Zero;

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

        // 1. Process Core Inputs
        float throttle = Input.IsActionPressed("accelerate") ? 1f : 0f;
        float brake = Input.IsActionPressed("brake") ? 1f : 0f;
        float steerInput =
            (Input.IsActionPressed("steer_right") ? 1f : 0f)
            - (Input.IsActionPressed("steer_left") ? 1f : 0f);

        float targetSteerAngle = steerInput * MaxSteerAngle;

        // 2. Ackerman Steering Calculations
        float steerLeft = 0f;
        float steerRight = 0f;

        if (Mathf.Abs(targetSteerAngle) > 0.001f)
        {
            float radius = Wheelbase / Mathf.Tan(Mathf.Abs(targetSteerAngle));
            if (targetSteerAngle > 0f) // Right turn
            {
                steerRight =
                    Mathf.Atan(Wheelbase / (radius - (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
                steerLeft =
                    Mathf.Atan(Wheelbase / (radius + (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
            }
            else // Left turn
            {
                steerLeft =
                    Mathf.Atan(Wheelbase / (radius - (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
                steerRight =
                    Mathf.Atan(Wheelbase / (radius + (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
            }
        }

        // 3. True 2D Acceleration & Force Extraction
        Vector2 currentVelocityMps = LinearVelocity / PixelsPerMeter;
        Vector2 accelerationMps = (currentVelocityMps - _previousLinearVelocity) / dt;
        _previousLinearVelocity = currentVelocityMps;

        // Convert acceleration vectors into the car's local grid space
        Vector2 localAccel = accelerationMps.Rotated(-GlobalRotation);
        float lateralAcceleration = localAccel.X;
        float longitudinalAcceleration = localAccel.Y;

        // 4. Dynamic Weight Transfer (Lateral + Longitudinal Matrix)
        float lateralWeightTransfer =
            (Mass * lateralAcceleration * CenterOfMassHeight) / TrackWidth;
        _longitudinalWeightTransfer =
            (Mass * longitudinalAcceleration * CenterOfMassHeight) / Wheelbase;

        // Safely clamp transfer matrices to protect against calculation explosion
        lateralWeightTransfer = Mathf.Clamp(
            lateralWeightTransfer,
            -NominalLoadOnTire * 1.5f,
            NominalLoadOnTire * 1.5f
        );
        _longitudinalWeightTransfer = Mathf.Clamp(
            _longitudinalWeightTransfer,
            -NominalLoadOnTire * 1.5f,
            NominalLoadOnTire * 1.5f
        );

        // Calculate four-corner dynamic tire load mapping
        // Accelerating shifts weight backward (subtracts from front, adds to rear)
        float fzFL =
            NominalLoadOnTire - (lateralWeightTransfer / 2f) - (_longitudinalWeightTransfer / 2f);
        float fzFR =
            NominalLoadOnTire + (lateralWeightTransfer / 2f) - (_longitudinalWeightTransfer / 2f);
        float fzRL =
            NominalLoadOnTire - (lateralWeightTransfer / 2f) + (_longitudinalWeightTransfer / 2f);
        float fzRR =
            NominalLoadOnTire + (lateralWeightTransfer / 2f) + (_longitudinalWeightTransfer / 2f);

        // Guard wheels against dropping below minimum contact load (0 Newtons)
        fzFL = Mathf.Max(10f, fzFL);
        fzFR = Mathf.Max(10f, fzFR);
        fzRL = Mathf.Max(10f, fzRL);
        fzRR = Mathf.Max(10f, fzRR);

        // 5. Motor Calculations
        // Calculate Drivetrain Torques
        float driveTorque =
            throttle * _maxEngineTorque * _gearRatio * _finalDriveRatio * GearDirection;
        float driveTorquePerWheel = driveTorque / 2f;

        // ADD BRAKE BIAS: Send 65% of braking power to the front, 35% to the rear
        // Or lower the front bias slightly if the front wheels still lock up too easily
        float frontBrakeInput = brake * 0.60f;
        float rearBrakeInput = brake * 0.40f;

        // Execute Wheel Simulation Framework with biased brake forces
        ProcessWheel(_frontLeftWheel, ref _omegaFL, steerLeft, 0f, frontBrakeInput, fzFL, dt);
        ProcessWheel(_frontRightWheel, ref _omegaFR, steerRight, 0f, frontBrakeInput, fzFR, dt);
        ProcessWheel(
            _rearLeftWheel,
            ref _omegaRL,
            0f,
            driveTorquePerWheel,
            rearBrakeInput,
            fzRL,
            dt
        );
        ProcessWheel(
            _rearRightWheel,
            ref _omegaRR,
            0f,
            driveTorquePerWheel,
            rearBrakeInput,
            fzRR,
            dt
        );
        // 7. PHYSICS-BASED CHASSIS VISUAL BODY ROLL
        if (_carVisual != null)
        {
            // Spring Hooke's Law equation: Accel = (-Stiffness * Position) - (Damping * Velocity)
            // Driven by real-world lateral G forces (lateralAcceleration)
            float targetForce = -lateralAcceleration * 0.002f;
            float springForce = (targetForce - _currentBodyRollAngle) * _bodyRollStiffness;
            _bodyRollVelocity += springForce - (_bodyRollVelocity * _bodyRollDamping);
            _currentBodyRollAngle += _bodyRollVelocity;

            // Apply visual physics transforms to the car sprite layer
            _carVisual.Rotation = _currentBodyRollAngle;

            // Dynamic pitch compression: backend drops down on throttle, front drops on brakes
            float pitchOffset = -longitudinalAcceleration * 0.001f;
            _carVisual.Skew = pitchOffset;
        }
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
        // 1. FIXED: Natively derive wheel direction using car transforms
        // GlobalTransform.Y is the car's forward axis (negative is up).
        // We rotate it by the steering angle to get the exact world-space direction.
        Vector2 wheelForward = (-GlobalTransform.Y).Rotated(steerAngle).Normalized();

        GD.Print($"WheelForward: {wheelForward}");
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
                isWheelLockedByBrakes = true;
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

        // 5. Apply forces using the wheel's precise local coordinates relative to center of mass
        Vector2 pixelForce = result.forces * PixelsPerMeter;
        Vector2 globalWheelOffset = wheel.GlobalPosition - GlobalPosition;
        ApplyForce(pixelForce, globalWheelOffset);

        // If locked, exit early
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
        wheelOmega = Mathf.Clamp(wheelOmega, -300f, 300f);
    }

    private Vector2 GetVelocityAtWheel(Marker2D wheelMarker)
    {
        // Derive the true world-space offset relative to the car's origin
        Vector2 globalOffset = wheelMarker.GlobalPosition - GlobalPosition;

        // Calculate the tangential velocity correctly oriented in world space
        Vector2 tangentialVelocity = new Vector2(-globalOffset.Y, globalOffset.X) * AngularVelocity;

        // Combine world linear velocity with world tangential velocity, then scale down to meters
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
