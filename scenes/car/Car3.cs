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
    public AnimationPlayer AnimPlayer { get; set; }

    [ExportGroup("WheelPositions")]
    [Export]
    private Wheel _frontLeftWheel;

    [Export]
    private Wheel _frontRightWheel;

    [Export]
    private Wheel _rearLeftWheel;

    [Export]
    private Wheel _rearRightWheel;

    [ExportGroup("Engine")]
    [Export]
    public EngineData Engine { get; set; }

    [ExportGroup("Drivetrain")]
    [ExportSubgroup("Transmission")]
    [Export]
    public TransmissionData Transmission { get; set; }

    [ExportSubgroup("Steering")]
    [Export]
    public float MaxSteerAngle = 0.50f;

    [ExportGroup("Visuals & Chassis")]
    [Export]
    private Node2D CarSprite; // Assign the main car Sprite2D/Node2D here!

    [Export]
    private float BodyRollStiffness = 0.05f; // Controls how stiff the suspension resists roll

    [Export]
    private float BodyRollDamping = 0.15f; // Dampens body bounce vibrations

    [Export]
    private float CenterOfMassHeight = 0.4f; // Estimated height of car's CG in meters

    [Export]
    public new float Mass = 1398f;

    [ExportSubgroup("Braking")]
    [Export]
    private float BrakeTorque = 1500f; // Max braking power

    private float Wheelbase =>
        Mathf.Abs(_frontLeftWheel.Position.Y - _rearLeftWheel.Position.Y)
        / GameSettings.Instance.PixelsPerMeter;

    private float TrackWidth =>
        Mathf.Abs(_frontLeftWheel.Position.X - _frontRightWheel.Position.X)
        / GameSettings.Instance.PixelsPerMeter;

    private Vector2 _previousLinearVelocity = Vector2.Zero;

    private float Weight
    {
        get => Mass * GameSettings.Instance.Gravity;
    }
    private float NominalLoadOnTire
    {
        get => Weight / 4;
    }

    private float VerticalLoadOnTire { get; set; }

    public override void _Ready()
    {
        ZIndex = 10;

        if (AnimPlayer == null)
        {
            GD.PrintErr("AnimationPlayer is not assigned in the Inspector!");
        }

        AnimPlayer.Play("normal");
    }

    private float _currentBodyRollAngle = 0f;
    private float _bodyRollVelocity = 0f;

    // Variable to track longitudinal load shift for accurate spin-outs
    private float _longitudinalWeightTransfer = 0f;

    // Variable to track lateral load shift for accurate spin-outs
    private float _lateralWeightTransfer = 0;

    public override void _PhysicsProcess(double delta)
    {
        float dt = (float)delta;

        // 1. Process shifting inputs first
        if (Input.IsActionJustPressed("shift_up"))
        {
            Transmission.ShiftGearUp();
        }

        // Shift Down (6 -> 1 -> Neutral -> Reverse)
        if (Input.IsActionJustPressed("shift_down"))
        {
            Transmission.ShiftGearDown();
        }

        float throttle = Input.IsActionPressed("accelerate") ? 1f : 0f;
        float brake = Input.IsActionPressed("brake") ? 1f : 0f;
        float steerInput =
            (Input.IsActionPressed("steer_right") ? 1f : 0f)
            - (Input.IsActionPressed("steer_left") ? 1f : 0f);

        (float steerLeft, float steerRight) = GetAckermanSteerValues(
            steerInput,
            Wheelbase,
            TrackWidth
        );

        // 3. Dynamic Weight Transfer Systems
        Vector2 currentVelocityMps = GetVelocityInMetersPerSecond();
        float speedKmh = currentVelocityMps.Length() * 3.6f;
        GD.Print($"KM/H: {speedKmh}");
        (float lateralAcceleration, float longitudinalAcceleration) =
            GetAccelerationInMetersPerSecond(_previousLinearVelocity, currentVelocityMps, dt);

        _previousLinearVelocity = currentVelocityMps;

        UpdateWeightTransfers(lateralAcceleration, longitudinalAcceleration);

        // 4. MOTOR DYNAMICS & BI-DIRECTIONAL RPM MATRIX
        float averageRearWheelOmega = (_rearLeftWheel.Omega + _rearRightWheel.Omega) / 2f;

        float driveTorquePerWheel = Engine.GetWheelTorqueFromEnginePower(
            Transmission.GetCurrentGearRatio(),
            Transmission.FinalDriveRatio,
            throttle,
            averageRearWheelOmega,
            dt
        );

        // 5. Split Brake Input safely
        float frontBrakeInput = brake * 0.60f;
        float rearBrakeInput = brake * 0.40f;

        (float fzFL, float fzFR, float fzRL, float fzRR) = GetWheelVerticalLoads();

        // // 6. Execute Wheel Simulation Pipelines
        _frontLeftWheel.UpdatePhysics(dt, steerLeft, 0f, BrakeTorque, frontBrakeInput, fzFL);
        _frontRightWheel.UpdatePhysics(dt, steerRight, 0f, BrakeTorque, frontBrakeInput, fzFR);
        _rearLeftWheel.UpdatePhysics(
            dt,
            0f,
            driveTorquePerWheel,
            BrakeTorque,
            rearBrakeInput,
            fzRL
        );
        _rearRightWheel.UpdatePhysics(
            dt,
            0f,
            driveTorquePerWheel,
            BrakeTorque,
            rearBrakeInput,
            fzRR
        );

        // 7. Physics-Based Visual Chassis Roll Matrix
        if (CarSprite != null)
        {
            UpdateChassisRoll(lateralAcceleration, longitudinalAcceleration);
        }
        SleepCarIfNearlyStopped(throttle, brake, steerInput);
    }

    public (
        float lateralAcceleration,
        float longitudinalAcceleration
    ) GetAccelerationInMetersPerSecond(
        Vector2 previousLinearVelocity,
        Vector2 currentVelocityMps,
        float dt
    )
    {
        Vector2 accelerationMps = (currentVelocityMps - previousLinearVelocity) / dt;
        Vector2 localAccel = accelerationMps.Rotated(-GlobalRotation);

        return (localAccel.X, localAccel.Y);
    }

    public Vector2 GetVelocityInMetersPerSecond()
    {
        return LinearVelocity / GameSettings.Instance.PixelsPerMeter;
    }

    public void UpdateWeightTransfers(float lateralAcceleration, float longitudinalAcceleration)
    {
        _lateralWeightTransfer = (Mass * lateralAcceleration * CenterOfMassHeight) / TrackWidth;
        _longitudinalWeightTransfer =
            (Mass * longitudinalAcceleration * CenterOfMassHeight) / Wheelbase;

        _lateralWeightTransfer = Mathf.Clamp(
            _lateralWeightTransfer,
            -NominalLoadOnTire * 1.5f,
            NominalLoadOnTire * 1.5f
        );
        _longitudinalWeightTransfer = Mathf.Clamp(
            _longitudinalWeightTransfer,
            -NominalLoadOnTire * 1.5f,
            NominalLoadOnTire * 1.5f
        );
    }

    public void UpdateChassisRoll(float lateralAcceleration, float longitudinalAcceleration)
    {
        float targetForce = -lateralAcceleration * 0.002f;
        float springForce = (targetForce - _currentBodyRollAngle) * BodyRollStiffness;
        _bodyRollVelocity += springForce - (_bodyRollVelocity * BodyRollDamping);
        _currentBodyRollAngle += _bodyRollVelocity;

        CarSprite.Rotation = _currentBodyRollAngle;
        CarSprite.Skew = -longitudinalAcceleration * 0.001f;
    }

    public (float fzFL, float fzFR, float fzRL, float fzRR) GetWheelVerticalLoads()
    {
        float fzFL = Mathf.Max(
            10f,
            NominalLoadOnTire - (_lateralWeightTransfer / 2f) - (_longitudinalWeightTransfer / 2f)
        );
        float fzFR = Mathf.Max(
            10f,
            NominalLoadOnTire + (_lateralWeightTransfer / 2f) - (_longitudinalWeightTransfer / 2f)
        );
        float fzRL = Mathf.Max(
            10f,
            NominalLoadOnTire - (_lateralWeightTransfer / 2f) + (_longitudinalWeightTransfer / 2f)
        );
        float fzRR = Mathf.Max(
            10f,
            NominalLoadOnTire + (_lateralWeightTransfer / 2f) + (_longitudinalWeightTransfer / 2f)
        );

        return (fzFL, fzFR, fzRL, fzRR);
    }

    public (float steerLeft, float steerRight) GetAckermanSteerValues(
        float steerInput,
        float wheelbase,
        float trackWidth
    )
    {
        float targetSteerAngle = steerInput * MaxSteerAngle;
        float steerLeft = 0f;
        float steerRight = 0f;
        if (Mathf.Abs(targetSteerAngle) > 0.001f)
        {
            float radius = wheelbase / Mathf.Tan(Mathf.Abs(targetSteerAngle));
            if (targetSteerAngle > 0f)
            {
                steerRight =
                    Mathf.Atan(wheelbase / (radius - (trackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
                steerLeft =
                    Mathf.Atan(wheelbase / (radius + (trackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
            }
            else
            {
                steerLeft =
                    Mathf.Atan(wheelbase / (radius - (trackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
                steerRight =
                    Mathf.Atan(wheelbase / (radius + (trackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
            }
        }

        return (steerLeft, steerRight);
    }

    private void SleepCarIfNearlyStopped(float throttle, float brake, float steerInput)
    {
        float speedMps = LinearVelocity.Length() / GameSettings.Instance.PixelsPerMeter;

        bool noInput = throttle <= 0f && brake <= 0f && Mathf.Abs(steerInput) < 0.001f;

        bool nearlyStopped = speedMps < 0.15f && Mathf.Abs(AngularVelocity) < 0.05f;

        if (noInput && nearlyStopped)
        {
            LinearVelocity = Vector2.Zero;
            AngularVelocity = 0f;

            _frontLeftWheel.Omega = 0f;
            _frontRightWheel.Omega = 0f;
            _rearLeftWheel.Omega = 0f;
            _rearRightWheel.Omega = 0f;
        }
    }
}
