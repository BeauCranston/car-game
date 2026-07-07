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

    [Export]
    private float PixelsPerMeter { get; set; } = 100f;

    [ExportGroup("WheelPositions")]
    [Export]
    private Marker2D _frontLeftWheel;

    [Export]
    private Marker2D _frontRightWheel;

    [Export]
    private Marker2D _rearLeftWheel;

    [Export]
    private Marker2D _rearRightWheel;

    [ExportGroup("Vehicle")]
    [ExportSubgroup("Visuals & Chassis")]
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

    [ExportSubgroup("Steering")]
    [Export]
    private float MaxSteerAngle = 0.50f;

    [ExportSubgroup("Tire")]
    [Export]
    private float TireRadius = 0.33f;

    [Export]
    private float WheelInertia = 3f;

    private float Wheelbase =>
        Mathf.Abs(_frontLeftWheel.Position.Y - _rearLeftWheel.Position.Y) / PixelsPerMeter;

    private float TrackWidth =>
        Mathf.Abs(_frontLeftWheel.Position.X - _frontRightWheel.Position.X) / PixelsPerMeter;

    private Vector2 _previousLinearVelocity = Vector2.Zero;

    private float _gravity = 9.81f;
    private float Weight
    {
        get => Mass * _gravity;
    }
    private float NominalLoadOnTire
    {
        get => Weight / 4;
    }

    private float VerticalLoadOnTire { get; set; }

    private Gear _gear = Gear.Forward;
    private int GearDirection => _gear == Gear.Forward ? 1 : -1;

    [ExportGroup("Power Train")]
    [ExportSubgroup("Engine")]
    [Export]
    private float MaxEngineTorque = 233f; // Newton-meters

    [Export]
    private float IdleRPM { get; set; } = 1000f;

    [Export]
    private const float RevLimitHoldTime = 0.15f;

    [Export]
    private float RedlineRPM { get; set; } = 7000f;

    [ExportSubgroup("Transmission")]
    [Export]
    private float _finalDriveRatio = 4.06f; // Differential ratio

    [Export]
    private float FirstGear = 3.538f; // 1: 1st Gear

    [Export]
    private float SecondGear = 2.047f; // 2: 2nd Gear

    [Export]
    private float ThirdGear = 1.375f; // 3: 3rd Gear

    [Export]
    private float FourthGear = 1.025f; // 4: 4th Gear (Increased from 1.12)

    [Export]
    private float FifthGear = 0.875f; // 5: 5th Gear (Increased from 0.85)

    [Export]
    private float SixthGear = 0.733f; // 6: 6th Gear (Increased from 0.68)

    [Export]
    private float ReverseGear = -3.55f; // 7: Reverse Gear

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

    // Gear ratios for a typical 6-speed close-ratio transmission + Reverse
    private float[] _gearRatios
    {
        get
        {
            return new float[]
            {
                0.0f, // 0: Neutral
                FirstGear,
                SecondGear,
                ThirdGear,
                FourthGear,
                FifthGear,
                SixthGear,
                ReverseGear,
            };
        }
    }

    private int _currentGearIndex = 1; // Start in 1st gear
    private float EngineRPM { get; set; } = 1000f;
    private float _revLimiterTimer = 0f;

    private float _currentBodyRollAngle = 0f;
    private float _bodyRollVelocity = 0f;

    // Variable to track longitudinal load shift for accurate spin-outs
    private float _longitudinalWeightTransfer = 0f;

    private float GetEngineTorqueAtRPM(float rpm, float throttle, float dt)
    {
        if (rpm < IdleRPM)
            rpm = IdleRPM;

        // 1. STABLE REV LIMITER (Ignition/Fuel Cut)
        if (_revLimiterTimer > 0f)
        {
            _revLimiterTimer -= dt;
            return -50f; // Soft internal engine friction during the spark cut
        }

        if (rpm >= RedlineRPM)
        {
            _revLimiterTimer = RevLimitHoldTime; // Activate the hold timer
            return -50f;
        }

        // 2. FIXED: FLAT SPORTS CAR POWER PLATEAU
        float normalizedRPM = (rpm - IdleRPM) / (RedlineRPM - IdleRPM);

        // This profile guarantees that from 2500 RPM up to 6000 RPM, the engine
        // maintains a flat 95% to 100% of its maximum torque capacity.
        float torqueFactor = 1.0f;

        if (rpm < 3000f)
        {
            // Smoothly climb from 80% power at idle up to 100% at 3000 RPM
            float lowEndScale = (rpm - IdleRPM) / (3000f - IdleRPM);
            torqueFactor = Mathf.Lerp(0.80f, 1.0f, lowEndScale);
        }
        else if (rpm > 5500f)
        {
            // Smoothly taper off power slightly near the redline to simulate natural engine breath limits
            float highEndScale = (rpm - 5500f) / (RedlineRPM - 5500f);
            torqueFactor = Mathf.Lerp(1.0f, 0.85f, highEndScale);
        }

        float combustionTorque = MaxEngineTorque * torqueFactor * throttle;

        // 3. ENGINE FRICTION
        float engineFriction = 5f + (15f * normalizedRPM);

        return combustionTorque - engineFriction;
    }

    private void UpdateGearInput()
    {
        // Shift Up (1 -> 6, can't shift past 6)
        // Index 7 is Reverse, so if we are in Neutral (0) we shift to 1st (1)
        if (Input.IsActionJustPressed("shift_up"))
        {
            if (_currentGearIndex == 7) // From Reverse to Neutral
                _currentGearIndex = 0;
            else if (_currentGearIndex < 6) // Normal sequential upshift
                _currentGearIndex++;

            GD.Print($"Shifted UP to Gear: {GetGearName()}");
        }

        // Shift Down (6 -> 1 -> Neutral -> Reverse)
        if (Input.IsActionJustPressed("shift_down"))
        {
            if (_currentGearIndex == 0) // From Neutral to Reverse
                _currentGearIndex = 7;
            else if (_currentGearIndex > 0 && _currentGearIndex <= 6) // Normal downshift
                _currentGearIndex--;

            GD.Print($"Shifted DOWN to Gear: {GetGearName()}");
        }
    }

    private string GetGearName()
    {
        if (_currentGearIndex == 0)
            return "Neutral";
        if (_currentGearIndex == 7)
            return "Reverse";
        return $"{_currentGearIndex}";
    }

    public override void _PhysicsProcess(double delta)
    {
        float dt = (float)delta;

        // 1. Process shifting inputs first
        UpdateGearInput();

        float throttle = Input.IsActionPressed("accelerate") ? 1f : 0f;
        float brake = Input.IsActionPressed("brake") ? 1f : 0f;
        float steerInput =
            (Input.IsActionPressed("steer_right") ? 1f : 0f)
            - (Input.IsActionPressed("steer_left") ? 1f : 0f);

        float targetSteerAngle = steerInput * MaxSteerAngle;

        // 2. Ackerman Steering Architecture
        float steerLeft = 0f;
        float steerRight = 0f;
        if (Mathf.Abs(targetSteerAngle) > 0.001f)
        {
            float radius = Wheelbase / Mathf.Tan(Mathf.Abs(targetSteerAngle));
            if (targetSteerAngle > 0f)
            {
                steerRight =
                    Mathf.Atan(Wheelbase / (radius - (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
                steerLeft =
                    Mathf.Atan(Wheelbase / (radius + (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
            }
            else
            {
                steerLeft =
                    Mathf.Atan(Wheelbase / (radius - (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
                steerRight =
                    Mathf.Atan(Wheelbase / (radius + (TrackWidth / 2f)))
                    * Mathf.Sign(targetSteerAngle);
            }
        }

        // 3. Dynamic Weight Transfer Systems
        Vector2 currentVelocityMps = LinearVelocity / PixelsPerMeter;
        Vector2 accelerationMps = (currentVelocityMps - _previousLinearVelocity) / dt;
        _previousLinearVelocity = currentVelocityMps;

        Vector2 localAccel = accelerationMps.Rotated(-GlobalRotation);
        float lateralAcceleration = localAccel.X;
        float longitudinalAcceleration = localAccel.Y;

        float lateralWeightTransfer =
            (Mass * lateralAcceleration * CenterOfMassHeight) / TrackWidth;
        _longitudinalWeightTransfer =
            (Mass * longitudinalAcceleration * CenterOfMassHeight) / Wheelbase;

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

        float fzFL = Mathf.Max(
            10f,
            NominalLoadOnTire - (lateralWeightTransfer / 2f) - (_longitudinalWeightTransfer / 2f)
        );
        float fzFR = Mathf.Max(
            10f,
            NominalLoadOnTire + (lateralWeightTransfer / 2f) - (_longitudinalWeightTransfer / 2f)
        );
        float fzRL = Mathf.Max(
            10f,
            NominalLoadOnTire - (lateralWeightTransfer / 2f) + (_longitudinalWeightTransfer / 2f)
        );
        float fzRR = Mathf.Max(
            10f,
            NominalLoadOnTire + (lateralWeightTransfer / 2f) + (_longitudinalWeightTransfer / 2f)
        );

        // 4. MOTOR DYNAMICS & BI-DIRECTIONAL RPM MATRIX
        float currentGearRatio = _gearRatios[_currentGearIndex];
        float driveTorquePerWheel = 0f;

        if (_currentGearIndex == 0) // NEUTRAL: Wheels disconnected from engine
        {
            // Engine relaxes back toward standard idle or revs freely up to redline on throttle
            float targetRPM = throttle > 0.1f ? RedlineRPM : IdleRPM;
            EngineRPM = Mathf.MoveToward(EngineRPM, targetRPM, 4000f * dt);
            driveTorquePerWheel = 0f;
        }
        else // IN GEAR: Connect wheels directly back to calculate authentic engine speed
        {
            float averageRearWheelOmega = (_omegaRL + _omegaRR) / 2f;

            // 1. Calculate raw target RPM from wheel speed
            float calculatedEngineRPM = Mathf.Abs(
                averageRearWheelOmega
                    * currentGearRatio
                    * _finalDriveRatio
                    * (60f / (2f * Mathf.Pi))
            );

            // 2. FIXED: Smoothly interpolate the RPM instead of jumping instantly.
            // A smoothing factor of 15.0f lets the engine react quickly but filters out frame-by-frame spikes.
            EngineRPM = Mathf.Lerp(EngineRPM, calculatedEngineRPM, 15.0f * dt);

            // 3. Clamp the engine RPM between legal operational bounds
            EngineRPM = Mathf.Clamp(EngineRPM, IdleRPM, RedlineRPM);

            // 4. Derive true variable torque force from the engine's RPM curve
            float currentEngineTorque = GetEngineTorqueAtRPM(EngineRPM, throttle, dt);

            // 5. Calculate drive axle torque and send it down to the wheels
            float totalDriveTorque =
                throttle * currentEngineTorque * currentGearRatio * _finalDriveRatio;
            driveTorquePerWheel = totalDriveTorque / 2f;
        }
        // 5. Split Brake Input safely
        float frontBrakeInput = brake * 0.60f;
        float rearBrakeInput = brake * 0.40f;

        // 6. Execute Wheel Simulation Pipelines
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

        // 7. Physics-Based Visual Chassis Roll Matrix
        if (CarSprite != null)
        {
            float targetForce = -lateralAcceleration * 0.002f;
            float springForce = (targetForce - _currentBodyRollAngle) * BodyRollStiffness;
            _bodyRollVelocity += springForce - (_bodyRollVelocity * BodyRollDamping);
            _currentBodyRollAngle += _bodyRollVelocity;

            CarSprite.Rotation = _currentBodyRollAngle;
            CarSprite.Skew = -longitudinalAcceleration * 0.001f;
        }
        GD.Print($"EngineRPM: {EngineRPM}");
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
            prospectiveAppliedBrakeTorque = brakeInput * BrakeTorque * brakeDirection;

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
