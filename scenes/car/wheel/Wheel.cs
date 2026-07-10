using Godot;

public partial class Wheel : Marker2D
{
    // Physical / setup parameters – can be overridden per wheel in the inspector
    [Export]
    public float TireRadius { get; set; } = 0.33f;

    [Export]
    public PackedScene TireSmokeScene { get; set; }

    [Export]
    public float WheelInertia { get; set; } = 3f;

    [ExportGroup("Tire Model")]
    [Export]
    public float FrictionCoefficient { get; set; } = 1.0f;

    [Export]
    public float RollingResistanceCoefficient { get; set; } = 0.015f;

    [Export]
    public float MinimumSlipSpeed { get; set; } = 0.5f;

    [Export]
    public float MaximumWheelAngularVelocity { get; set; } = 300f;

    [ExportGroup("Debug")]
    [Export]
    public bool DebugTireTelemetry { get; set; } = false;

    private CpuParticles2D _tireSmokeEmitter;

    // Tire angular velocity in radians per second.
    public float Omega { get; set; } = 0f;

    private const float LongitudinalStiffness = 10.0f;
    private const float LongitudinalShape = 1.65f;
    private const float LongitudinalCurvature = 0.97f;

    private const float LateralStiffness = 7.0f;
    private const float LateralShape = 1.30f;
    private const float LateralCurvature = -0.73f;

    private const float AmbientTemp = 25.0f;
    private const float BaseCoolingRate = 0.1f;
    private const float RestSpeedThreshold = 0.02f;
    private const float RestOmegaThreshold = 0.02f;
    private const float TinyValue = 0.0001f;

    private float _smoothedFrictionWatts = 0f;
    private float _frictionSmoothingCoefficient = 20f;
    private float _tireTemperature = AmbientTemp;
    private float _activeRubberMass = 0.5f;
    private float _rubberHeatCapacity = 1250.0f;
    private float _tireSmokePoint = 200.0f;

    // Cached reference to the car parent.
    private RigidBody2D _carBody;

    private readonly struct TireContactPatchState
    {
        public TireContactPatchState(
            Vector2 force,
            float longitudinalForce,
            float lateralForce,
            float forwardSpeed,
            float lateralSpeed,
            float wheelSurfaceSpeed,
            float longitudinalSlipSpeed,
            float slipRatio,
            float slipAngle,
            float wheelAngularVelocity
        )
        {
            Force = force;
            LongitudinalForce = longitudinalForce;
            LateralForce = lateralForce;
            ForwardSpeed = forwardSpeed;
            LateralSpeed = lateralSpeed;
            WheelSurfaceSpeed = wheelSurfaceSpeed;
            LongitudinalSlipSpeed = longitudinalSlipSpeed;
            SlipRatio = slipRatio;
            SlipAngle = slipAngle;
            WheelAngularVelocity = wheelAngularVelocity;
        }

        public Vector2 Force { get; }
        public float LongitudinalForce { get; }
        public float LateralForce { get; }
        public float ForwardSpeed { get; }
        public float LateralSpeed { get; }
        public float WheelSurfaceSpeed { get; }
        public float LongitudinalSlipSpeed { get; }
        public float SlipRatio { get; }
        public float SlipAngle { get; }
        public float WheelAngularVelocity { get; }
    }

    public override void _Ready()
    {
        _carBody = GetParent<RigidBody2D>();

        if (_carBody == null)
        {
            GD.PrintErr($"{nameof(Wheel)} must be a child of a RigidBody2D car.");
        }

        if (TireSmokeScene == null)
        {
            return;
        }

        Node smokeNode = TireSmokeScene.Instantiate();
        _tireSmokeEmitter = smokeNode as CpuParticles2D;

        if (_tireSmokeEmitter == null)
        {
            smokeNode.QueueFree();
            GD.PrintErr($"{nameof(TireSmokeScene)} must instantiate a CpuParticles2D.");
            return;
        }

        AddChild(_tireSmokeEmitter);
        _tireSmokeEmitter.GlobalPosition = GlobalPosition;
        _tireSmokeEmitter.Emitting = false;
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_tireSmokeEmitter != null)
        {
            _tireSmokeEmitter.GlobalPosition = GlobalPosition;
        }
    }

    /// <summary>
    /// Called every physics tick by the car controller.
    /// </summary>
    public void UpdatePhysics(
        float dt,
        float steerAngle,
        float driveTorque,
        float brakeTorque,
        float brakeInput,
        float verticalLoad
    )
    {
        if (_carBody == null || dt <= 0f)
        {
            return;
        }

        verticalLoad = Mathf.Max(0f, verticalLoad);
        brakeInput = Mathf.Clamp(brakeInput, 0f, 1f);

        Vector2 wheelForward = (-_carBody.GlobalTransform.Y).Rotated(steerAngle).Normalized();
        Vector2 wheelRight = new Vector2(-wheelForward.Y, wheelForward.X);
        Vector2 wheelVelocity = GetVelocityAtWheel();

        if (ShouldSnapWheelToRest(wheelVelocity, driveTorque, brakeInput))
        {
            Omega = 0f;
        }

        TireContactPatchState contactPatch = CalculateTireContactPatch(
            wheelVelocity,
            wheelForward,
            wheelRight,
            Omega,
            TireRadius,
            verticalLoad,
            FrictionCoefficient
        );

        Vector2 pixelForce = contactPatch.Force * GameSettings.Instance.PixelsPerMeter;
        Vector2 globalWheelOffset = GlobalPosition - _carBody.GlobalPosition;
        _carBody.ApplyForce(pixelForce, globalWheelOffset);

        float frictionWatts = CalculateTireFrictionWatts(contactPatch, verticalLoad, dt);
        CalculateTireTemperature(frictionWatts, contactPatch.ForwardSpeed, dt);
        UpdateTireSmoke(frictionWatts);

        IntegrateWheelAngularVelocity(
            dt,
            driveTorque,
            brakeTorque,
            brakeInput,
            contactPatch.LongitudinalForce
        );

        if (DebugTireTelemetry)
        {
            GD.Print(
                $"vx:{contactPatch.ForwardSpeed}, vy:{contactPatch.LateralSpeed}, "
                    + $"wheelSurfaceSpeed:{contactPatch.WheelSurfaceSpeed}, "
                    + $"longitudinalSlipSpeed:{contactPatch.LongitudinalSlipSpeed}, "
                    + $"slipRatio:{contactPatch.SlipRatio}, slipAngle:{contactPatch.SlipAngle}, "
                    + $"fx:{contactPatch.LongitudinalForce}, fy:{contactPatch.LateralForce}, "
                    + $"sampledOmega:{contactPatch.WheelAngularVelocity}, omega:{Omega}, "
                    + $"frictionWatts:{frictionWatts}, tireTemp:{_tireTemperature}"
            );
        }
    }

    private Vector2 GetVelocityAtWheel()
    {
        Vector2 globalOffset = GlobalPosition - _carBody.GlobalPosition;
        Vector2 tangentialVelocity =
            new Vector2(-globalOffset.Y, globalOffset.X) * _carBody.AngularVelocity;

        return (_carBody.LinearVelocity + tangentialVelocity)
            / GameSettings.Instance.PixelsPerMeter;
    }

    private bool ShouldSnapWheelToRest(Vector2 wheelVelocity, float driveTorque, float brakeInput)
    {
        bool wheelIsNearlyStopped = Mathf.Abs(Omega) < RestOmegaThreshold;
        bool contactPatchIsNearlyStopped = wheelVelocity.Length() < RestSpeedThreshold;
        bool noDriveTorque = Mathf.Abs(driveTorque) < TinyValue;
        bool noBrakeInput = brakeInput < TinyValue;

        return wheelIsNearlyStopped && contactPatchIsNearlyStopped && noDriveTorque && noBrakeInput;
    }

    /// <summary>
    /// Calculates the normalized Magic Formula coefficient.
    /// Returns a factor between roughly -1.0 and 1.0.
    /// </summary>
    private float PacejkaFormula(float slip, float stiffness, float shape, float curvature)
    {
        float bx = stiffness * slip;
        float insideArcTan = bx - curvature * (bx - Mathf.Atan(bx));
        return Mathf.Sin(shape * Mathf.Atan(insideArcTan));
    }

    private TireContactPatchState CalculateTireContactPatch(
        Vector2 wheelLinearVelocity,
        Vector2 wheelForwardVector,
        Vector2 wheelRightVector,
        float wheelAngularVelocity,
        float tireRadius,
        float verticalLoad,
        float frictionCoefficient
    )
    {
        float forwardSpeed = SnapToZero(wheelLinearVelocity.Dot(wheelForwardVector));
        float lateralSpeed = SnapToZero(wheelLinearVelocity.Dot(wheelRightVector));

        float wheelSurfaceSpeed = SnapToZero(wheelAngularVelocity * tireRadius);
        float longitudinalSlipSpeed = SnapToZero(wheelSurfaceSpeed - forwardSpeed);

        float slipDenominator = Mathf.Max(
            Mathf.Abs(forwardSpeed),
            Mathf.Abs(wheelSurfaceSpeed),
            MinimumSlipSpeed
        );

        float slipRatio = SnapToZero(longitudinalSlipSpeed / slipDenominator);
        float slipAngle = SnapToZero(Mathf.Atan2(lateralSpeed, slipDenominator));

        float maxForce = verticalLoad * frictionCoefficient;

        float longitudinalForce =
            maxForce
            * PacejkaFormula(
                slipRatio,
                LongitudinalStiffness,
                LongitudinalShape,
                LongitudinalCurvature
            );

        float lateralMagnitude =
            maxForce * PacejkaFormula(slipAngle, LateralStiffness, LateralShape, LateralCurvature);

        // Positive lateral velocity means the contact patch is sliding toward wheelRight.
        // Tire force should oppose that slide, so the local lateral force is negative.
        float lateralForce = -lateralMagnitude;

        ApplyCombinedSlipLimit(ref longitudinalForce, ref lateralForce, maxForce);

        longitudinalForce = CleanNumber(longitudinalForce);
        lateralForce = CleanNumber(lateralForce);

        Vector2 worldLongitudinalForce = wheelForwardVector * longitudinalForce;
        Vector2 worldLateralForce = wheelRightVector * lateralForce;
        Vector2 worldForce = worldLongitudinalForce + worldLateralForce;

        return new TireContactPatchState(
            worldForce,
            longitudinalForce,
            lateralForce,
            forwardSpeed,
            lateralSpeed,
            wheelSurfaceSpeed,
            longitudinalSlipSpeed,
            slipRatio,
            slipAngle,
            wheelAngularVelocity
        );
    }

    private void ApplyCombinedSlipLimit(
        ref float longitudinalForce,
        ref float lateralForce,
        float maxForce
    )
    {
        if (maxForce <= 0f)
        {
            longitudinalForce = 0f;
            lateralForce = 0f;
            return;
        }

        float combinedForce = Mathf.Sqrt(
            longitudinalForce * longitudinalForce + lateralForce * lateralForce
        );

        if (combinedForce <= maxForce || combinedForce <= TinyValue)
        {
            return;
        }

        float scale = maxForce / combinedForce;
        longitudinalForce *= scale;
        lateralForce *= scale;
    }

    private void IntegrateWheelAngularVelocity(
        float dt,
        float driveTorque,
        float brakeTorque,
        float brakeInput,
        float longitudinalTireForce
    )
    {
        if (WheelInertia <= 0f)
        {
            Omega = 0f;
            return;
        }

        float brakeCapacity = brakeInput * brakeTorque;

        if (ShouldKeepWheelLocked(driveTorque, brakeCapacity))
        {
            Omega = 0f;
            return;
        }

        float tireFeedbackTorque = longitudinalTireForce * TireRadius;
        float appliedBrakeTorque = CalculateBrakeTorque(brakeCapacity, driveTorque);

        float totalTorque = driveTorque - tireFeedbackTorque + appliedBrakeTorque;
        float angularAcceleration = totalTorque / WheelInertia;
        float nextOmega = Omega + angularAcceleration * dt;

        if (BrakeWouldStopWheel(brakeCapacity, driveTorque, nextOmega))
        {
            nextOmega = 0f;
        }

        Omega = Mathf.Clamp(nextOmega, -MaximumWheelAngularVelocity, MaximumWheelAngularVelocity);
        Omega = SnapToZero(Omega);
    }

    private bool ShouldKeepWheelLocked(float driveTorque, float brakeCapacity)
    {
        bool wheelIsAlmostStopped = Mathf.Abs(Omega) < RestOmegaThreshold;
        bool brakeCanHoldAgainstDrive = brakeCapacity > Mathf.Abs(driveTorque);

        return wheelIsAlmostStopped && brakeCanHoldAgainstDrive;
    }

    private float CalculateBrakeTorque(float brakeCapacity, float driveTorque)
    {
        if (brakeCapacity <= 0f)
        {
            return 0f;
        }

        if (Mathf.Abs(Omega) > RestOmegaThreshold)
        {
            return -Mathf.Sign(Omega) * brakeCapacity;
        }

        if (Mathf.Abs(driveTorque) > TinyValue)
        {
            return -Mathf.Sign(driveTorque) * brakeCapacity;
        }

        return 0f;
    }

    private bool BrakeWouldStopWheel(float brakeCapacity, float driveTorque, float nextOmega)
    {
        if (brakeCapacity <= 0f)
        {
            return false;
        }

        if (brakeCapacity <= Mathf.Abs(driveTorque))
        {
            return false;
        }

        bool wheelWasMoving = Mathf.Abs(Omega) > RestOmegaThreshold;
        bool crossedZero = wheelWasMoving && Mathf.Sign(Omega) != Mathf.Sign(nextOmega);
        bool almostStopped = Mathf.Abs(nextOmega) < RestOmegaThreshold;

        return crossedZero || almostStopped;
    }

    // Friction power measured in watts.
    private float CalculateTireFrictionWatts(
        TireContactPatchState contactPatch,
        float verticalLoad,
        float dt
    )
    {
        // Contact patch slip velocity is the velocity of the rubber relative to the road.
        // Longitudinal: if wheel surface is faster than the car, rubber slips backward.
        // Lateral: lateralSpeed already describes sideways sliding at the contact patch.
        float contactSlipX = -contactPatch.LongitudinalSlipSpeed;
        float contactSlipY = contactPatch.LateralSpeed;

        float forceDotSlipVelocity =
            (contactPatch.LongitudinalForce * contactSlipX)
            + (contactPatch.LateralForce * contactSlipY);

        float slidingHeatWatts = Mathf.Max(0f, -forceDotSlipVelocity);

        float rollingSpeed = Mathf.Abs(contactPatch.ForwardSpeed);
        float rollingResistanceHeatWatts =
            RollingResistanceCoefficient * verticalLoad * rollingSpeed;

        float rawFrictionWatts = CleanNumber(slidingHeatWatts + rollingResistanceHeatWatts);

        float smoothingAlpha = 1f - Mathf.Exp(-_frictionSmoothingCoefficient * dt);
        _smoothedFrictionWatts = Mathf.Lerp(
            _smoothedFrictionWatts,
            rawFrictionWatts,
            smoothingAlpha
        );

        _smoothedFrictionWatts = CleanNumber(_smoothedFrictionWatts);

        return _smoothedFrictionWatts;
    }

    private float CalculateTireTemperature(
        float frictionWatts,
        float vehicleVelocityX,
        float deltaTime
    )
    {
        if (deltaTime <= 0f)
        {
            return _tireTemperature;
        }

        float heatEnergyIn = frictionWatts * deltaTime;
        float temperatureIncrease = heatEnergyIn / (_activeRubberMass * _rubberHeatCapacity);
        _tireTemperature += temperatureIncrease;

        float velocityFactor = Mathf.Abs(vehicleVelocityX);
        float dynamicCoolingRate = BaseCoolingRate + (velocityFactor * 0.05f);

        float temperatureDifference = _tireTemperature - AmbientTemp;
        float temperatureDecrease = temperatureDifference * dynamicCoolingRate * deltaTime;
        _tireTemperature -= temperatureDecrease;

        _tireTemperature = Mathf.Max(_tireTemperature, AmbientTemp);
        _tireTemperature = CleanNumber(_tireTemperature);

        return _tireTemperature;
    }

    private void UpdateTireSmoke(float frictionWatts)
    {
        if (_tireSmokeEmitter == null)
        {
            return;
        }

        _tireSmokeEmitter.GlobalPosition = GlobalPosition;
        _tireSmokeEmitter.Emitting = _tireTemperature >= _tireSmokePoint && frictionWatts > 100f;
    }

    private float SnapToZero(float value)
    {
        return Mathf.Abs(value) < TinyValue ? 0f : value;
    }

    private float CleanNumber(float value)
    {
        if (float.IsNaN(value) || float.IsInfinity(value))
        {
            return 0f;
        }

        return SnapToZero(value);
    }
}
