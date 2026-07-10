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

    private CpuParticles2D _tireSmokeEmitter;

    // Tire AngularVelocity
    public float Omega { get; set; } = 0f; // rad/s

    private static float B_long = 10.0f;
    private static float C_long = 1.65f;
    private static float E_long = 0.97f;

    private static float B_lat = 7.0f;
    private static float C_lat = 1.30f;
    private static float E_lat = -0.73f;

    private float _smoothedFriction = 0f;
    private float _frictionSmoothingCoefficient = 20f; // Higher numbers react faster
    private const float AmbientTemp = 25.0f; //outside temp
    private float _tireTemperature = 25f; //tire temp starts at ambient temp
    private float _activeRubberMass = 0.5f; // weight of tire interacting with the road. 0.5kg
    private float _rubberHeatCapacity = 1250.0f; // Specific heat capacity of tire rubber J/(kg Celsius)
    private float _tireSmokePoint = 200.0f; // degrees celcius
    private const float BaseCoolingRate = 0.1f; // Passive cooling rate when stationary

    // Cached reference to the car (parent RigidBody2D)
    private RigidBody2D _carBody;

    public override void _Ready()
    {
        _carBody = GetParent<RigidBody2D>();
        _tireSmokeEmitter = TireSmokeScene.Instantiate() as CpuParticles2D;

        if (_carBody == null)
            GD.PrintErr($"{nameof(Wheel)} must be a child of a RigidBody2D car.");
    }

    public override void _PhysicsProcess(double delta)
    {
        _tireSmokeEmitter.GlobalPosition = GlobalPosition;
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
        if (_carBody == null)
            return;

        // 1. Wheel forward direction in world space
        Vector2 wheelForward = (-_carBody.GlobalTransform.Y).Rotated(steerAngle).Normalized();
        Vector2 wheelRight = new Vector2(-wheelForward.Y, wheelForward.X);
        // 2. Free‑rolling sync
        if (Mathf.Abs(driveTorque) < 0.0001f && Mathf.Abs(brakeInput) < 0.0001f)
        {
            Vector2 wVel = GetVelocityAtWheel();
            float groundSpeed = wVel.Dot(wheelForward);
            Omega = groundSpeed / TireRadius;
        }

        // 3. Brake locking logic
        float prospectiveAppliedbrakeTorque = 0f;
        bool isWheelLocked = false;

        if (brakeInput > 0f && Mathf.Abs(Omega) > 0.01f)
        {
            float brakeDirection = -Mathf.Sign(Omega);
            prospectiveAppliedbrakeTorque = brakeInput * brakeTorque * brakeDirection;

            Vector2 preVel = GetVelocityAtWheel();
            var preResult = CalculateTireForces(
                dt,
                preVel,
                wheelForward,
                Omega,
                TireRadius,
                verticalLoad
            );
            float preFeedback = preResult.forces.Dot(wheelForward) * TireRadius;

            float netTorque = driveTorque - preFeedback + prospectiveAppliedbrakeTorque;
            float prospectiveDeltaOmega = (netTorque / WheelInertia) * dt;

            if (Mathf.Sign(Omega) != Mathf.Sign(Omega + prospectiveDeltaOmega))
            {
                Omega = 0f;
                isWheelLocked = true;
            }
        }

        // 4. Final tire forces
        Vector2 wheelVelocity = GetVelocityAtWheel();
        var result = CalculateTireForces(
            dt,
            wheelVelocity,
            wheelForward,
            Omega,
            TireRadius,
            verticalLoad
        );

        // 5. Apply force to car body at this wheel’s offset
        Vector2 pixelForce = result.forces * GameSettings.Instance.PixelsPerMeter;
        Vector2 globalWheelOffset = GlobalPosition - _carBody.GlobalPosition;
        _carBody.ApplyForce(pixelForce, globalWheelOffset);

        // 6. Stay locked if required
        if (isWheelLocked || (brakeInput > 0f && Mathf.Abs(Omega) <= 0.01f))
        {
            Omega = 0f;
        }
        else
        {
            // 7. Integrate omega
            float fxTire = result.forces.Dot(wheelForward);
            float tireFeedbackTorque = fxTire * TireRadius;
            float totalTorque = driveTorque - tireFeedbackTorque + prospectiveAppliedbrakeTorque;
            float angularAcceleration = totalTorque / WheelInertia;
            Omega += angularAcceleration * dt;
            Omega = Mathf.Clamp(Omega, -300f, 300f);
        }
        float finalVx = wheelVelocity.Dot(wheelForward);
        float finalVy = wheelVelocity.Dot(wheelRight);
        float finalVSlipX = (Omega * TireRadius) - finalVx;
        float finalFx = result.forces.Dot(wheelForward);
        float finalFy = result.forces.Dot(wheelRight);

        GD.Print(
            $"finalVSlipX:{finalVSlipX}, finalFX:{finalFx}, finalFy:{finalFy}, Omega:{Omega}, isWheelLocked:{isWheelLocked}"
        );
        // Calculate power dissipation and update heat state
        float currentWatts = CalculateTireFriction(finalVSlipX, finalFx, finalVy, finalFy, dt);
        CalculateTireTemperature(currentWatts, finalVx, dt);
    }

    private Vector2 GetVelocityAtWheel()
    {
        Vector2 globalOffset = GlobalPosition - _carBody.GlobalPosition;
        Vector2 tangentialVelocity =
            new Vector2(-globalOffset.Y, globalOffset.X) * _carBody.AngularVelocity;
        return (_carBody.LinearVelocity + tangentialVelocity)
            / GameSettings.Instance.PixelsPerMeter;
    }

    /// <summary>
    /// Calculates the normalized Magic Formula coefficient (returns a factor between -1.0 and 1.0)
    /// </summary>
    private float PacejkaFormula(float slip, float B, float C, float E)
    {
        float bx = B * slip;
        float insideArcTan = bx - E * (bx - Mathf.Atan(bx));
        return Mathf.Sin(C * Mathf.Atan(insideArcTan));
    }

    private (Vector2 forces, float slipRatio, float slipAngle) CalculateTireForces(
        float dt,
        Vector2 wheelLinearVelocity,
        Vector2 wheelForwardVector,
        float wheelAngularVelocity,
        float tireRadius,
        float verticalLoad,
        float frictionCoefficient = 1.0f
    )
    {
        Vector2 wheelRightVector = new Vector2(-wheelForwardVector.Y, wheelForwardVector.X);

        // Transform global velocity to wheel local space
        float vx = wheelLinearVelocity.Dot(wheelForwardVector);
        float vy = wheelLinearVelocity.Dot(wheelRightVector);

        // Calculate wheel speed and actual sliding speed
        float wheelLinearSpeed = wheelAngularVelocity * tireRadius;
        float vSlip = wheelLinearSpeed - vx;

        // ====================================================================
        // CRITICAL FIX: LOW-SPEED VELOCITY DAMPENING (ZERO VELOCITY SAFEGUARD)
        // ====================================================================
        // If BOTH the car and the wheel are barely moving, force everything to 0
        // This stops phantom 40kW calculations at a dead stop.
        float speedThreshold = 0.2f; // 20 cm/s
        float lowSpeedFade = 1.0f;

        if (Mathf.Abs(vx) < speedThreshold && Mathf.Abs(wheelLinearSpeed) < speedThreshold)
        {
            // Smoothly fade forces to zero as the wheel comes to a complete rest
            float maxSpeed = Mathf.Max(Mathf.Abs(vx), Mathf.Abs(wheelLinearSpeed));
            lowSpeedFade = maxSpeed / speedThreshold;
        }
        // ====================================================================

        // Safeguard for division by zero near zero velocity
        float epsilon = 0.05f;
        float safeVx = vx;
        if (Mathf.Abs(safeVx) < epsilon)
        {
            safeVx = epsilon * (vx >= 0 ? 1f : -1f);
        }

        // 1. Calculate Pure Longitudinal Force (Fx)
        float slipRatio = vSlip / Mathf.Abs(safeVx);

        float D_long = verticalLoad * frictionCoefficient;
        float fxMag = D_long * PacejkaFormula(slipRatio, B_long, C_long, E_long);

        // 2. Calculate Pure Lateral Force (Fy)
        float slipAngle = Mathf.Atan2(vy, Mathf.Abs(safeVx));
        if (Mathf.Abs(slipAngle) < 0.0025f)
            slipAngle = 0f;

        float D_lat = verticalLoad * frictionCoefficient;
        float fyMag = D_lat * PacejkaFormula(slipAngle, B_lat, C_lat, E_lat);

        // 3. Normalized Slip Vector Scaling (Pacejka Combined Model)
        float absoluteSlipX = Mathf.Clamp(Mathf.Abs(slipRatio), 0f, 10f);
        float absoluteSlipY = Mathf.Clamp(Mathf.Abs(Mathf.Tan(slipAngle)), 0f, 10f);
        float combinedSlip = Mathf.Sqrt(
            absoluteSlipX * absoluteSlipX + absoluteSlipY * absoluteSlipY
        );

        if (combinedSlip > 0.001f)
        {
            fxMag *= (absoluteSlipX / combinedSlip);
            fyMag *= (absoluteSlipY / combinedSlip);
        }

        // ====================================================================
        // APPLY THE FADE BEFORE CALCULATING FRICTION AND TEMPERATURE
        // ====================================================================
        fxMag *= lowSpeedFade;
        fyMag *= lowSpeedFade;
        // ====================================================================

        // Clean the data for safety
        if (float.IsNaN(fxMag) || float.IsInfinity(fxMag))
            fxMag = 0f;
        if (float.IsNaN(fyMag) || float.IsInfinity(fyMag))
            fyMag = 0f;

        // 4. Absolute Friction Circle Limit Safeguard
        float maxForce = verticalLoad * frictionCoefficient;
        float combinedForceMag = Mathf.Sqrt(fxMag * fxMag + fyMag * fyMag);

        if (combinedForceMag > maxForce && combinedForceMag > 0)
        {
            fxMag = (fxMag / combinedForceMag) * maxForce;
            fyMag = (fyMag / combinedForceMag) * maxForce;
        }

        // Convert scalar forces back to 2D world vectors
        Vector2 worldFx = wheelForwardVector * fxMag;
        Vector2 worldFy = wheelRightVector * -fyMag;

        return (worldFx + worldFy, slipRatio, slipAngle);
    }

    // friction measured in watts
    private float CalculateTireFriction(float vSlipX, float fx, float vSlipY, float fy, float dt)
    {
        var rawLongitudinalFriction = Mathf.Abs(fx * vSlipX);
        var rawLateralFriction = Mathf.Abs(fy * vSlipY);

        var rawFriction = rawLateralFriction + rawLongitudinalFriction;

        _smoothedFriction = Mathf.Lerp(
            _smoothedFriction,
            rawFriction,
            _frictionSmoothingCoefficient * dt
        );

        GD.Print($"Friction in watts: {_smoothedFriction}");

        return _smoothedFriction;
    }

    private float CalculateTireTemperature(
        float frictionWatts,
        float vehicleVelocityX,
        float deltaTime
    )
    {
        if (deltaTime <= 0f)
            return _tireTemperature;

        // 1. Calculate thermal energy input (Joules = Watts * seconds)
        float heatEnergyIn = frictionWatts * deltaTime;

        // 2. Calculate localized temperature increase
        // ΔT = Q / (m * c)
        float temperatureIncrease = heatEnergyIn / (_activeRubberMass * _rubberHeatCapacity);
        _tireTemperature += temperatureIncrease;

        // 3. Calculate cooling (Newton's Law of Cooling)
        // Rushing air cools the tire faster. We scale cooling by vehicle forward speed (vx)
        float velocityFactor = Mathf.Abs(vehicleVelocityX);
        float dynamicCoolingRate = BaseCoolingRate + (velocityFactor * 0.05f);

        // Cool the tire down toward ambient room temperature
        float temperatureDifference = _tireTemperature - AmbientTemp;
        float temperatureDecrease = temperatureDifference * dynamicCoolingRate * deltaTime;
        _tireTemperature -= temperatureDecrease;

        // Clamp temperature so it never drops below the ambient outdoor temperature
        _tireTemperature = Mathf.Max(_tireTemperature, AmbientTemp);

        GD.Print($"Tire Temp: {_tireTemperature}°C");
        return _tireTemperature;
    }
}
