using Godot;

// GlobalClass registers this resource type in the Godot Inspector
[GlobalClass]
public partial class EngineData : Resource
{
    [Export]
    public string EngineName { get; set; } = "Standard Engine";

    [Export]
    public float MaxEngineTorque { get; set; } = 233f;

    [Export]
    public float IdleRPM { get; set; } = 1000f;

    [Export]
    public float RedlineRPM { get; set; } = 7000f;

    [Export]
    public Curve TorqueCurve { get; set; }

    private const float REV_LIMIT_HOLD_TIME = 0.15f;
    private float _revLimiterTimer = 0f;

    private float EngineRPM { get; set; } = 1000f;

    // Note: Resources do not automatically process _Process,
    // so you must call this method manually from your Car Node script.
    public float GetEngineTorqueAtRPM(float rpm, float throttle, float dt)
    {
        if (rpm < IdleRPM)
            rpm = IdleRPM;

        // 1. REV LIMITER
        if (_revLimiterTimer > 0f)
        {
            _revLimiterTimer -= dt;
            return -50f;
        }

        if (rpm >= RedlineRPM)
        {
            _revLimiterTimer = REV_LIMIT_HOLD_TIME;
            return -50f;
        }

        // 2. TORQUE RESOURCE SAMPLING
        float normalizedRPM = (rpm - IdleRPM) / (RedlineRPM - IdleRPM);
        float torqueFactor = 1.0f;

        if (TorqueCurve != null)
        {
            torqueFactor = TorqueCurve.Sample(normalizedRPM);
        }

        float combustionTorque = MaxEngineTorque * torqueFactor * throttle;

        // 3. ENGINE FRICTION
        float engineFriction = 5f + (15f * normalizedRPM);

        return combustionTorque - engineFriction;
    }

    public float GetWheelTorqueFromEnginePower(
        float gearRatio,
        float finalDriveRatio,
        float throttle,
        float averageRearWheelOmega,
        float dt
    )
    {
        if (gearRatio == 0.0f) // NEUTRAL: Wheels disconnected from engine
        {
            // Engine relaxes back toward standard idle or revs freely up to redline on throttle
            float targetRPM = throttle > 0.1f ? RedlineRPM : IdleRPM;
            EngineRPM = Mathf.MoveToward(EngineRPM, targetRPM, 4000f * dt);
            return 0f;
        }
        else // IN GEAR: Connect wheels directly back to calculate authentic engine speed
        {
            // float averageRearWheelOmega = (_rearLeftWheel.Omega + _rearRightWheel.Omega) / 2f;

            // 1. Calculate raw target RPM from wheel speed
            float calculatedEngineRPM = Mathf.Abs(
                averageRearWheelOmega * gearRatio * finalDriveRatio * (60f / (2f * Mathf.Pi))
            );

            // 2. FIXED: Smoothly interpolate the RPM instead of jumping instantly.
            // A smoothing factor of 15.0f lets the engine react quickly but filters out frame-by-frame spikes.
            EngineRPM = Mathf.Lerp(EngineRPM, calculatedEngineRPM, 15.0f * dt);

            // 3. Clamp the engine RPM between legal operational bounds
            EngineRPM = Mathf.Clamp(EngineRPM, IdleRPM, RedlineRPM);

            // 4. Derive true variable torque force from the engine's RPM curve
            float currentEngineTorque = GetEngineTorqueAtRPM(EngineRPM, throttle, dt);

            // 5. Calculate drive axle torque and send it down to the wheels
            float totalDriveTorque = throttle * currentEngineTorque * gearRatio * finalDriveRatio;

            return totalDriveTorque / 2;
        }
    }
}
