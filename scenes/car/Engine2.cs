using Godot;

public partial class Engine2 : Node
{
    [Export]
    private float MaxEngineTorque = 233f; // Newton-meters

    [Export]
    private float IdleRPM { get; set; } = 1000f;

    [Export]
    private float RedlineRPM { get; set; } = 7000f;

    private float EngineRPM { get; set; } = 1000f;
    private const float REV_LIMIT_HOLD_TIME = 0.15f;
    private float _revLimiterTimer = 0f;

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
            _revLimiterTimer = REV_LIMIT_HOLD_TIME; // Activate the hold timer
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
}
