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
}
