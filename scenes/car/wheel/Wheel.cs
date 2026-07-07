using Godot;

public partial class Wheel : Marker2D
{
    // Physical / setup parameters – can be overridden per wheel in the inspector
    [Export]
    public float TireRadius { get; set; } = 0.33f;

    [Export]
    public float WheelInertia { get; set; } = 3f;

    // Dynamic state
    public float Omega { get; set; } = 0f; // rad/s

    // Cached reference to the car (parent RigidBody2D)
    private RigidBody2D _carBody;

    public override void _Ready()
    {
        _carBody = GetParent<RigidBody2D>();
        if (_carBody == null)
            GD.PrintErr($"{nameof(Wheel)} must be a child of a RigidBody2D car.");
    }

    public override void _PhysicsProcess(double delta) { }

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
            var preResult = TirePhysics.CalculateTireForces(
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
        var result = TirePhysics.CalculateTireForces(
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
            return;
        }

        // 7. Integrate omega
        float fxTire = result.forces.Dot(wheelForward);
        float tireFeedbackTorque = fxTire * TireRadius;
        float totalTorque = driveTorque - tireFeedbackTorque + prospectiveAppliedbrakeTorque;
        float angularAcceleration = totalTorque / WheelInertia;
        Omega += angularAcceleration * dt;
        Omega = Mathf.Clamp(Omega, -300f, 300f);
    }

    private Vector2 GetVelocityAtWheel()
    {
        Vector2 globalOffset = GlobalPosition - _carBody.GlobalPosition;
        Vector2 tangentialVelocity =
            new Vector2(-globalOffset.Y, globalOffset.X) * _carBody.AngularVelocity;
        return (_carBody.LinearVelocity + tangentialVelocity)
            / GameSettings.Instance.PixelsPerMeter;
    }
}
