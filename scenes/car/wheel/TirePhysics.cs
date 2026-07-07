using System;
using Godot;

public static class TirePhysics
{
    // Typical empirical coefficients for a standard passenger tire
    public static float B_long = 10.0f;
    public static float C_long = 1.65f;
    public static float E_long = 0.97f;

    public static float B_lat = 7.0f;
    public static float C_lat = 1.30f;
    public static float E_lat = -0.73f;

    /// <summary>
    /// Calculates the normalized Magic Formula coefficient (returns a factor between -1.0 and 1.0)
    /// </summary>
    private static float PacejkaFormula(float slip, float B, float C, float E)
    {
        float bx = B * slip;
        float insideArcTan = bx - E * (bx - Mathf.Atan(bx));
        return Mathf.Sin(C * Mathf.Atan(insideArcTan));
    }

    public static (Vector2 forces, float slipRatio, float slipAngle) CalculateTireForces(
        Vector2 wheelLinearVelocity,
        Vector2 wheelForwardVector,
        float wheelAngularVelocity,
        float tireRadius,
        float verticalLoad,
        float frictionCoefficient = 1.0f
    )
    {
        Vector2 wheelRightVector = new Vector2(-wheelForwardVector.Y, wheelForwardVector.X); // 2D Perpendicular

        // Transform global velocity to wheel local space
        float vx = wheelLinearVelocity.Dot(wheelForwardVector);
        float vy = wheelLinearVelocity.Dot(wheelRightVector);

        // Safeguard for division by zero near zero velocity
        float epsilon = 0.1f;
        float safeVx = Mathf.Abs(vx) < epsilon ? epsilon * Mathf.Sign(vx) : vx;
        if (safeVx == 0)
            safeVx = epsilon;

        // 1. Calculate Pure Longitudinal Force (Fx)
        float wheelLinearSpeed = wheelAngularVelocity * tireRadius;
        float slipRatio = (wheelLinearSpeed - vx) / Mathf.Abs(safeVx);

        float D_long = verticalLoad * frictionCoefficient;
        float fxMag = D_long * PacejkaFormula(slipRatio, B_long, C_long, E_long);

        // 2. Calculate Pure Lateral Force (Fy)
        float slipAngle = Mathf.Atan2(vy, Mathf.Abs(safeVx));

        // FIXED: Add a micro-deadzone for straight-line driving (approx. less than 0.15 degrees of slip)
        // This prevents high-speed calculation jitter from generating phantom cornering drag
        if (Mathf.Abs(slipAngle) < 0.0025f)
        {
            slipAngle = 0f;
        }

        float D_lat = verticalLoad * frictionCoefficient;
        float fyMag = D_lat * PacejkaFormula(slipAngle, B_lat, C_lat, E_lat);

        // 3. Normalized Slip Vector Scaling (Pacejka Combined Model)
        // This looks at how much the tire is sliding in both directions combined
        float absoluteSlipX = Mathf.Abs(slipRatio);
        float absoluteSlipY = Mathf.Abs(Mathf.Tan(slipAngle));
        float combinedSlip = Mathf.Sqrt(
            absoluteSlipX * absoluteSlipX + absoluteSlipY * absoluteSlipY
        );

        if (combinedSlip > 0.001f)
        {
            // Reduce each force smoothly based on how much grip the OTHER direction is taking
            fxMag *= (absoluteSlipX / combinedSlip);
            fyMag *= (absoluteSlipY / combinedSlip);
        }

        // 4. Absolute Friction Circle Limit Safeguard
        // Ensures physical bounds are never breached
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
}
