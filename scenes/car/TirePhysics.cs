using System;
using Godot;

public static class TirePhysics
{
    // Typical empirical coefficients for a standard passenger tire
    public static float B_long = 10.0f;
    public static float C_long = 1.65f;
    public static float E_long = 0.97f;

    public static float B_lat = 12.0f;
    public static float C_lat = 1.30f;
    public static float E_lat = -2.33f;

    /// <summary>
    /// Calculates the normalized Magic Formula coefficient (returns a factor between -1.0 and 1.0)
    /// </summary>
    private static float PacejkaFormula(float slip, float B, float C, float E)
    {
        float bx = B * slip;
        float insideArcTan = bx - E * (bx - Mathf.Atan(bx));
        return Mathf.Sin(C * Mathf.Atan(insideArcTan));
    }

    /// <summary>
    /// Calculates both longitudinal and lateral forces for a wheel.
    /// </summary>
    /// <param name="wheelLinearVelocity">The ground velocity vector at the wheel's position.</param>
    /// <param name="wheelForwardVector">Normalized vector pointing where the wheel is facing.</param>
    /// <param name="wheelAngularVelocity">Rotational speed of the wheel (rad/s).</param>
    /// <param name="tireRadius">Radius of the tire in meters.</param>
    /// <param name="verticalLoad">The downward force on this tire (Newton), affected by weight transfer.</param>
    /// <param name="frictionCoefficient">Surface friction modifier (e.g., 1.0 for asphalt, 0.3 for ice).</param>
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

        // 1. Longitudinal Force (Fx)
        float wheelLinearSpeed = wheelAngularVelocity * tireRadius;
        float slipRatio = (wheelLinearSpeed - vx) / Mathf.Abs(safeVx);

        // D parameter scales directly with normal load and surface friction
        float D_long = verticalLoad * frictionCoefficient;
        float fxMag = D_long * PacejkaFormula(slipRatio, B_long, C_long, E_long);

        // 2. Lateral Force (Fy)
        float slipAngle = Mathf.Atan2(vy, Mathf.Abs(safeVx));
        float D_lat = verticalLoad * frictionCoefficient;
        float fyMag = D_lat * PacejkaFormula(slipAngle, B_lat, C_lat, E_lat);

        // Friction Circle / Combined Slip Limitation (Simplification)
        // This ensures the tire cannot exceed total available physical friction
        float maxForce = verticalLoad * frictionCoefficient;
        float combinedForceMag = Mathf.Sqrt(fxMag * fxMag + fyMag * fyMag);

        if (combinedForceMag > maxForce && combinedForceMag > 0)
        {
            fxMag = (fxMag / combinedForceMag) * maxForce;
            fyMag = (fyMag / combinedForceMag) * maxForce;
        }

        // Convert scalar forces back to 2D world vectors
        // Note: Lateral force opposes the direction of lateral slip (hence the negative sign)
        Vector2 worldFx = wheelForwardVector * fxMag;
        Vector2 worldFy = wheelRightVector * -fyMag;

        return (worldFx + worldFy, slipRatio, slipAngle);
    }
}
