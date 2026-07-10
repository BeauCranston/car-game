Yes, but with an important caveat:

▌ There is no purely correct Pacejka slip-ratio equation at exactly zero speed, because  
▌ slip ratio is mathematically ill-defined there.

So you either need:

1 a separate low-speed/static model, or  
 2 a single unified model with regularized slip math and wheel-settling constraints.

If you want to avoid “two implementations,” I would use option 2.

---

What is actually causing your issue

Your wheel is oscillating around the rolling solution.

Example from your debug:

sampledOmega:-3.30, omega:1.17

That means:

1 Tire force was calculated with Omega = -3.30.  
 2 Then wheel integration pushed Omega past zero to 1.17.  
 3 Next tick it may flip back again.

So the tire never settles to:

Omega ≈ forwardSpeed / TireRadius

Instead, it keeps creating artificial slip, which creates artificial heat.

---

Math fix 1: use a smoother regularized denominator

Right now you use:

float slipDenominator = Mathf.Max(  
 Mathf.Max(Mathf.Abs(forwardSpeed), Mathf.Abs(wheelSurfaceSpeed)),  
 MinimumSlipSpeed  
);

That works, but it has a hard corner because of Max.

A smoother version is:

float speedReference = Mathf.Sqrt(  
 forwardSpeed _ forwardSpeed  
 + wheelSurfaceSpeed _ wheelSurfaceSpeed  
 + MinimumSlipSpeed \* MinimumSlipSpeed  
);

float slipRatio = longitudinalSlipSpeed / speedReference;  
float slipAngle = Mathf.Atan2(lateralSpeed, speedReference);

This keeps the same Pacejka implementation, but makes the slip values behave more smoothly  
near zero speed.

So in CalculateTireContactPatch() replace:

float slipDenominator = Mathf.Max(  
 Mathf.Max(Mathf.Abs(forwardSpeed), Mathf.Abs(wheelSurfaceSpeed)),  
 MinimumSlipSpeed  
);  
float slipRatio = SnapToZero(longitudinalSlipSpeed / slipDenominator);  
float slipAngle = SnapToZero(Mathf.Atan2(lateralSpeed, slipDenominator));

with:

float speedReference = Mathf.Sqrt(  
 forwardSpeed _ forwardSpeed  
 + wheelSurfaceSpeed _ wheelSurfaceSpeed  
 + MinimumSlipSpeed \* MinimumSlipSpeed  
);

float slipRatio = SnapToZero(longitudinalSlipSpeed / speedReference);  
float slipAngle = SnapToZero(Mathf.Atan2(lateralSpeed, speedReference));

This is still one tire model.

---

Math fix 2: settle Omega to the rolling solution

This is probably the bigger fix.

For a freely rolling wheel, the target angular velocity is:

float rollingOmega = forwardSpeed / TireRadius;

When there is no drive torque and no braking, the tire feedback torque should make the wheel
approach that rolling value, not overshoot forever.

Right now IntegrateWheelAngularVelocity() only knows about tire force. It does not know the  
correct rolling omega.

Change the call from:

IntegrateWheelAngularVelocity(  
 dt,  
 driveTorque,  
 brakeTorque,  
 brakeInput,  
 contactPatch.LongitudinalForce  
);

to:

IntegrateWheelAngularVelocity(  
 dt,  
 driveTorque,  
 brakeTorque,  
 brakeInput,  
 contactPatch.LongitudinalForce,  
 contactPatch.ForwardSpeed  
);

Then change the method signature:

private void IntegrateWheelAngularVelocity(  
 float dt,  
 float driveTorque,  
 float brakeTorque,  
 float brakeInput,  
 float longitudinalTireForce,  
 float forwardSpeed  
)

Inside it, after calculating nextOmega, add this:

float rollingOmega = forwardSpeed / TireRadius;

float currentSlipOmega = Omega - rollingOmega;  
float nextSlipOmega = nextOmega - rollingOmega;

bool noDriveTorque = Mathf.Abs(driveTorque) < TinyValue;  
bool noBrakeTorque = brakeCapacity < TinyValue;  
bool crossedRollingOmega =  
 Mathf.Abs(currentSlipOmega) > TinyValue  
 && Mathf.Sign(currentSlipOmega) != Mathf.Sign(nextSlipOmega);

if (noDriveTorque && noBrakeTorque && crossedRollingOmega)  
{  
 nextOmega = rollingOmega;  
}

Then continue:

Omega = Mathf.Clamp(nextOmega, -MaximumWheelAngularVelocity, MaximumWheelAngularVelocity);  
Omega = SnapToZero(Omega);

This prevents the wheel from endlessly flipping around the rolling solution.

This is not a second tire model. It is just a better wheel angular integration constraint.

---

Math fix 3: remove the conflicting rest snap

Your current UpdatePhysics() has both of these:

if (noDrive && noBrake && nearlyStopped)  
{  
 Omega = forwardSpeedForSleep / TireRadius;  
 \_smoothedFrictionWatts = 0f;  
}

if (ShouldSnapWheelToRest(wheelVelocity, driveTorque, brakeInput))  
{  
 Omega = 0f;  
}

These can fight each other.

The first one says:

Omega = forwardSpeed / TireRadius;

The second one says:

Omega = 0f;

For a rolling wheel, the first one is usually more correct.

I would remove ShouldSnapWheelToRest() entirely, or only use it when the car body is truly  
sleeping/stopped.

Also, your current file references:

WheelSleepSpeed  
WheelSleepAngularSpeed

but those constants/properties are not defined in the provided Wheel.cs. That will not  
compile unless they exist somewhere else. You probably intended to add:

[Export]  
public float WheelSleepSpeed { get; set; } = 0.25f;

[Export]  
public float WheelSleepAngularSpeed { get; set; } = 0.5f;

---

Best unified approach

If you do not want a separate low-speed tire implementation, I would do this:

1 Keep one Pacejka force calculation.  
 2 Use smooth regularized slip:

speedReference = sqrt(vx² + wheelSurfaceSpeed² + minSlipSpeed²)

3 Add rolling-omega settling in wheel angular integration.  
 4 Use car-level sleep for actual full stop.

That gives you one tire model, but avoids the zero-speed singularity and the Omega  
sign-flipping problem.

---

---

Important reality check

Even with better math, you still need some kind of rest/sleep handling in a game physics

simulation.

Real tire physics has static friction constraints. Your model is force-based and explicit,
so
at rest it can jitter forever unless you either:

• solve static contact constraints, or

• regularize/sleep small values.

So no, you probably cannot solve this with “just Pacejka + epsilon.” But yes, you can avoi
d a
totally separate low-speed tire implementation by using a unified regularized Pacejka mode
l  
plus a rolling-omega constraint.
