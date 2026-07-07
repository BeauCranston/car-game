# Engine Torque Curve

## Specifications

- Idle RPM: ~1,000 RPM (normalized X = 0.0)
- Redline RPM: 6,200 RPM (normalized X = 1.0)
- Max Engine Torque: 233 N·m (172 lb-ft) at 4,100 RPM
- Max Power: 179 hp at 6,000 RPM

## Axis Normalization

The Curve resource uses an X axis ranging from 0.0 (Idle) to 1.0 (Redline).  
To place a given engine speed on this axis we compute:

\[
\text{Normalized } X = \frac{\text{Target RPM} - \text{Idle RPM}}{\text{Redline RPM} - \text{Idle RPM}}
\]

Key positions, derived from the above formula:

| Event            | Target RPM  | Calculation                     | Normalized X |
| ---------------- | ----------- | ------------------------------- | ------------ |
| Peak Torque      | 4,100 RPM   | (4100‑1000)/(6200‑1000)         | 0.596        |
| Peak Horsepower  | 6,000 RPM   | (6000‑1000)/(6200‑1000)         | 0.961        |

## Configuring the EngineData Resource

In the inspector for your `EngineData` resource, set the following values:

- **MaxEngineTorque**: 233
- **IdleRPM**: 1000
- **RedlineRPM**: 6200

## Godot Torque Curve Points

Open the `TorqueCurve` visual graph editor and add the points listed below.
These coordinates replicate the natural breathing behavior of Toyota’s dual VVT‑i system.

| Point # | X (RPM ratio) | Y (Torque %) | Context                                                                          |
| ------- | ------------- | ------------ | -------------------------------------------------------------------------------- |
| 0       | 0.0           | 0.78         | Idle (~1,000 RPM) – engine starts at roughly 78 % of max torque                  |
| 1       | 0.38          | 0.95         | Low mid‑range (~3,000 RPM) – torque rises as variable valve timing engages       |
| 2       | 0.596         | 1.0          | Peak torque (4,100 RPM) – maximum pulling power (100 %)                          |
| 3       | 0.961         | 0.85         | Peak horsepower (6,000 RPM) – torque drops to ~85 % to sustain peak power        |
| 4       | 1.0           | 0.77         | Redline (6,200 RPM) – torque chokes off just before the rev limiter              |
