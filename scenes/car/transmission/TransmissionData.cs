using Godot;

[GlobalClass]
public partial class TransmissionData : Resource
{
    [Export]
    public float FinalDriveRatio { get; private set; } = 4.06f; // Differential ratio

    [Export]
    private float FirstGear = 3.538f; // 1: 1st Gear

    [Export]
    private float SecondGear = 2.047f; // 2: 2nd Gear

    [Export]
    private float ThirdGear = 1.375f; // 3: 3rd Gear

    [Export]
    private float FourthGear = 1.025f; // 4: 4th Gear (Increased from 1.12)

    [Export]
    private float FifthGear = 0.875f; // 5: 5th Gear (Increased from 0.85)

    [Export]
    private float SixthGear = 0.733f; // 6: 6th Gear (Increased from 0.68)

    [Export]
    private float ReverseGear = -3.55f; // 7: Reverse Gear

    public float[] GearRatios
    {
        get
        {
            return new float[]
            {
                0.0f, // 0: Neutral
                FirstGear,
                SecondGear,
                ThirdGear,
                FourthGear,
                FifthGear,
                SixthGear,
                ReverseGear,
            };
        }
    }

    public int CurrentGearIndex = 1; // Start in 1st gear

    public void ShiftGearUp()
    {
        if (CurrentGearIndex == 7) // From Reverse to Neutral
            CurrentGearIndex = 0;
        else if (CurrentGearIndex < 6) // Normal sequential upshift
            CurrentGearIndex++;

        GD.Print($"Shifted UP to Gear: {GetGearName()}");
    }

    public void ShiftGearDown()
    {
        if (CurrentGearIndex == 0) // From Neutral to Reverse
            CurrentGearIndex = 7;
        else if (CurrentGearIndex > 0 && CurrentGearIndex <= 6) // Normal downshift
            CurrentGearIndex--;

        GD.Print($"Shifted DOWN to Gear: {GetGearName()}");
    }

    public float GetCurrentGearRatio()
    {
        return GearRatios[CurrentGearIndex];
    }

    private string GetGearName()
    {
        if (CurrentGearIndex == 0)
            return "Neutral";
        if (CurrentGearIndex == 7)
            return "Reverse";
        return $"{CurrentGearIndex}";
    }
}
