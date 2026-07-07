using System;
using Godot;

public partial class GameSettings : Node
{
    public static GameSettings Instance { get; private set; }

    [Export]
    public float PixelsPerMeter { get; set; } = 100f;

    // Add any other global settings here, e.g.:
    [Export]
    public float Gravity { get; set; } = 9.81f;

    [Export]
    public float DefaultFriction { get; set; } = 0.9f;

    public override void _Ready()
    {
        Instance = this;
    }
}
