using System;
using Godot;

public partial class Main : Node2D
{
    private Car _car;

    // private Parallax2D _roadParalax;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        _car = GetNode<Car>("Car");
        // _roadParalax = GetNode<Parallax2D>("Road");
        // _car.VelocityChanged += OnCarVelocityChanged;
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta) { }

    private void OnCarVelocityChanged(Vector2 velocity)
    {
        // _roadParalax.ScrollOffset += (new Vector2(0, -velocity.Y)) * (float)GetProcessDeltaTime();
    }
}
