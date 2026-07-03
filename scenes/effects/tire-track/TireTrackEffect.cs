using Godot;

public partial class TireTrackEffect : Sprite2D
{
    private Vector2 _startGlobalPosition;
    private double _lifeTime = 8;

    public override async void _Ready()
    {
        ZIndex = 0;
        _startGlobalPosition = GlobalPosition;
        await ToSignal(GetTree().CreateTimer(_lifeTime), SceneTreeTimer.SignalName.Timeout);
        QueueFree();
    }
}
