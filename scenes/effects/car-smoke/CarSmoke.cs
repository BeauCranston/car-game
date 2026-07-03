using Godot;

public partial class CarSmoke : AnimatedSprite2D
{
    public override void _Ready()
    {
        AnimationFinished += OnAnimationFinished;
        Play();
    }

    private void OnAnimationFinished()
    {
        QueueFree();
    }
}
