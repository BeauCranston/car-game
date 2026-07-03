using Godot;

public partial class ExhaustEffect : AnimatedSprite2D
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
