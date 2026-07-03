using System.Collections.Generic;
using System.Linq;
using Godot;

/// <summary>
/// Road manager that chunks road pieces based on the car's position relative to the chunks behind and in front of the car.
///
/// When chunking you must keep in mind that "Forward" is actually goind towards negative infinity, and "Backward" is actually going towards positive infinity
/// </summary>
/// <param name="parameterName">Parameter description.</param>
/// <returns>Type and description of the returned object.</returns>
/// <example>Write me later.</example>
public partial class RoadManager : Node2D
{
    [Export]
    public Node2D Car;

    private float _behindBuffer = 1024f;

    private float _aheadBuffer = 2048f;
    private float _chunkHeight = 1024f;
    private PackedScene _road = GD.Load<PackedScene>("res://scenes/roadChunks/road.tscn");
    private LinkedList<Node> _chunks;

    private LinkedList<Node> InitializeChunks(PackedScene roadScene)
    {
        var negativeDirection = roadScene.Instantiate<Sprite2D>();
        var center = roadScene.Instantiate<Sprite2D>();
        var positiveDirection = roadScene.Instantiate<Sprite2D>();
        _chunkHeight = center.Texture.GetHeight();
        center.GlobalPosition = Car.GlobalPosition;
        negativeDirection.GlobalPosition = new Vector2(
            center.GlobalPosition.X,
            center.GlobalPosition.Y - center.Texture.GetHeight()
        );
        positiveDirection.GlobalPosition = new Vector2(
            center.GlobalPosition.X,
            center.GlobalPosition.Y + center.Texture.GetHeight()
        );
        AddChild(negativeDirection);
        AddChild(center);
        AddChild(positiveDirection);

        return new LinkedList<Node>(GetChildren());
    }

    public override void _Ready()
    {
        _chunks = InitializeChunks(_road);
        // _behindBuffer = _chunkHeight;
        // _aheadBuffer = _chunkHeight * 2f;
    }

    public override void _Process(double delta)
    {
        MoveChunks();
    }

    private void MoveChunks()
    {
        // var backNode = _chunks.Last;
        // var frontNode = _chunks.First;
        // var backRoad = backNode.Value as Sprite2D;
        // var frontRoad = frontNode.Value as Sprite2D;
        //
        // GD.Print("Front edge back:", GetFrontEdgeY(backRoad));
        // GD.Print("Back edge front:", GetBackEdgeY(frontRoad));
        // GD.Print("Car Pos:", Car.GlobalPosition.Y);

        if (Car.GlobalPosition.Y > GetFrontEdgeY(_chunks.Last.Value as Sprite2D))
            MoveChunksBack();
        if (Car.GlobalPosition.Y < GetBackEdgeY(_chunks.First.Value as Sprite2D))
            MoveChunksForward();
    }

    private void MoveChunksBack()
    {
        var front = _chunks.First.Value;
        var currentBack = _chunks.Last.Value as Sprite2D;
        RemoveChild(front);
        _chunks.RemoveFirst();
        var newBack = _road.Instantiate<Sprite2D>();
        newBack.GlobalPosition = new Vector2(
            currentBack.GlobalPosition.X,
            currentBack.GlobalPosition.Y + _chunkHeight
        );
        AddChild(newBack);
        _chunks.AddLast(newBack);
        front.QueueFree();
    }

    private void MoveChunksForward()
    {
        var back = _chunks.Last.Value;
        var currentFront = _chunks.First.Value as Sprite2D;
        RemoveChild(back);
        _chunks.RemoveLast();
        var newFront = _road.Instantiate<Sprite2D>();
        newFront.GlobalPosition = new Vector2(
            currentFront.GlobalPosition.X,
            currentFront.GlobalPosition.Y - _chunkHeight
        );
        AddChild(newFront);
        _chunks.AddFirst(newFront);
        back.QueueFree();
    }

    private float GetFrontEdgeY(Sprite2D chunk)
    {
        // Forward is negative Y, so the front/top edge is center - half height.
        return chunk.GlobalPosition.Y - chunk.Texture.GetHeight() / 2f;
    }

    //
    private float GetBackEdgeY(Sprite2D chunk)
    {
        // Backward is positive Y, so the back/bottom edge is center + half height.
        return chunk.GlobalPosition.Y + chunk.Texture.GetHeight() / 2f;
    }
    //
    // private void MoveChunkToFront(Sprite2D chunk)
    // {
    //     float minY = GetChildren().OfType<Sprite2D>().Min(c => c.GlobalPosition.Y);
    //
    //     chunk.GlobalPosition = new Vector2(chunk.GlobalPosition.X, minY - _chunkHeight);
    // }
    //
    // private void MoveChunkToBack(Sprite2D chunk)
    // {
    //     float maxY = GetChildren().OfType<Sprite2D>().Max(c => c.GlobalPosition.Y);
    //
    //     chunk.GlobalPosition = new Vector2(chunk.GlobalPosition.X, maxY + _chunkHeight);
    // }
}
