public class SurfaceCondition
{
    public float B { get; set; }
    public float C { get; set; }
    public float D { get; set; }
    public float E { get; set; }

    public SurfaceCondition(float b, float c, float d, float e)
    {
        B = b;
        C = c;
        D = d;
        E = e;
    }
}

public static class SurfaceConditions
{
    public static SurfaceCondition DryTarmac
    {
        get => new SurfaceCondition(10, 1.9f, 1, 0.97f);
    }
    public static SurfaceCondition WetTarmac
    {
        get => new SurfaceCondition(12, 2.3f, 0.82f, 1f);
    }
    public static SurfaceCondition Snow
    {
        get => new SurfaceCondition(5, 2f, 0.3f, 1f);
    }
    public static SurfaceCondition Ice
    {
        get => new SurfaceCondition(4, 2f, 0.1f, 1f);
    }
}
