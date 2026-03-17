using UnityEngine;

public static class ThrusterCurveUtils
{
    public static AnimationCurve FromArray(float[] values, float step)
    {
        AnimationCurve curve = new AnimationCurve();

        for (int i = 0; i < values.Length; i++)
        {
            float x = -1f + i * step;
            float y = values[i];
            curve.AddKey(x, y);
        }

        return curve;
    }
}
