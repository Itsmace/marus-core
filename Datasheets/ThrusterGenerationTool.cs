#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using Marus.Actuators;
using Marus.Actuators.Datasheets;

public class ThrusterGenerationTool
{
    [MenuItem("Tools/Generate All Thruster Curves")]
    public static void GenerateAll()
    {
        GenerateT200();
        GenerateT500();
    }

    static void GenerateT200()
    {
        Create("T200-V10", T200ThrusterDatasheet.V10, T200ThrusterDatasheet.step);
        Create("T200-V16", T200ThrusterDatasheet.V16, T200ThrusterDatasheet.step);
        Create("T200-V20", T200ThrusterDatasheet.V20, T200ThrusterDatasheet.step);
        
        // Optional scaled version
        Create("T200-V20x10", T200ThrusterDatasheet.V20x10, T200ThrusterDatasheet.step);
        
    }

    static void GenerateT500()
    {
        Create("T500-V12", T500ThrusterDatasheet.V12, T500ThrusterDatasheet.step);
        Create("T500-V16", T500ThrusterDatasheet.V16, T500ThrusterDatasheet.step);
        Create("T500-V20", T500ThrusterDatasheet.V20, T500ThrusterDatasheet.step);
        Create("T500-V22", T500ThrusterDatasheet.V22, T500ThrusterDatasheet.step);
        Create("T500-V24", T500ThrusterDatasheet.V24, T500ThrusterDatasheet.step);

        // Optional scaled version
        Create("T500-V24x10", T500ThrusterDatasheet.V24x10, T500ThrusterDatasheet.step);
    }

    static void Create(string name, float[] data, float step)
    {
        var asset = ScriptableObject.CreateInstance<ThrusterAsset>();
        asset.name = name;
        asset.curve = FromArray(data, step);

        AssetDatabase.CreateAsset(asset, $"Assets/marus-core/Datasheets/{name}.asset");
    }

    static AnimationCurve FromArray(float[] values, float step)
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
#endif
