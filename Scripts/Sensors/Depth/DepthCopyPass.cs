using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class DepthCopyPass : CustomPass
{
    public RenderTexture target;
    public Material material;

    protected override void Execute(CustomPassContext ctx)
    {
        if (material == null || target == null)
            return;

        // Set render target
        CoreUtils.SetRenderTarget(ctx.cmd, target);

        // Clear it
        ctx.cmd.ClearRenderTarget(false, true, Color.black);

        // 🔥 THIS is the important fix
        CoreUtils.DrawFullScreen(
            ctx.cmd,
            material,
            shaderPassId: 0
        );
    }
}
