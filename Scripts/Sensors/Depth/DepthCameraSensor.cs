using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections;
using Marus.Sensors.Core;

namespace Marus.Sensors
{
    public class DepthCameraSensor : SensorBase
    {
        public RenderTexture depthRT;

        public int ImageWidth => depthRT.width;
        public int ImageHeight => depthRT.height;
        // public string frameId = "depth_camera";

        public byte[] Data;

        bool readbackPending = false;

        protected override void SampleSensor()
        {
            if (readbackPending || depthRT == null)
                return;

            readbackPending = true;

            AsyncGPUReadback.Request(
                depthRT,
                0,
                TextureFormat.RFloat,
                ReadbackCompleted
            );
        }

        void ReadbackCompleted(AsyncGPUReadbackRequest request)
        {
            readbackPending = false;

            if (request.hasError)
                return;

            var floatData = request.GetData<float>();

            if (Data == null || Data.Length != floatData.Length)
                Data = new byte[floatData.Length];

            float minDepth = 0.3f;
            float maxDepth = 10f;

            for (int i = 0; i < floatData.Length; i++)
            {
                float d = floatData[i];
                float norm = Mathf.Clamp01((d - minDepth) / (maxDepth - minDepth));
                Data[i] = (byte)(norm * 255);
            }

            hasData = true; 
        }
    }
}
