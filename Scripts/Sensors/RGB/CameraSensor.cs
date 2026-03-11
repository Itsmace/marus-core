using UnityEngine;
using UnityEngine.Rendering;
using Marus.Sensors.Core;

namespace Marus.Sensors
{
    [RequireComponent(typeof(Camera))]
    public class CameraSensor : SensorBase
    {
        public int ImageWidth = 1920;
        public int ImageHeight = 1080;

        float nextCaptureTime = 0;
        bool readbackPending = false;

        Camera _camera;
        RenderTexture _renderTexture;

        TextureFormat _readbackFormat = TextureFormat.RGB24;
        int _channels = 3;

        public RenderTexture DebugTexture => _renderTexture;

        [HideInInspector]
        public byte[] Data;

        void Start()
        {
            _camera = GetComponent<Camera>();

            _camera.aspect = (float)ImageWidth / ImageHeight;

            _renderTexture = new RenderTexture(
                ImageWidth,
                ImageHeight,
                0,
                RenderTextureFormat.ARGB32
            );

            _renderTexture.Create();
            _camera.targetTexture = _renderTexture;

            _camera.enabled = true;

            Data = new byte[ImageHeight * ImageWidth * _channels];
        }

        protected override void SampleSensor()
        {
            if (Time.time < nextCaptureTime || readbackPending)
                return;

            nextCaptureTime = Time.time + 1f / SampleFrequency;

            readbackPending = true;

            AsyncGPUReadback.Request(
                _renderTexture,
                0,
                _readbackFormat,
                ReadbackCompleted
            );
        }

        void ReadbackCompleted(AsyncGPUReadbackRequest request)
        {
            readbackPending = false;

            if (request.hasError)
                return;

            var raw = request.GetData<byte>();
            raw.CopyTo(Data);

            hasData = true;
        }
    }
}
