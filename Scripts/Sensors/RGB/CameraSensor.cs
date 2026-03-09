// Copyright 2022 Laboratory for Underwater Systems and Technologies (LABUST)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif
using Marus.Sensors.Core;

namespace Marus.Sensors
{
    [RequireComponent(typeof(Camera))]
    /// <summary>
    /// Camera sensor implementation
    /// </summary>
    public class CameraSensor : SensorBase
    {
        [Header("Camera Resolution")]
        public int ImageWidth = 1920;
        public int ImageHeight = 1080;

        Camera _camera;
        public RenderTexture DebugTexture => _renderTexture;
        
        RenderTexture _renderTexture;
        TextureFormat _textureFormat = TextureFormat.RGB24;
        Texture2D _texture;

        [HideInInspector]
        public byte[] Data;

        void Start()
        {
            ImageWidth = Mathf.Max(ImageWidth, 1);
            ImageHeight = Mathf.Max(ImageHeight, 1);

            _camera = GetComponent<Camera>();
            _camera.enabled = true; // Enable camera to avoid extra forced HDRP renderi pipeline
            
            _camera.aspect = (float)ImageWidth / ImageHeight;
            
            // Create persistent RenderTexture
            _renderTexture = new RenderTexture(ImageWidth, ImageHeight, 16);
            _camera.targetTexture = _renderTexture;
            
            Data = new byte[ImageHeight*ImageWidth*3];
            _texture = new Texture2D
            (
                ImageWidth,
                ImageHeight,
                _textureFormat,
                false
            );
        }

        protected override void SampleSensor()
        {
            RenderTexture.active = _renderTexture;
            //_camera.Render();
            AsyncGPUReadback.Request(_renderTexture, 0, _textureFormat, ReadbackCompleted);
        }

        void ReadbackCompleted(AsyncGPUReadbackRequest request)
        {
            //Debug.Log("Camera frame captured");
            
            Data = request.GetData<byte>().ToArray();
            hasData = true;
        }
        
        void OnDestroy()
        {
            if (_renderTexture != null)
            {
                _renderTexture.Release();
            }
        }
    }
}
