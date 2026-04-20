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

using Marus.Visualization;
using Unity.Collections;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System;
using Marus.ObjectAnnotation;
using Marus.Utils;

namespace Marus.Sensors
{

    /// <summary>
    /// Lidar implemented using rays
    /// Implemented using IJobParallelFor on CPU
    /// Can drop performance
    /// </summary>
    public class RaycastLidar : SensorBase
    {

        // Disable config mode
        public bool UseConfig = false; 
        /// <summary>
        /// Number of horizontal rays
        /// </summary>
        public int WidthRes = 1024;

        /// <summary>
        /// Number of vertical rays
        /// </summary>
        public int HeightRes = 16;

        /// <summary>
        /// Maximum range in meters
        /// </summary>
        public float MaxDistance = 100;

        /// <summary>
        /// Minimum range in meters
        /// </summary>
        public float MinDistance = 0.2f;

        public float VerticalFieldOfView = 30;
        public float HorizontalFieldOfView = 360;

        /// <summary>
        /// PointCloud compute shader
        /// </summary>
        public ComputeShader pointCloudShader;

        /// <summary>
        /// Material set for point cloud display
        /// </summary>
        public Material ParticleMaterial;

        public NativeArray<Vector3> Points;
        public NativeArray<LidarReading> Readings;


        PointCloudManager _pointCloudManager;
        RaycastJobHelper<LidarReading> _raycastHelper;
        Coroutine _coroutine;
        [HideInInspector] public List<LidarConfig> Configs;
        [HideInInspector]  public int ConfigIndex = 0;
        [SerializeField] [HideInInspector] public NativeArray<(float, float)> _rayAngles;
        public List<RayInterval> _rayIntervals;
        public RayDefinitionType _rayType;

        private PointCloudSegmentationSaver _saver;
        private bool saverExists;

        [HideInInspector]
        private Dictionary<int, int> colliderLayer;

        public LayerMask raycastMask;
        public LayerMask blackHoleLayers;

        void Start()
        {
            colliderLayer = new Dictionary<int, int>();

            if (blackHoleLayers != 0)
            {
                GameObject[] objects = Helpers.FindGameObjectsInLayerMask(blackHoleLayers);

                foreach (var obj in objects)
                {
                    Collider instId = obj.GetComponent<Collider>();
                    if (instId)
                    {
                        colliderLayer.Add(instId.GetInstanceID(), instId.gameObject.layer);
                    }
                }
            }

            _saver = GetComponent<PointCloudSegmentationSaver>();
            saverExists = _saver is not null && _saver.isActiveAndEnabled;

            // Apply config ONLY if enabled
            if (UseConfig && Configs != null && Configs.Count > 0)
            {
                ApplyLidarConfig();
            }

            // ALWAYS initialize rays first
            InitializeRayArray();

            // REAL number of rays
            int totalRays = _rayAngles.Length;

            // Stop old coroutine and dispose old helper before re-allocating
            if (_coroutine != null)
            {
                StopCoroutine(_coroutine);
                _coroutine = null;
            }
            _raycastHelper?.Dispose();

            if (Points.IsCreated) Points.Dispose();
            if (Readings.IsCreated) Readings.Dispose();

            Points = new NativeArray<Vector3>(totalRays, Allocator.Persistent);
            Readings = new NativeArray<LidarReading>(totalRays, Allocator.Persistent);

            var directionsLocal = RaycastJobHelper.CalculateRayDirections(_rayAngles);

            _raycastHelper = new RaycastJobHelper<LidarReading>(
                gameObject,
                directionsLocal,
                OnLidarHit,
                OnFinish,
                maxDistance: MaxDistance,
                minDistance: MinDistance,
                sampleFrequency: SampleFrequency,
                layerMask: raycastMask
            );

            if (ParticleMaterial == null)
                ParticleMaterial = PointCloudManager.FindMaterial("PointMaterial");
            if (pointCloudShader == null)
                pointCloudShader = PointCloudManager.FindComputeShader("PointCloudCS");

            _pointCloudManager = PointCloudManager.CreatePointCloud(gameObject,name + "_PointCloud", totalRays, ParticleMaterial, pointCloudShader);

            _coroutine = StartCoroutine(_raycastHelper.RaycastInLoop());

        }

        protected override void SampleSensor()
        {
            _pointCloudManager.UpdatePointCloud(Points);
            _raycastHelper.SampleFrequency = SampleFrequency;
        }

        private void OnFinish(NativeArray<Vector3> points, NativeArray<LidarReading> readings)
        {
            points.CopyTo(this.Points);
            readings.CopyTo(this.Readings);
            hasData = true;
        }

        private LidarReading OnLidarHit(RaycastHit hit, Vector3 direction, int index)
        {
            var reading = new LidarReading();
            (int, int) value;
            if (saverExists)
            {
                if (_saver.objectClassesAndInstances.TryGetValue(hit.colliderInstanceID, out value))
                {
                    reading.ClassId = value.Item1;
                    reading.InstanceId = value.Item2;
                }
            }
            if (hit.colliderInstanceID is not 0)
            {
                reading.IsValid = true;
            }
            if(colliderLayer.Count > 0)
            {
                int layer;
                if(colliderLayer.TryGetValue(hit.colliderInstanceID, out layer))
                {
                    reading.IsValid = false;
                }
            }
            reading.Intensity = 255;
            return reading;
        }

        /// <summary>
        /// This method applies parameters and configuration
        /// Active configuration is selected using dropdown from inspector
        /// </summary>
        public void ApplyLidarConfig()
        {
            if (!UseConfig) return;   // 👈 HARD STOP

            if (_rayAngles.IsCreated)
            {
                _rayAngles.Dispose();
            }

            var cfg = Configs[ConfigIndex];

            MaxDistance = cfg.MaxRange;
            MinDistance = cfg.MinRange;
            WidthRes = cfg.HorizontalResolution;
            HeightRes = cfg.VerticalResolution;
            HorizontalFieldOfView = cfg.HorizontalFieldOfView;
            VerticalFieldOfView = cfg.VerticalFieldOfView;
            SampleFrequency = cfg.Frequency;
            _rayType = cfg.Type;
            _rayIntervals = cfg.RayIntervals;

            if (cfg.Type == RayDefinitionType.Angles)
            {
                HeightRes = cfg.ChannelAngles.Count;
            }
            else if (cfg.Type == RayDefinitionType.Intervals)
            {
                if (_rayIntervals != null && _rayIntervals.Count > 0)
                {
                    HeightRes = _rayIntervals.Sum(x => x.NumberOfRays);
                    VerticalFieldOfView =
                        _rayIntervals.Last().EndingAngle - _rayIntervals.First().StartingAngle;
                }
            }
        }

        /// <summary>
        /// Initializes ray directions from ray angles, custom ray intervals or uniform distribution.
        /// These directions emulate lidar vertical array rotating to get the surrounding pointcloud
        /// </summary>
        public void InitializeRayArray()
        {
            if (_rayAngles.IsCreated) _rayAngles.Dispose();

            if (UseConfig && Configs != null && Configs.Count > 0)
            {
                var cfg = Configs[ConfigIndex];

                if (cfg.Type == RayDefinitionType.Intervals)
                {
                    var angles = RaycastJobHelper.InitVerticalAnglesFromIntervals(
                        _rayIntervals,
                        WidthRes,
                        HorizontalFieldOfView
                    );

                    _rayAngles = RaycastJobHelper.InitCustomRays(
                        angles,
                        cfg.HorizontalResolution,
                        HorizontalFieldOfView
                    );
                }
                else if (cfg.Type == RayDefinitionType.Uniform)
                {
                    _rayAngles = RaycastJobHelper.InitUniformRays(
                        WidthRes,
                        HeightRes,
                        HorizontalFieldOfView,
                        VerticalFieldOfView
                    );
                }
                else if (cfg.Type == RayDefinitionType.Angles)
                {
                    HeightRes = cfg.ChannelAngles.Count;

                    _rayAngles = RaycastJobHelper.InitCustomRays(
                        cfg.ChannelAngles,
                        cfg.HorizontalResolution,
                        HorizontalFieldOfView
                    );
                }
            }
            else
            {
                // PURE manual mode (no config interference)
                _rayAngles = RaycastJobHelper.InitUniformRays(
                    WidthRes,
                    HeightRes,
                    HorizontalFieldOfView,
                    VerticalFieldOfView
                );
            }
        }

        void OnDestroy()
        {
            if (_coroutine != null)
            {
                StopCoroutine(_coroutine);
                _coroutine = null;
            }
            _raycastHelper?.Dispose();

            if (Points.IsCreated) Points.Dispose();
            if (Readings.IsCreated) Readings.Dispose();
            if (_rayAngles.IsCreated) _rayAngles.Dispose();
        }
    }

    /// <summary>
    /// Class containing all needed lidar properties.
    /// </summary>
    [System.Serializable]
    public class LidarConfig
    {
        public string Name;
        public RayDefinitionType Type;
        public float Frequency;
        public string FrameId;
        public float MaxRange;
        public float MinRange;
        public int HorizontalResolution;
        public int VerticalResolution;
        public float HorizontalFieldOfView;
        public float VerticalFieldOfView;
        public List<float> ChannelAngles;
        public List<RayInterval> RayIntervals;
    }

    /// <summary>
    /// Uniform - rays are distributed uniformly
    /// Intervals - rays are distributed based on given angle intervals with number of rays
    /// Angles - rays are distributed based on given angles of vertical rays
    /// </summary>
    public enum RayDefinitionType
    {
        Uniform,
        Intervals,
        Angles
    }

    public struct LidarReading
    {
        public int ClassId;
        public int InstanceId;
        public ushort Intensity;
        public bool IsValid;
    }

    /// <summary>
    /// Ray interval definition
    /// </summary>
    [Serializable]
	public class RayInterval
	{
		public float StartingAngle;
		public float EndingAngle;
		public int NumberOfRays;
	}
}
