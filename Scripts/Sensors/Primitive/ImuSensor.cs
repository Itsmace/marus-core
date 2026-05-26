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

using System;
using Marus.NoiseDistributions;
using Marus.Utils;
using Std;
using UnityEngine;

namespace Marus.Sensors.Primitive
{
    /// <summary>
    /// Imu sensor implementation
    /// </summary>
    public class ImuSensor : SensorBase
    {
        public bool withGravity = true;
        public bool debug = true;
        [NonSerialized] public Vector3 linearAcceleration;
        [NonSerialized] public Vector3 localVelocity;
        [NonSerialized] public double[] linearAccelerationCovariance = new double[9];

        [NonSerialized] public Vector3 angularVelocity;
        [NonSerialized] public double[] angularVelocityCovariance = new double[9];

        [NonSerialized]public Vector3 eulerAngles;
        [NonSerialized]public Quaternion orientation;
        [NonSerialized] public double[] orientationCovariance = new double[9];


        [Header("Accelerometer")]
        public NoiseParameters AccelerometerNoise;
        [ReadOnly] public Vector3 LinearAcceleration;
        [ReadOnly] public Vector3 LocalVelocity;

        [Header("Gyro")]
        public NoiseParameters GyroNoise;
        [ReadOnly] public Vector3 AngularVelocity;

        [Header("Orientation")]
        public NoiseParameters OrientationNoise;
        [ReadOnly] public Vector3 EulerAngles;
        [ReadOnly] public Quaternion Orientation;
        private Rigidbody veh_rb;
        private Vector3 lastVelocity = Vector3.zero;
        private double _lastSampleTime;


        new void Reset()
        {
            base.Reset();
            UpdateVehicle();
        }

        new void UpdateVehicle()
        {
            base.UpdateVehicle();

            var veh = vehicle;
            veh_rb = veh.GetComponent<Rigidbody>();

            if (veh == transform || veh_rb is null)
                veh_rb = Helpers.GetComponentInParents<Rigidbody>(veh.parent?.gameObject);

            if (veh_rb == null)
                Debug.LogWarning($"ImuSensor on {gameObject.name}: no vehicle Rigidbody found.");
        }

        void Start()
        {
            UpdateVehicle();
        }

        protected override void SampleSensor()
        {
            if (veh_rb == null) return;

            double timeElapsed = Time.timeAsDouble - _lastSampleTime;
            _lastSampleTime = Time.timeAsDouble;

            // lever-arm: velocity at IMU position = v_CoM + ω × r
            Vector3 r = transform.position - veh_rb.worldCenterOfMass;
            Vector3 worldVelocity = veh_rb.velocity + Vector3.Cross(veh_rb.angularVelocity, r);
            localVelocity = transform.InverseTransformVector(worldVelocity);
            localVelocity[0]+=Noise.Sample(AccelerometerNoise);
            localVelocity[1]+=Noise.Sample(AccelerometerNoise);
            localVelocity[2]+=Noise.Sample(AccelerometerNoise);
            if (timeElapsed > 0)
                linearAcceleration = (localVelocity - lastVelocity) / (float)timeElapsed;

            angularVelocity = veh_rb.angularVelocity;
            angularVelocity[0]+=Noise.Sample(GyroNoise);
            angularVelocity[1]+=Noise.Sample(GyroNoise);
            angularVelocity[2]+=Noise.Sample(GyroNoise);

            eulerAngles = transform.rotation.eulerAngles;
            eulerAngles.x += Noise.Sample(OrientationNoise);
            eulerAngles.y += Noise.Sample(OrientationNoise);
            eulerAngles.z += Noise.Sample(OrientationNoise);
            orientation = Quaternion.Euler(eulerAngles);

            lastVelocity = localVelocity;

            if (withGravity)
                linearAcceleration -= transform.InverseTransformVector(UnityEngine.Physics.gravity);

            if (debug)
            {
                LinearAcceleration = linearAcceleration.Round(2);
                LocalVelocity = localVelocity.Round(2);
                AngularVelocity = angularVelocity.Round(2);
                EulerAngles = eulerAngles.Round(2);
                Orientation = orientation.Round(2);
            }
            Log(new { linearAcceleration, angularVelocity, eulerAngles, localVelocity });
            hasData = true;
        }
    }
}