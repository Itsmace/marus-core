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

using Marus.Core;
using Marus.Networking;
using Sensor;
using Sensorstreaming;
using Std;
using UnityEngine;
using static Sensorstreaming.SensorStreaming;

namespace Marus.Sensors.Primitive
{
    /// <summary>
    /// Imu sensor implementation
    /// </summary>
    [RequireComponent(typeof(ImuSensor))]
    public class ImuROS : SensorStreamer<SensorStreamingClient, ImuStreamingRequest>
    {
        ImuSensor sensor;

        private ImuStreamingRequest _cachedRequest;
        private Imu _cachedImu;
        private Header _cachedHeader;
        private Geometry.Quaternion _cachedOrientation;
        private Geometry.Vector3 _cachedAngularVelocity;
        private Geometry.Vector3 _cachedLinearAcceleration;

        new void Start()
        {
            sensor = GetComponent<ImuSensor>();
            StreamSensor(sensor, streamingClient.StreamImuSensor);
            base.Start();  // sets address via SetAddresSufix

            _cachedHeader = new Header { FrameId = sensor.frameId };
            _cachedOrientation = new Geometry.Quaternion();
            _cachedAngularVelocity = new Geometry.Vector3();
            _cachedLinearAcceleration = new Geometry.Vector3();
            _cachedImu = new Imu
            {
                Header = _cachedHeader,
                Orientation = _cachedOrientation,
                AngularVelocity = _cachedAngularVelocity,
                LinearAcceleration = _cachedLinearAcceleration,
            };
            _cachedImu.OrientationCovariance.AddRange(sensor.orientationCovariance);
            _cachedImu.LinearAccelerationCovariance.AddRange(sensor.linearAccelerationCovariance);
            _cachedImu.AngularVelocityCovariance.AddRange(sensor.angularVelocityCovariance);
            _cachedRequest = new ImuStreamingRequest { Data = _cachedImu, Address = address };
        }

        protected override ImuStreamingRequest ComposeMessage()
        {
            _cachedHeader.Timestamp = TimeHandler.Instance.TimeDouble;

            var ori = sensor.orientation.Unity2Map();
            _cachedOrientation.X = ori.x;
            _cachedOrientation.Y = ori.y;
            _cachedOrientation.Z = ori.z;
            _cachedOrientation.W = ori.w;

            var angVel = (-sensor.angularVelocity).Unity2Body();
            _cachedAngularVelocity.X = angVel.x;
            _cachedAngularVelocity.Y = angVel.y;
            _cachedAngularVelocity.Z = angVel.z;

            var linAcc = sensor.linearAcceleration.Unity2Body();
            _cachedLinearAcceleration.X = linAcc.x;
            _cachedLinearAcceleration.Y = linAcc.y;
            _cachedLinearAcceleration.Z = linAcc.z;

            return _cachedRequest;
        }
    }
}