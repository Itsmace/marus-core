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

using Marus.Networking;
using UnityEngine;
using Sensorstreaming;
using Marus.Core;
using Sensor;
using System;
using System.Collections.Generic;
using Unity.Collections;
using static Sensorstreaming.SensorStreaming;

namespace Marus.Sensors
{

    /// <summary>
    /// Lidar that cast N rays evenly distributed in configured field of view.
    /// Implemented using IJobParallelFor on CPU
    /// Can drop performance
    /// </summary>
    [RequireComponent(typeof(RaycastLidar))]
    public class RaycastLidarPointCloud2ROS : SensorStreamer<SensorStreamingClient, PointCloud2StreamingRequest>
    {
        RaycastLidar sensor;

        new void Start()
        {
            sensor = GetComponent<RaycastLidar>();
            StreamSensor(sensor,
                streamingClient.StreamPointCloud2);
            base.Start();
        }

        private PointCloud2 GeneratePointCloud2(NativeArray<Vector3> points, NativeArray<LidarReading> readings)
        {
            // Count valid hits to exclude miss rays (stored as Vector3.zero)
            int validCount = 0;
            for (int i = 0; i < readings.Length; i++)
                if (readings[i].IsValid) validCount++;

            // Pack only valid points using Unity2Body convention (FLU: x=forward, y=left, z=up).
            // TfStreamerROS also uses Unity2Body for the sensor frame, so the convention must match.
            // Unity sensor-local: x=right, y=up, z=forward
            // ROS FLU:            x=forward(Uz), y=left(-Ux), z=up(Uy)
            byte[] bytes = new byte[validCount * 12];
            int byteIdx = 0;
            for (int i = 0; i < points.Length; i++)
            {
                if (!readings[i].IsValid) continue;
                var p = points[i];
                Buffer.BlockCopy(BitConverter.GetBytes( p.z), 0, bytes, byteIdx,     4); // x = forward
                Buffer.BlockCopy(BitConverter.GetBytes(-p.x), 0, bytes, byteIdx + 4, 4); // y = left
                Buffer.BlockCopy(BitConverter.GetBytes( p.y), 0, bytes, byteIdx + 8, 4); // z = up
                byteIdx += 12;
            }

            PointCloud2 pointCloud = new PointCloud2();
            pointCloud.Header = new Std.Header()
            {
                FrameId = sensor.frameId,
                Timestamp = TimeHandler.Instance.TimeDouble
            };
            pointCloud.Height = 1;
            pointCloud.Width = (uint)validCount;
            pointCloud.Fields.AddRange(
                new List<PointField>()
                {
                    // Protobuf enum is 0-indexed; Float64=7 maps to ROS datatype 7 = FLOAT32
                    new PointField() { Name = "x", Offset = 0, Datatype = PointField.Types.DataType.Float64, Count = 1 },
                    new PointField() { Name = "y", Offset = 4, Datatype = PointField.Types.DataType.Float64, Count = 1 },
                    new PointField() { Name = "z", Offset = 8, Datatype = PointField.Types.DataType.Float64, Count = 1 }
                }
            );
            pointCloud.IsBigEndian = false;
            pointCloud.PointStep = sizeof(float) * 3;
            pointCloud.RowStep = (uint)(validCount * sizeof(float) * 3);
            pointCloud.IsDense = true;
            pointCloud.Data = Google.Protobuf.ByteString.CopyFrom(bytes);
            return pointCloud;
        }

        protected override PointCloud2StreamingRequest ComposeMessage()
        {
            PointCloud2 _pointCloud = GeneratePointCloud2(sensor.Points, sensor.Readings);
            return new PointCloud2StreamingRequest()
            {
                Data = _pointCloud,
                Address = address
            };
        }
    }
}
