using UnityEngine;
using Google.Protobuf;
using Sensorstreaming;
using Marus.Networking;
using Marus.Core;

using static Sensorstreaming.SensorStreaming;

namespace Marus.Sensors
{
    [RequireComponent(typeof(DepthCameraSensor))]
    public class DepthCameraSensorROS : SensorStreamer<SensorStreamingClient, CameraStreamingRequest>
    {
        DepthCameraSensor sensor;

        new void Start()
        {
            sensor = GetComponent<DepthCameraSensor>();
            StreamSensor(sensor,
                streamingClient.StreamCameraSensor);
            base.Start();
        }

        protected override CameraStreamingRequest ComposeMessage()
        {
            return new CameraStreamingRequest
            {
                Image = new Sensor.Image
                {
                    Header = new Std.Header
                    {
                        Timestamp = Time.time,
                        FrameId = sensor.frameId
                    },
                    Data = ByteString.CopyFrom(sensor.Data),
                    Height = (uint)sensor.ImageHeight,
                    Width = (uint)sensor.ImageWidth
                },
                Address = address,
            };
        }
    }
}

