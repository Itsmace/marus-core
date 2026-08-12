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

using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

namespace Marus.Core
{

    public static class TfExtensions
    {
        public static Vector3 Map2Unity(this Vector3 vector3)
        {
            return new Vector3(vector3.x, vector3.z, vector3.y);
        }
        
        /// Use this conversion when translating global coordinates from Unity to ROS Map (ENU) frame.
        public static Vector3 Unity2Map(this Vector3 vector3)
        {
            return new Vector3(vector3.x, vector3.z, vector3.y);
        }
        
        public static Quaternion Map2Unity(this Quaternion quaternion)
        {
            return new Quaternion(-quaternion.x, -quaternion.z, -quaternion.y, quaternion.w);
        } 

        /// Use this conversion when translating global rotation from Unity to ROS Map (ENU) frame.
        public static Quaternion Unity2Map(this Quaternion quaternion)
        {
            return new Quaternion(-quaternion.x, -quaternion.z, -quaternion.y, quaternion.w);
        }
        
        /// Use this conversion when translating local coordinates from Unity to ROS Forward-Left-Up body frames.
        ///
        /// NOTE (SC2 fork): upstream Marus assumed the vehicle model's local Z is forward
        /// (Unity's default convention). The SC2 ROV is modelled X-forward / Y-up, so the
        /// original permutation (z, -x, y) picked the wrong axis as ROS "forward" and put a
        /// systematic 90 deg yaw error on every sensor's data. With X-forward / Y-up, and the
        /// determinant fixed at -1 by the Unity(left-handed) -> ROS(right-handed) flip, the
        /// mapping is forced to: ROS x = unity x (forward), y = unity z (left), z = unity y (up).
        /// That makes it identical to Unity2Map, which is expected: the ROV's local axes are
        /// laid out the same way as the world axes (X forward/East, Y up, Z left/North).
        public static Vector3 Unity2Body(this Vector3 vector3)
        {
            return new Vector3(vector3.x, vector3.z, vector3.y);
        }

        /// Use this conversion when translating local rotations from Unity to ROS Forward-Left-Up body frames.
        /// Same permutation as the Vector3 overload, negated for the handedness flip (see note above).
        public static Quaternion Unity2Body(this Quaternion quaternion)
        {
            return new Quaternion(-quaternion.x, -quaternion.z, -quaternion.y, quaternion.w);
        }
    }

}