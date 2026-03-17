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
#if UNITY_EDITOR

using UnityEditor;
using UnityEngine;
using Marus.Actuators;
using System.Linq;
using System.Collections.Generic;
using System;

namespace Marus.Sensors
{
    /// <summary>
    /// Custom editor for Thruster component.
    /// Enables thruster configuration loading, saving and modifying.
    /// </summary>
    [CustomEditor(typeof(Thruster))]
    public class ThrusterEditor : Editor
    {
        SerializedObject _ThrusterSO;
        Thruster _myThruster;
        List<string> _thursterNames;
        List<ThrusterAsset> _thrusters;

        void OnEnable()
        {
            _ThrusterSO = new SerializedObject(target);
            _myThruster = (Thruster)target;
            _thrusters = GetAllInstances<ThrusterAsset>().ToList();
            _thursterNames = _thrusters.Select(x => x.name).ToList();
        }

        public override void OnInspectorGUI()
        {
            _ThrusterSO.Update();

            // Get current index
            int currentIndex = GetThrusterIndex(_myThruster);

            // Dropdown
            int newIndex = EditorGUILayout.Popup(
                "Thruster",
                currentIndex,
                _thursterNames.ToArray()
            );

            // Only assign if user changed it
            if (newIndex != currentIndex)
            {
                _myThruster.ThrusterAsset = _thrusters[newIndex];
                EditorUtility.SetDirty(_myThruster);
            }

            // Show curve (READ-ONLY preview)
            if (_myThruster.ThrusterAsset != null)
            {
                EditorGUILayout.CurveField(
                    "Curve (read-only)",
                    _myThruster.ThrusterAsset.curve
                );
            }

            // Runtime info (unchanged)
            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.LabelField("Info");
            EditorGUILayout.FloatField("Last force requested", _myThruster.LastForceRequest);
            EditorGUILayout.FloatField("Time since force request", _myThruster.TimeSinceForceRequest);
            EditorGUILayout.FloatField("Normalized input", _myThruster.NormalizedInput);
            EditorGUI.EndDisabledGroup();

            _ThrusterSO.ApplyModifiedProperties();
        }

        /// <summary>
        /// Getting all instances of ThrusterAsset in the project
        /// </summary>
        private static T[] GetAllInstances<T>() where T : ScriptableObject
        {
            string[] guids = AssetDatabase.FindAssets("t:"+ typeof(T).Name);
            T[] a = new T[guids.Length];
            for(int i =0;i<guids.Length;i++)
            {
                string path = AssetDatabase.GUIDToAssetPath(guids[i]);
                a[i] = AssetDatabase.LoadAssetAtPath<T>(path);
            }
            return a;
        }

        private int GetThrusterIndex(Thruster thruster)
        {
            int index = _thrusters.IndexOf(thruster.ThrusterAsset);
            if(index == -1) index = 0;
            return index;
        }
    }
}

#endif
