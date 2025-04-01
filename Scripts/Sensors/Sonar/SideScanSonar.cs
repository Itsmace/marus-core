// Copyright 2025 Laboratory for Underwater Systems and Technologies (LABUST)
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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using Marus.Networking;
using Marus.Sensors;
using Marus.Sensors.Core;
using Marus.Visualization;
using Unity.Collections;
using System.Threading;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.UI;
using Sensorstreaming;
using Marus.CustomInspector;
using Marus.ObjectAnnotation;

namespace Marus.Sensors
{

    /// <summary>
    /// Implementation of Side Scan Sonar (SSS) using raytracing model
    /// Generates waterfall image of two side scan swats
    /// Implemented using IJobParallelFor on CPU
    /// Can drop performance
    /// </summary>
    public class SideScanSonar : SensorBase
    {
        /// <summary>
        /// Number of acoustic rays per swath width
        /// </summary>
        public int SwathRes = 256;

        /// <summary>
        /// Number of rays to model a single beam azimuth (longitudinal beam width)
        /// </summary>
        public int BeamRes = 16;

        /// <summary>
        /// Sonar output gain
        /// </summary>
        public float SonarGain = 50;

        /// <summary>
        /// Maximum sonar range in meters
        /// </summary>
        public float SonarRange = 300;

        /// <summary>
        /// Starting sonar SonarRange in meters
        /// </summary>
        public float MinDistance = 0.6F;

        /// <summary>
        /// Acoustic shadow zone angle, measured from vertical towards each side (assumingly symetrical)
        /// </summary> 
        public float NadirAngle = 20;

        /// <summary>
        /// Single side vertical SSS field of view, in degrees
        /// </summary> 
        public float VerticalBeamwidth = 50;

        /// <summary>
        /// Horizontal beam width view in degrees
        /// </summary>
        public float HorizontalBeamwidth = 0.7F;

        /// <summary>
        /// Vertical resolution (or size) of the SSS image.
        /// Effectively sets the time window that is displayed, or number of rows containing previous readings.
        /// </summary>
        public int imageHeight = 512;

        /// <summary>
        /// Equiangular - rays are distributed uniformly
        /// Equidistant - rays are distributed equidistantly on a horizontal plane
        /// </summary>
        public enum RayDistribution { Equiangular, Equidistant }
        public RayDistribution sonarRayDistribution;

        /// <summary>
        /// Saved configuration presets for replicating common sonar models 
        /// Custom - configuration settings set in the inspector editor
        /// </summary>
        public enum SonarConfiguration { Custom, Klein, Norbit }
        public SonarConfiguration sonarConfig;

        /// <summary>
        /// Optional saving of generated polar and cartesian images. 
        /// If enabled, images saved in project_folder/SaveImages/ or the set image save path
        /// </summary>
        public bool SaveImages = false;

        [ConditionalHideInInspector("SaveImages", false)]
        public string ImageSavePath;

        /// <summary>
        /// Optional grid overlay.
        /// </summary>
        public bool Grid = false;

        /// <summary>
        /// Sonar noise parameters.
        /// </summary>
        public bool AddNoise = false;
        public float NoiseLevel = 0.1f; // General noise intensity
        public float SpeckleLevel = 0.05f; // Speckle noise intensity
        public float RayleighScale = 1.0f; // Rayleigh noise scale
        private System.Random systemRandom = new System.Random();

        public bool AddColormap = false;
        [ConditionalHideInInspector("AddColormap", false)]
        public Colormap.Palette SelectedPalette = Colormap.Palette.Gemini;
        public enum ColormapType
        {
            Gemini, Jet, Parula, Turbo, HSV, Hot
        }

        private static Dictionary<ColormapType, Color[]> colormapLUT = new Dictionary<ColormapType, Color[]>();


        /// <summary>
        /// Number of raycast rays simulating a single acoustic ray
        /// </summary>
        int NumRaysPerAccusticRay = 1;

        public bool DisplayImages = false;

        /// <summary>
        /// Display arrays for canvas display
        /// </summary>
        [ConditionalHideInInspector("DisplayImages", false)]
        public RawImage WaterfallDisplay, ClassInstanceDisplay;

        /// <summary>
        /// Cartesian and polar texture2D arrays
        /// </summary>
        [HideInInspector]
        public Texture2D SonarImageRow, SideScanImage, ClassInstanceImage;

        public NativeArray<SonarReading> sonarData;
        int imageCount = 1;
        double thetha;
        double r;
        const float WATER_LEVEL = 0;
        float altitude, pitch;
        PointCloudManager _pointCloudManager;
        RaycastJobHelper<SonarReading> _raycastHelper;
        Coroutine _coroutine;
        Vector3 sonarPosition;
        NativeArray<Vector3> directionsLocal;
        private SonarObjectDetectionSaver _saver;
        private bool saverExists;

        void Start()
        {
            if (String.IsNullOrEmpty(ImageSavePath))
            {
                ImageSavePath = Application.dataPath + "/../SaveImages/";
            }
            Directory.CreateDirectory(ImageSavePath);

            int totalRays = 2 * SwathRes * BeamRes * NumRaysPerAccusticRay; // Two sets, one for each side of the swath

            _saver = GetComponent<SonarObjectDetectionSaver>();
            saverExists = _saver is not null && _saver.isActiveAndEnabled == true;

            sonarData = new NativeArray<SonarReading>(totalRays, Allocator.Persistent);

            SonarImageRow = new Texture2D(2 * SwathRes, 1, TextureFormat.RGB24, false); // Array for one row of sonar data, composed of a single beam reading from left and right side
            SideScanImage = new Texture2D(2 * SwathRes, imageHeight, TextureFormat.RGB24, false); // Array for the waterfalled SSS image, composed of left and right swath echo
            ClassInstanceImage = new Texture2D(2 * SwathRes, imageHeight, TextureFormat.RGB24, false); // Array for the annotation data of automatic data tool

            //sLoadSonarConfigs(); // Load sonar parameters from the preset sonar model list
            InitializeRayArray(); // Calculate ray angles for the whole swath

            // Initiallize the sonar image to black pixels
            Color black = new Color(0, 0, 0, 1);

            // Get image dimensions
            int width = SideScanImage.width;
            int height = SideScanImage.height;

            // Set all pixels to black
            Color[] blackPixels = Enumerable.Repeat(black, width * height).ToArray();
            SideScanImage.SetPixels(blackPixels);
            SideScanImage.Apply(); // Apply changes to the texture

            _raycastHelper = new RaycastJobHelper<SonarReading>(gameObject, directionsLocal, OnSonarHit, OnFinish, SonarRange);
            _raycastHelper.SampleFrequency = SampleFrequency;

            _coroutine = StartCoroutine(_raycastHelper.RaycastInLoop());
        }

        protected override void SampleSensor()
        {
            sonarPosition = transform.position;
        }

        private void OnFinish(NativeArray<Vector3> points, NativeArray<SonarReading> sonarReadings)
        {
            // DEBUG
            Debug.Log($"sonarReadings Length: {sonarReadings.Length}, sonarData Length: {sonarData.Length}");

            sonarReadings.CopyTo(sonarData);
            ComposeImageRow(sonarReadings);
            WaterfallImage(SonarImageRow, SideScanImage);

            hasData = true;
        }
        /// <summary>
        /// Function for converting hit distance to Y coordinate of the cartesian projection. 
        /// </summary>
        private int DistanceToImageY(float distance)
        {
            if (distance < SonarRange && distance >= MinDistance)
            {
                int y = (int)Math.Floor(((distance - MinDistance) / (SonarRange - MinDistance)) * imageHeight);
                return y;
            }
            else
            {
                return 0;
            }
        }
        /// <summary>
        /// Initializes raycast ray directions based on the selection and returns ray directions. 
        /// Equidistant distribution projects equidistant points on a horizontal plane (useful for bathymetric or down looking sonar).
        /// Depends on sonar pitch angle and altitude from the bottom plane.
        /// Equiangular distribution sets vertical angles equally.
        /// </summary>
        public void InitializeRayArray()
        {
            if (sonarRayDistribution == RayDistribution.Equidistant)
            {
                //get the sonar pitch angle, which in case of SSS is the angle of the middle of the swath towards each side
                //pitch = NadirAngle + VerticalBeamwidth/2;
                pitch = 90; // Single swath to sample both sonar sides - pitch angle looking directly at the nadir centre, downwards.

                //sample depth under the sonar for equidistant ray projection
                RaycastHit hit;
                if (UnityEngine.Physics.Raycast(transform.position, Vector3.down, out hit, Mathf.Infinity))
                {
                    altitude = hit.distance;
                }

                directionsLocal = RaycastJobHelper.EquidistantRays(2 * SwathRes, BeamRes, ((VerticalBeamwidth + NadirAngle) * 2), HorizontalBeamwidth, altitude, pitch);
            }
            else if (sonarRayDistribution == RayDistribution.Equiangular)
            {
                var _rayAngles = RaycastJobHelper.InitUniformRays(2 * SwathRes, BeamRes, ((VerticalBeamwidth + NadirAngle) * 2), HorizontalBeamwidth);
                directionsLocal = RaycastJobHelper.CalculateRayDirections(_rayAngles);
                _rayAngles.Dispose();
            }
            else
            {
                gameObject.active = false;
                return;
            }
        }

        /// <summary>
        /// Method for setting custom sonar configurations selected from the dropdown sonar list
        /// </summary>
        /*public void SetSonarConfig()
        {

        } */

        /*public void LoadSonarConfigs()
        {
            var jsonText = File.ReadAllText("SonarPresets.json");
            SonarConfigs = JsonConvert.DeserializeObject<List<SonarConfigs>>(jsonText);
            sonarObj = target as RaycastSonar;
            sonarObj.Configs = SonarConfigs;
            _choices = new string[SonarConfigs.Count];
            var i = 0;
            foreach(var cfg in SonarConfigs)
            {
                _choices[i++] = cfg.Name;
            }
            _configName = _choices[sonarObj.ConfigIndex];
        } */

        void OnDestroy()
        {
            try
            {
                _raycastHelper.Dispose();
                sonarData.Dispose();
                directionsLocal.Dispose();
            }
            catch { }

        }

        public SonarReading OnSonarHit(RaycastHit hit, Vector3 direction, int i)
        {
            var distance = hit.distance;
            var sonarReading = new SonarReading();
            (int, int) value;
            float intensity = 0;

            if (saverExists)
            {
                if (_saver.objectClassesAndInstances.TryGetValue(hit.colliderInstanceID, out value))
                {
                    sonarReading.ClassId = value.Item1;
                    sonarReading.InstanceId = value.Item2;
                }
            }

            //in case of out of SonarRange rays add only thermal and speckle noise
            if (distance < MinDistance || hit.point.y > WATER_LEVEL || hit.point == Vector3.zero)
            {
                sonarReading.Valid = false;
                sonarReading.Intensity = 0;
            }
            else
            {
                sonarReading.Valid = true;
                sonarReading.Distance = hit.distance;

                double alpha = Math.PI - (Math.Acos(Vector3.Dot(direction, hit.normal)));
                intensity = (SonarGain / 10) * (float)(Math.Cos(alpha) * Math.Cos(alpha));

                sonarReading.Intensity = intensity;
            }
            return sonarReading;
        }

        /// <summary>
        /// Function for composing a row of sonar data, averaged over the beam width. 
        /// </summary>
        private void ComposeImageRow(NativeArray<SonarReading> reading)
        {
            Color pixel;
            pixel = new Color(0, 0, 0, 1);
            float intensity = 0;
            for (var x = 0; x < (2 * SwathRes); x++)
            {
                for (var y = 0; y < BeamRes; y++)
                {
                    if (reading[x * BeamRes + y].Valid)
                    {
                        intensity += reading[x * BeamRes + y].Intensity;
                    }
                }
                intensity /= (float)BeamRes; // Averaging intensities over the beam width
                pixel = new Color(intensity, intensity, intensity, 1);
                SonarImageRow.SetPixel(x, 0, pixel);
                intensity = 0;
            }

            SonarImageRow.Apply();
            reading.Dispose();
        }

        /// <summary>
        /// Function for updating the sonar image in a waterfall fashion
        /// Width inherited from width sonar resolution, height can be set indipendently. .png image saving optional.
        /// </summary>
        private void WaterfallImage(Texture2D sonarRow, Texture2D oldImage)
        {

            //copy previous frame
            Color[] oldPixels = oldImage.GetPixels();
            Color[] newPixels = new Color[oldPixels.Length];
            
             // Fill the new data with black pixels initially
            for (int i = 0; i < newPixels.Length; i++)
            {
                newPixels[i] = new Color(0, 0, 0, 1);
            }

            Texture2D previousImage = new Texture2D(oldImage.width, oldImage.height, TextureFormat.RGBA32, false);
            previousImage.SetPixels(oldPixels);
            previousImage.Apply();

            //Color annPixel; // pixel for storing annotation data, in R, G and B
            float[] yIntensity = new float[imageHeight];
            int[] currentClassId = new int[imageHeight];
            int[] currentInstanceId = new int[imageHeight];

            // Shift all existing rows downwards
            // Copy previous image data and shift it down by one row
            for (int y = 1; y < oldImage.height; y++)
            {
                for (int x = 0; x < (oldImage.width); x++)
                {
                    newPixels[y * oldImage.width + x] = oldPixels[(y - 1) * oldImage.width + x];
                }
            }

            // Insert the new row at the top
            Color[] sonarRowPixels = sonarRow.GetPixels();
            for (int x = 0; x < (2 * SwathRes); x++)
            {
                newPixels[x] = sonarRowPixels[x]; // Copying new row pixels to the top row
            }

            // Apply the updated pixels back to the texture
            SideScanImage.SetPixels(newPixels);
            SideScanImage.Apply();

            // Copy updated texture back to the previousImage
            Graphics.CopyTexture(SideScanImage, previousImage);

            SideScanImage.Apply();

            if (WaterfallDisplay is not null)
            {
                WaterfallDisplay.texture = SideScanImage;
            }

            if (SaveImages)
            {
                byte[] bytes = SideScanImage.EncodeToPNG();
                File.WriteAllBytes(Path.Combine(ImageSavePath, "SideScanImage" + imageCount + ".png"), bytes);
            }

        }

        public Texture2D AddGridAndLabels(Texture2D image)
        {
            //add horizontal grid
            int r = 0;
            for (int i = 1; i < SonarRange / 10; i++)
            {
                r = DistanceToImageY(i * 10);
                image = DrawLine(image, 0, SwathRes, r, r);
            }

            r = DistanceToImageY(SonarRange);
            image = DrawLine(image, 0, SwathRes, r, r);

            //add vertical grid
            for (int i = 0; i < 5; i++)
            {
                image = DrawLine(image, i * SwathRes / 4, i * SwathRes / 4, 0, imageHeight);
            }

            return image;
        }

        public Texture2D DrawLine(Texture2D baseImage, int startX, int endX, int startY, int endY)
        {
            UnityEngine.Color color = new UnityEngine.Color(1, 1, 1, 1);

            for (int x = startX; x <= endX; x++)
            {
                for (int y = startY; y <= endY; y++)
                {
                    baseImage.SetPixel(x, y, color);
                }
            }
            return baseImage;
        }

        private float AddGaussianNoise(float intensity)
        {
            float noise = RandomGaussian() * NoiseLevel;
            return Mathf.Clamp(intensity + noise, 0.0f, 1.0f);
        }

        private float RandomGaussian()
        {
            float u1 = 1.0f - (float)systemRandom.NextDouble();
            float u2 = 1.0f - (float)systemRandom.NextDouble();
            return Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        }

        private float AddSpeckleNoise(float intensity)
        {
            float speckle = (1 + RandomGaussian() * SpeckleLevel);
            return Mathf.Clamp(intensity * speckle, 0.0f, 1.0f);
        }

        private float AddRayleighNoise(float intensity, float distance)
        {
            float rayleighNoise = DistanceRayleigh(RayleighScale, distance);
            return Mathf.Clamp(intensity + rayleighNoise, 0.0f, 1.0f);
        }

        private float DistanceRayleigh(float sigma, float r)
        {
            float p_r = (r / sigma * sigma) * Mathf.Exp(-((r * r) / (2 * sigma * sigma)));
            return p_r;
        }
        private float RandomRayleigh(float scale)
        {
            float u = (float)systemRandom.NextDouble();
            return scale * Mathf.Sqrt(-2.0f * Mathf.Log(u));
        }

        public Color ApplyColorMapping(float intensity)
        {
            if (AddColormap)
            {
                return Colormap.GetColor(intensity, SelectedPalette);
            }
            else
            {
                return new Color(intensity, intensity, intensity, 1f); // Grayscale
            }
        }

    }
}
