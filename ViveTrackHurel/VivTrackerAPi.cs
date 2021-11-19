using System;
using System.Collections.Generic;
using System.Diagnostics;
using MathNet.Numerics.LinearAlgebra;
using System.Text;
using Valve.VR;
using System.Numerics;

namespace Hurel.PG.Positioning
{
    public class ViveTrackerApi
    {
        /// <summary>
        /// refs:
        /// https://github.com/sergiobd/ViveTrackerUtilities - use vive tracker without HMD
        /// https://vvvv.org/blog/using-htc-vive-trackers-without-headset - use vive tracker without HMD
        /// https://gist.github.com/jmptable/bf94e911f09eac5fed518964f01f80c5 - openvr code
        /// </summary>
        public ViveTrackerApi()
        {
            Initiate();
        }

        private bool isInitiated = false;
        public bool IsInitiated
        {
            get { return isInitiated; }
        }
        public void Initiate()
        {
            OpenVR.Shutdown();
            EVRInitError error = EVRInitError.Unknown;
            OpenVR.Init(ref error);
            WriteLine($"OpenVR Init ({error})");
            TrackerIndex.Clear();
            TrackingRefIndex.Clear();
            if (error != EVRInitError.None)
            {
                isInitiated = false;
                return;
            }

        
            for (uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; ++i)
            {
                var device = OpenVR.System.GetTrackedDeviceClass(i);
                if (OpenVR.System.GetTrackedDeviceClass(i) == ETrackedDeviceClass.GenericTracker)
                {
                    ETrackedDeviceProperty prop = ETrackedDeviceProperty.Prop_SerialNumber_String;
                    ETrackedPropertyError pError = ETrackedPropertyError.TrackedProp_Success;
                    StringBuilder myStringBuilder = new StringBuilder("", 100);

                    OpenVR.System.GetStringTrackedDeviceProperty(i, prop, myStringBuilder, 100, ref pError);
                    string lightHouseSerial = myStringBuilder.ToString();

                    prop = ETrackedDeviceProperty.Prop_ControllerRoleHint_Int32;
                    pError = ETrackedPropertyError.TrackedProp_Success;
                    myStringBuilder = new StringBuilder("", 100);
                    OpenVR.System.GetStringTrackedDeviceProperty(i, prop, myStringBuilder, 100, ref pError);
                    string attachedSerial = myStringBuilder.ToString();
                    WriteLine($"Index({i}), Tracker Serial({lightHouseSerial})");
                    TrackerIndex.Add(new TrackedDevice(i, lightHouseSerial));
                }
                if (OpenVR.System.GetTrackedDeviceClass(i) == ETrackedDeviceClass.TrackingReference)
                {
                    ETrackedDeviceProperty prop = ETrackedDeviceProperty.Prop_SerialNumber_String;
                    ETrackedPropertyError pError = ETrackedPropertyError.TrackedProp_Success;
                    StringBuilder myStringBuilder = new StringBuilder("", 100);

                    OpenVR.System.GetStringTrackedDeviceProperty(i, prop, myStringBuilder, 100, ref pError);
                    string lightHouseSerial = myStringBuilder.ToString();

                    WriteLine($"Index({i}), Light House Serial({lightHouseSerial})");
                    TrackingRefIndex.Add(new TrackedDevice(i, lightHouseSerial));
                }
            }



            if (TrackerIndex.Count == 0)
            {
                WriteLine($"ViveTrackerApi: Tracker is not Found");
                isInitiated = false;
            }
            else
            {
                WriteLine($"ViveTrackerApi: Tracker is found!!! {TrackerIndex.Count}.");
                isInitiated = true;
            }
        }
        
        public void GetPosesAndWrite()
        {
            if (TrackerIndex.Count == 0)
            {
                WriteLine("Tracke not found. Try re-initiate.");
                Initiate();
                return;
            }
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            int iterationCount = 1;

            WriteLine("--------------------------TRACKER------------------------------------------");
            foreach (var tracker in TrackerIndex)
            {
                Vector3 v = Vector3.Zero;
                string connected = new string("");
                string valid = new string("");
                ETrackingResult tResult = ETrackingResult.Uninitialized;
                for (int i = 0; i < iterationCount; ++i)
                {
                    OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);

                    uint index = tracker.Index;

                    HmdMatrix34_t pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    tResult = trackedDevicePose_T[index].eTrackingResult;
                    if (trackedDevicePose_T[index].bPoseIsValid)
                    {
                        valid = new string("Valid");
                    }
                    else
                    {
                        valid = new string("Not Valid");
                    }
                    if (trackedDevicePose_T[index].bDeviceIsConnected)
                    {
                        connected = new string("Connected");
                    }
                    else
                    {
                        connected = new string("Not Connected");
                    }
                    Matrix4x4 m = GetMatrix4x4FromHmdMatrix(pose);
                    v += GetPosition(m);
                }
                v /= iterationCount;
                WriteLine($"{v.X * 1000:f1}, {v.Y * 1000:f1}, {v.Z * 1000:f1} [mm], {tracker.Serial}: {tResult} ({valid}, {connected})       ");
                WriteLine("------------------------------------------------------------------------");

            }
            WriteLine("--------------------------REF----------------------------------------------");

            foreach (var tracker in TrackingRefIndex)
            {
                Vector3 v = Vector3.Zero;
                string connected = new string("");
                string valid = new string("");
                ETrackingResult tResult = ETrackingResult.Uninitialized;
                for (int i = 0; i < iterationCount; ++i)
                {
                    OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);

                    uint index = tracker.Index;
                    var pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    tResult = trackedDevicePose_T[index].eTrackingResult;
                    if (trackedDevicePose_T[index].bPoseIsValid)
                    {
                        valid = new string("Valid");
                    }
                    else
                    {
                        valid = new string("Not Valid");
                    }
                    if (trackedDevicePose_T[index].bDeviceIsConnected)
                    {
                        connected = new string("Connected");
                    }
                    else
                    {
                        connected = new string("Not Connected");
                    }
                    Matrix4x4 m = GetMatrix4x4FromHmdMatrix(pose);
                    v += GetPosition(m);
                }
                v /= iterationCount;
                WriteLine($"{v.Z * 1000:f1}, {v.Y * 1000:f1}, {v.Z * 1000:f1} [mm], {tracker.Serial}: {tResult} ({valid}, {connected})     ");
                WriteLine("------------------------------------------------------------------------");

            }

            WriteLine("--------------------------END----------------------------------------------");

            
               
            Console.SetCursorPosition(0, Console.CursorTop - 2 * TrackerIndex.Count - 2 * TrackingRefIndex.Count - 3);          
        }
        public void GetTransPosesAndWrite()
        {
            if (TrackerIndex.Count == 0)
            {
                WriteLine("Tracke not found. Try re-initiate.");
                Initiate();
                return;
            }
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            int iterationCount = 10;
            WriteLine("--------------------------TRACKER------------------------------------------");
            foreach (var tracker in TrackerIndex)
            {
                Vector3 v = Vector3.Zero;
                Vector3 r = Vector3.Zero;
                string connected = new string("");
                string valid = new string("");
                ETrackingResult tResult = ETrackingResult.Uninitialized;
                for (int i = 0; i < iterationCount; ++i)
                {
                    OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);

                    uint index = tracker.Index;

                    HmdMatrix34_t pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    tResult = trackedDevicePose_T[index].eTrackingResult;
                    if (trackedDevicePose_T[index].bPoseIsValid)
                    {
                        valid = new string("Valid");
                    }
                    else
                    {
                        valid = new string("Not Valid");
                    }
                    if (trackedDevicePose_T[index].bDeviceIsConnected)
                    {
                        connected = new string("Connected");
                    }
                    else
                    {
                        connected = new string("Not Connected");
                    }
                    Matrix4x4 m = GetMatrix4x4FromHmdMatrix(pose);
                    m = m * GlobalTransformation ;
                    r += new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21 , m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31 , MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);

                    v += GetPosition(m);
                }
                v /= iterationCount;
                r /= iterationCount;
                WriteLine($"{v.X * 1000:+0000.0;-0000.0;+0000.0}, {v.Y * 1000:+0000.0;-0000.0;+0000.0}, {v.Z * 1000:+0000.0;-0000.0;+0000.0} [mm], yaw: {r.Y:f0}, pitch: {r.Z:f0} roll: {r.X:f0} {tracker.Serial}: {tResult} ({valid}, {connected})       ");

                WriteLine("------------------------------------------------------------------------");

            }
            WriteLine("--------------------------REF----------------------------------------------");

            foreach (var tracker in TrackingRefIndex)
            {
                Vector3 v = Vector3.Zero;
                string connected = new string("");
                string valid = new string("");
                ETrackingResult tResult = ETrackingResult.Uninitialized;
                for (int i = 0; i < iterationCount; ++i)
                {
                    OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);

                    uint index = tracker.Index;
                    var pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    tResult = trackedDevicePose_T[index].eTrackingResult;
                    if (trackedDevicePose_T[index].bPoseIsValid)
                    {
                        valid = new string("Valid");
                    }
                    else
                    {
                        valid = new string("Not Valid");
                    }
                    if (trackedDevicePose_T[index].bDeviceIsConnected)
                    {
                        connected = new string("Connected");
                    }
                    else
                    {
                        connected = new string("Not Connected");
                    }
                    Matrix4x4 m = GetMatrix4x4FromHmdMatrix(pose);
                    m = m * GlobalTransformation ;
                    v += GetPosition(m);
                }
                v /= iterationCount;
                WriteLine($"{v.X * 1000:+0000.0;-0000.0;+0000.0}, {v.Y * 1000:+0000.0;-0000.0;+0000.0}, {v.Z * 1000:+0000.0;-0000.0;+0000.0} [mm], {tracker.Serial}: {tResult} ({valid}, {connected})     ");
                WriteLine("------------------------------------------------------------------------");

            }

            WriteLine("--------------------------END----------------------------------------------");

            Console.SetCursorPosition(0, Console.CursorTop - (2 * TrackerIndex.Count + 2 * TrackingRefIndex.Count + 3));

            Console.SetCursorPosition(0, Console.CursorTop - 2*  TrackerIndex.Count - 2 * TrackingRefIndex.Count - 3);
        }

        private Matrix4x4 GlobalTransformation = Matrix4x4.Identity;

        public bool SetGlobalTransformation(TrackedDevice device, Vector3 deviceTruePos)
        {
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];

            Vector3 meanPos = Vector3.Zero;
            Vector3 meanROT = Vector3.Zero;

            int iteratoinCount = 100;

            for (int i = 0; i < iteratoinCount; ++i)
            {


                OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);


                uint index = device.Index;

                bool tResult = trackedDevicePose_T[index].bPoseIsValid;
                if (tResult)
                {
                    //update Pose
                    HmdMatrix34_t pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    Matrix4x4 T0 = GetMatrix4x4FromHmdMatrix(pose);
                    Matrix4x4 T1 = Matrix4x4.CreateTranslation(deviceTruePos);
                    Matrix4x4 T0I = Matrix4x4.Identity;
                    Matrix4x4.Invert(T0, out T0I);

                    var m = T1 * T0I;
                    Vector3 pos = m.Translation;
                    Vector3 rot = new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);
                    //Console.WriteLine($"{v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");


                    meanPos += pos;
                    meanROT += rot;
                }
                else
                {
                    --iteratoinCount;
                }
               
            }
            if (iteratoinCount == 0)
            {
                WriteLine("Global Transformaiton failed");
            }
            meanPos = meanPos / (iteratoinCount);
            meanROT = meanROT / (iteratoinCount);
            {
                Vector3 v = meanPos;
                Vector3 r = meanROT;
                Matrix4x4 Md = PosAndTrueYawPitchRoll(v, r);
                GlobalTransformation = Md;

            }
            WriteLine($"Global Transformaiton success i count: {iteratoinCount}");

            return true;
        }
        public bool SetGlobalTransformation(TrackedDevice device, Matrix4x4 deviceTruePos)
        {
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];


            OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);

            Vector3 meanPos = Vector3.Zero;
            Vector3 meanROT = Vector3.Zero;

            int iteratoinCount = 300;



            for (int i = 0; i < iteratoinCount; ++i)
            {


                OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);


                uint index = device.Index;

                bool tResult = trackedDevicePose_T[index].bPoseIsValid;
                if (tResult)
                {
                    //update Pose
                    HmdMatrix34_t pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    Matrix4x4 T0 = GetMatrix4x4FromHmdMatrix(pose);
                    Matrix4x4 T1 = deviceTruePos;;
                    Matrix4x4 T0I;
                    Matrix4x4.Invert(T0, out T0I);

                    var m = T0I * T1; 
                    Vector3 pos = m.Translation;
                    Vector3 rot = new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);
                    //Console.WriteLine($"{v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");


                    meanPos += pos;
                    meanROT += rot;
                }
                else
                {
                    --iteratoinCount;
                }

            }
            if (iteratoinCount == 0)
            {
                WriteLine("Global Transformaiton failed");
                return false;
            }
            meanPos = meanPos / (iteratoinCount);
            meanROT = meanROT / (iteratoinCount);
            {
                Vector3 v = meanPos;
                Vector3 r = meanROT;
                Matrix4x4 Md = PosAndTrueYawPitchRoll(v, r);



                GlobalTransformation = Md;

            }
            WriteLine($"Global Transformaiton success i count: {iteratoinCount}");
            return true;
        }
        public bool SetGlobalTransformation(TrackedDevice deviceAtOrigin, TrackedDevice deviceAtX, TrackedDevice deviceAtY, TrackedDevice deviceAtZ)
        {

            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];


            OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);
            Vector3 meanPosOrigin = Vector3.Zero;
            Vector3 meanRotOrigin = Vector3.Zero;
            Vector3 meanPosX = Vector3.Zero;
            Vector3 meanPosY = Vector3.Zero;
            Vector3 meanPosZ = Vector3.Zero;
            Vector3 meanRotX = Vector3.Zero;
            Vector3 meanRotY = Vector3.Zero;
            Vector3 meanRotZ = Vector3.Zero;

            int iteratoinCount = 300;
            int trueIterationCount = iteratoinCount;
            for (int i = 0; i < iteratoinCount; ++i)
            {


                OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);


                uint indexOrigin = deviceAtOrigin.Index;
                uint indexX = deviceAtX.Index;
                uint indexY = deviceAtY.Index;
                uint indexZ = deviceAtZ.Index;
                bool isAllPosValid = trackedDevicePose_T[indexOrigin].bPoseIsValid && trackedDevicePose_T[indexX].bPoseIsValid && trackedDevicePose_T[indexY].bPoseIsValid && trackedDevicePose_T[indexZ].bPoseIsValid;
                if (isAllPosValid)
                {
                    //update Pose
                    HmdMatrix34_t poseOrigin = trackedDevicePose_T[indexOrigin].mDeviceToAbsoluteTracking;
                    Matrix4x4 tPoseOrigin = GetMatrix4x4FromHmdMatrix(poseOrigin);
                    HmdMatrix34_t poseX= trackedDevicePose_T[indexX].mDeviceToAbsoluteTracking;
                    HmdMatrix34_t poseY= trackedDevicePose_T[indexY].mDeviceToAbsoluteTracking;
                    HmdMatrix34_t poseZ= trackedDevicePose_T[indexZ].mDeviceToAbsoluteTracking;
                    Matrix4x4 tPoseX = GetMatrix4x4FromHmdMatrix(poseX);
                    Matrix4x4 tPoseY = GetMatrix4x4FromHmdMatrix(poseY);
                    Matrix4x4 tPoseZ = GetMatrix4x4FromHmdMatrix(poseZ);

                    meanPosOrigin += tPoseOrigin.Translation;
                    //meanRotOrigin += GetRotation(tPoseOrigin);

                    meanPosX += tPoseX.Translation;
                    meanPosY += tPoseY.Translation;
                    meanPosZ += tPoseZ.Translation;
                    //meanRotX += GetRotation(tPoseX);
                    //meanRotY += GetRotation(tPoseY);
                    //meanRotZ += GetRotation(tPoseZ);
                }
                else
                {
                    --trueIterationCount;
                }

                if (i % (iteratoinCount / 10) == 0)
                {
                    WriteLine($"Global Transformation {(double)i / iteratoinCount * 100} %");
                }
            }

            if (trueIterationCount == 0)
            {
                WriteLine("Global Transformaiton failed");
                return false;
            }
            meanPosOrigin = meanPosOrigin / (trueIterationCount);
            //meanRotOrigin = meanRotOrigin / (trueIterationCount);

            meanPosX = meanPosX / trueIterationCount;
            meanPosY = meanPosY / trueIterationCount;
            meanPosZ = meanPosZ / trueIterationCount;

            //meanRotX = meanRotX / trueIterationCount;
            //meanRotY = meanRotY / trueIterationCount;
            //meanRotZ = meanRotZ / trueIterationCount;
            
            {
                Vector3 xAxis = meanPosX - meanPosOrigin;
                Vector3 yAxis = meanPosY - meanPosOrigin;
                Vector3 zAxis = meanPosZ - meanPosOrigin;
                xAxis = xAxis / xAxis.Length();
                yAxis = yAxis / yAxis.Length();
                zAxis = zAxis / zAxis.Length();
                Matrix4x4 refDevicePos = Matrix4x4.Identity;

                refDevicePos.M11 = xAxis.X;
                refDevicePos.M12 = xAxis.Y;
                refDevicePos.M13 = xAxis.Z;
                refDevicePos.M41 = meanPosOrigin.X;
                refDevicePos.M21 = yAxis.X;
                refDevicePos.M22 = yAxis.Y;
                refDevicePos.M23 = yAxis.Z;
                refDevicePos.M42 = meanPosOrigin.Y;
                refDevicePos.M31 = zAxis.X;
                refDevicePos.M32 = zAxis.Y;
                refDevicePos.M33 = zAxis.Z;
                refDevicePos.M43 = meanPosOrigin.Z;

                Matrix4x4 refDevicePosInverse;
                Matrix4x4.Invert(refDevicePos, out refDevicePosInverse);
                GlobalTransformation = refDevicePosInverse;

            }
            WriteLine($"Global Transformaiton success i count: {trueIterationCount}");
            return true;

        }
        public Vector3 GetTransformedPose(string deviceSerial, int iterationNum = 10)
        {
            var device = TrackerIndex.Find(x => x.Serial == deviceSerial);
            if (device.Serial == deviceSerial)
            {
                return GetTransformedPose(device, iterationNum);
            }
            device = TrackingRefIndex.Find(x => x.Serial == deviceSerial);
            if (device.Serial == deviceSerial)
            {
                return GetTransformedPose(device, iterationNum);
            }

            return Vector3.Zero;
        }
        public Vector3 GetTransformedPose(TrackedDevice device, int iterationNum = 10)
        {
            return GetTransformedPoses(new List<TrackedDevice>() { device }, iterationNum)[0];
        }


        public List<Vector3> GetTransformedPoses(List<TrackedDevice> devices, int iterationNum = 10)
        {
            List<Vector3> pos = new List<Vector3>(devices.Count);
            for (int i = 0; i < devices.Count; ++i)
            {
                pos.Add(Vector3.Zero);
            }
           
            foreach (var dev in devices)
            {
                bool isDevExist = false;

                foreach (var tracker in TrackerIndex)
                {
                    if (tracker == dev)
                    {
                        isDevExist = true;
                    }
                }
                foreach (var tracker in TrackingRefIndex)
                {
                    if (tracker == dev)
                    {
                        isDevExist = true;
                    }
                }

                if (!isDevExist)
                {
                    return pos;
                }
            }
            
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];

            for (int i = 0; i < iterationNum; ++i)
            {
                OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);

                for (int j = 0; j < devices.Count; ++j)
                {
                    uint index = devices[j].Index;
                    //update Pose
                    HmdMatrix34_t pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    Matrix4x4 m = GetMatrix4x4FromHmdMatrix(pose);
                    m = m * GlobalTransformation ;
                    //pos[j] = (pos[j] * i + GetPosition(m)) / (i + 1);
                    pos[j] += GetPosition(m);
                }
            }
            for (int j = 0; j < devices.Count; ++j)
            {
                pos[j] /= iterationNum;
            }

            return pos;
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="devices"></param>
        /// <param name="iterationNum"></param>
        /// <returns>
        /// (List of devices and array of iterations)
        /// </returns>
        public (List<Vector3[]>, List<Vector3[]>) GetTransformedPosesAndRotations(List<TrackedDevice> devices, int iterationNum = 10)
        {
            List<Matrix4x4[]> mList = this.GetTransformedMatrixs(devices, iterationNum);
            int deviceCount = devices.Count;

            List<Vector3[]> returnPosList = new List<Vector3[]>();
            List<Vector3[]> returnRotList = new List<Vector3[]>();

            for (int i = 0; i < deviceCount; ++i)
            {
                returnPosList.Add(new Vector3[iterationNum]);
                returnRotList.Add(new Vector3[iterationNum]);
                for (int j = 0; j < iterationNum; ++j)
                {
                    returnPosList[i][j] = GetPosition(mList[i][j]);
                    returnPosList[i][j] = GetRotation(mList[i][j]);
                }
            }

            return (returnPosList, returnRotList);
        }

        public (List<Vector3>, List<Vector3>) GetTransformedMeanPosesAndRotations(List<TrackedDevice> devices, int iterationNum = 10)
        {
            (var posList, var rotList) = GetTransformedPosesAndRotations(devices, iterationNum);
            int deviceCount = devices.Count;
            List<Vector3> returnMeanPosList = new List<Vector3>();
            List<Vector3> returnMeanRotList = new List<Vector3>();

            for (int i = 0; i < deviceCount; ++i)
            {
                returnMeanPosList[i] = Vector3.Zero;
                returnMeanRotList[i] = Vector3.Zero;
                int trueIterationNum = iterationNum;
                for (int j = 0; j < iterationNum; ++j)
                {    
                    Vector3 pos = posList[i][j];
                    Vector3 rot = rotList[i][j];
                    
                    if (pos == Vector3.Zero && rot == Vector3.Zero)
                    {
                        trueIterationNum--;
                        continue;
                    }
                    else
                    {
                        returnMeanPosList[i] += pos;
                        returnMeanRotList[i] += rot;
                    }
                }
                returnMeanPosList[i] = returnMeanPosList[i] / trueIterationNum;
                returnMeanRotList[i] = returnMeanRotList[i] / trueIterationNum;
            }

            return (returnMeanPosList, returnMeanRotList);
        }

        public Matrix4x4[] GetTransformedMatirxOrReturnNull(string deviceSerial, int iterationNum = 10)
        {
            var device = TrackerIndex.Find(x => x.Serial == deviceSerial);
            if (device.Serial == deviceSerial)
            {
                return GetTransformedMatrix(device, iterationNum);
            }
            device = TrackingRefIndex.Find(x => x.Serial == deviceSerial);
            if (device.Serial == deviceSerial)
            {
                return GetTransformedMatrix(device, iterationNum);
            }

            return null;
        }
        public Matrix4x4[] GetTransformedMatrix(TrackedDevice device, int iterationNum = 10)
        {
            return GetTransformedMatrixs(new List<TrackedDevice>() { device }, iterationNum)[0];
        }


        public List<Matrix4x4[]> GetTransformedMatrixs(List<TrackedDevice> devices, int iterationNum = 10) // iterationNum(전역변수) = 300번
        {
            List<Matrix4x4[]> pos = new List<Matrix4x4[]>(devices.Count);
            for (int i = 0; i < devices.Count; ++i)
            {
                pos.Add(new Matrix4x4[iterationNum]); // pos[devices.Count=3][iterationNum=300 -> 60]
            }
            
            foreach (var dev in devices)
            {
                bool isDevExist = false;

                foreach (var tracker in TrackerIndex)
                {
                    if (tracker == dev)
                    {
                        isDevExist = true;
                    }
                }
                foreach (var tracker in TrackingRefIndex)
                {
                    if (tracker == dev)
                    {
                        isDevExist = true;
                    }
                }

                if (!isDevExist)
                {
                    return pos;
                }
            }

            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];

            for (int i = 0; i < iterationNum; ++i)
            {
                OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);

                for (int j = 0; j < devices.Count; ++j)
                {
                    uint index = devices[j].Index;
                    //update Pose
                    HmdMatrix34_t pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                    Matrix4x4 m = GetMatrix4x4FromHmdMatrix(pose);
                    m = m * GlobalTransformation;
                    //pos[j] = (pos[j] * i + GetPosition(m)) / (i + 1);
                    pos[j][i] = m;
                }
            }

            return pos;
        }


        private static Matrix4x4 GetMatrix4x4FromHmdMatrix(HmdMatrix34_t pose)
        {
            Matrix4x4 m = Matrix4x4.Identity;

            m.M11 = pose.m0;
            m.M21 = pose.m1;
            m.M31 = pose.m2;
            m.M41 = pose.m3;             
            m.M12 = pose.m4;
            m.M22 = pose.m5;
            m.M32 = pose.m6;
            m.M42 = pose.m7;             
            m.M13 = pose.m8;
            m.M23 = pose.m9;
            m.M33 = pose.m10;
            m.M43 = pose.m11;

            return m;
        }
        public static Vector3 GetPosition(Matrix4x4 m)
        {
            return m.Translation;
        }
        public static Vector3 GetRotation(Matrix4x4 m)
        {
            //return new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);
            float pitch = MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI;
            float yaw = MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI;
            float roll = MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI;

            //Turning rotation for vive tracker
            if ((yaw > 180f) && (yaw <= 270f))
            {
                yaw = -(-yaw + 270) +180;
            }
            else
            {
                yaw = -(yaw + 180) +180;
            }
            roll = roll;
            if (pitch < 0)
            {
                pitch = -(180 + pitch);
            }
            else
            {
                pitch = (180 - pitch);
            }
            return new Vector3(yaw, pitch, roll);
        }
        private List<TrackedDevice> TrackerIndex = new List<TrackedDevice>();
        private List<TrackedDevice> TrackingRefIndex = new List<TrackedDevice>();

        public List<TrackedDevice> GetTrackers
        {
            get { return TrackerIndex; }
        }
        public List<TrackedDevice> GetTrackingRef
        {
            get { return TrackingRefIndex; }
        }

        public static Matrix4x4 PosAndTrueYawPitchRoll(Vector3 p, Vector3 r)
        {
            // A - theta, B - Psi, C - Pi
            float alpha = r.Y * MathF.PI / 180; // alpha(Y): theta
            float cosA = MathF.Cos(alpha);      // cos(theta)
            float sinA = MathF.Sin(alpha);      // sin(theta)
            float beta = r.Z * MathF.PI / 180;  // beta(Z): psi
            float cosB = MathF.Cos(beta);       // cos(psi)
            float sinB = MathF.Sin(beta);       // sin(psi)
            float gamma = r.X * MathF.PI / 180; // gamma(X): pi
            float cosC = MathF.Cos(gamma);      // cos(pi)
            float sinC = MathF.Sin(gamma);      // sin(pi)
            Matrix4x4 m = Matrix4x4.Identity;
            m.M11 = cosA * cosB;
            m.M12 = cosA * sinB * sinC - sinA * cosC;
            m.M13 = cosA * sinB * cosC + sinA * sinC;
            m.M41 = p.X;
            m.M21 = sinA * cosB;
            m.M22 = sinA * sinB * sinC + cosA * cosC;
            m.M23 = sinA * sinB * cosC - cosA * sinC;
            m.M42 = p.Y;
            m.M31 = -sinB;
            m.M32 = cosB * sinC;
            m.M33 = cosB * cosC;
            m.M43 = p.Z;

            return m;
        }

        public record TrackedDevice(uint Index, string Serial);       

        private static void WriteLine(string msg)
        {
            try
            {
                Console.WriteLine($"ViveTrackerApi: {msg}");
            }
            catch
            {
                System.Diagnostics.Debug.WriteLine($"ViveTrackerApi: {msg}");
            }
        }
    }
}
