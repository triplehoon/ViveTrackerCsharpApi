using System;
using System.Collections.Generic;
using System.Diagnostics;
using MathNet.Numerics.LinearAlgebra;
using System.Text;
using Valve.VR;


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

            int rightDown = 15;
            Console.SetCursorPosition(0, Console.CursorTop - 2*  TrackerIndex.Count - 2 * TrackingRefIndex.Count - 3);
        }

        private Matrix<float> GlobalTransformation = Matrix<float>.Build.DenseIdentity(4);

        public bool SetGlobalTransformation(TrackedDevice device, Vector3 deviceTruePos)
        {
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];


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
                GlobalTransformation = T1 * T0I;

                return true;
            }
            else
            {
                return false;
            }
        }
        public bool SetGlobalTransformation(TrackedDevice device, Matrix4x4 deviceTruePos)
        {
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];


            OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);


            uint index = device.Index;

            bool tResult = trackedDevicePose_T[index].bPoseIsValid;

            for (int i = 0; i < 20; ++i)
            {
                if (tResult)
                {
                    break;
                }
                tResult = trackedDevicePose_T[index].bPoseIsValid;
                WriteLine($"Try SetGlobalTransformation valid ({i})");
            }
            if (tResult)
            {
                //update Pose
                HmdMatrix34_t pose = trackedDevicePose_T[index].mDeviceToAbsoluteTracking;
                Matrix4x4 T0 = GetMatrix4x4FromHmdMatrix(pose);
                Matrix4x4 T1 = deviceTruePos;
                Matrix4x4 T0I;
                Matrix4x4 T1I;

                if (!Matrix4x4.Invert(T0, out T0I))
                {
                    return false;
                }
                if (!Matrix4x4.Invert(T1, out T1I))
                {
                    return false;
                }
                GlobalTransformation = T0I * T1I;
                WriteLine("GlobalTransformation has been set");
                return true;
            }
            else
            {

                WriteLine("GlobalTransformation fail");
                return false;
            }
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

        public Matrix4x4[] GetTransformedMatirxOrReturnNull(string deviceSerial, int iterationNum = 10)
        {
            var device = TrackerIndex.Find(x => x.Serial == deviceSerial);
            if (device.Serial == deviceSerial)
            {
                return GetTransformedMatirx(device, iterationNum);
            }
            device = TrackingRefIndex.Find(x => x.Serial == deviceSerial);
            if (device.Serial == deviceSerial)
            {
                return GetTransformedMatirx(device, iterationNum);
            }

            return null;
        }
        public Matrix4x4[] GetTransformedMatirx(TrackedDevice device, int iterationNum = 10)
        {
            return GetTransformedMatirx(new List<TrackedDevice>() { device }, iterationNum)[0];
        }
        public List<Matrix4x4[]> GetTransformedMatirx(List<TrackedDevice> devices, int iterationNum = 10)
        {
            List<Matrix4x4[]> pos = new List<Matrix4x4[]>(devices.Count);
            for (int i = 0; i < devices.Count; ++i)
            {
                pos.Add(new Matrix4x4[iterationNum]);
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

        private static Matrix<flo  GetMatrix4x4FromHmdMatrix(HmdMatrix34_t pose)
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
