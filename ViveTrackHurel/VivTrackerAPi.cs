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
        /// http://planning.cs.uiuc.edu/node102.html - transformation system
        /// Using Column major (important)
        /// DO NOT USE MATRIX4x4 FUNCTION ONLY USE VIVETRACKERAPI TO TRANSFORM ETC
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
           
        private Matrix4x4 GlobalTransformationInverse = Matrix4x4.Identity;

        public bool SetGlobalTransformation(TrackedDevice deviceAtOrigin, TrackedDevice deviceAtX, TrackedDevice deviceAtY, TrackedDevice deviceAtZ)
        {
            //origin = +8.8
            TrackedDevicePose_t[] trackedDevicePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            TrackedDevicePose_t[] trackedGamePose_T = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];


            OpenVR.Compositor.WaitGetPoses(trackedDevicePose_T, trackedGamePose_T);
            VivePose meanPosOrigin = VivePose.Zero();
            ViveYawPitchRoll meanRotOrigin = ViveYawPitchRoll.Zero();
            VivePose meanPosX = VivePose.Zero();
            VivePose meanPosY = VivePose.Zero();
            VivePose meanPosZ = VivePose.Zero();
            ViveYawPitchRoll meanRotX = ViveYawPitchRoll.Zero();
            ViveYawPitchRoll meanRotY = ViveYawPitchRoll.Zero();
            ViveYawPitchRoll meanRotZ = ViveYawPitchRoll.Zero();

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

                    meanPosOrigin += GetPosition(tPoseOrigin);
                    //meanRotOrigin += GetYawPitchRoll(tPoseOrigin);

                    meanPosX += GetPosition(tPoseX);
                    meanPosY += GetPosition(tPoseY);
                    meanPosZ += GetPosition(tPoseZ);
                    //meanRotX += GetYawPitchRoll(tPoseX);
                    //meanRotY += GetYawPitchRoll(tPoseY);
                    //meanRotZ += GetYawPitchRoll(tPoseZ);
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
                VivePose xAxis = meanPosX - meanPosOrigin;
                VivePose yAxis = meanPosY - meanPosOrigin;
                VivePose zAxis = meanPosZ - meanPosOrigin;
                xAxis = xAxis / xAxis.Length();
                yAxis = yAxis / yAxis.Length();
                zAxis = zAxis / zAxis.Length();
                Matrix4x4 refDevicePos = Matrix4x4.Identity;

                refDevicePos.M11 = xAxis.X;
                refDevicePos.M12 = xAxis.Y;
                refDevicePos.M13 = xAxis.Z;
                refDevicePos.M14 = meanPosOrigin.X;
                refDevicePos.M21 = yAxis.X;
                refDevicePos.M22 = yAxis.Y;
                refDevicePos.M23 = yAxis.Z;
                refDevicePos.M24 = meanPosOrigin.Y;
                refDevicePos.M31 = zAxis.X;
                refDevicePos.M32 = zAxis.Y;
                refDevicePos.M33 = zAxis.Z;
                refDevicePos.M34 = meanPosOrigin.Z + 8.8f / 1000;

                Matrix4x4 refDevicePosInverse;
                Matrix4x4.Invert(refDevicePos, out refDevicePosInverse);
                GlobalTransformationInverse = refDevicePosInverse;

            }
            WriteLine($"Global Transformaiton success i count: {trueIterationCount}");
            return true;

        }
        public VivePose GetTransformedPose(string deviceSerial, int iterationNum = 10)
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

            return VivePose.Zero();
        }
        public VivePose GetTransformedPose(TrackedDevice device, int iterationNum = 10)
        {
            return GetTransformedPoses(new List<TrackedDevice>() { device }, iterationNum)[0];
        }
        public List<VivePose> GetTransformedPoses(List<TrackedDevice> devices, int iterationNum = 10)
        {
            List<VivePose> pos = new List<VivePose>(devices.Count);
            for (int i = 0; i < devices.Count; ++i)
            {
                pos.Add(VivePose.Zero());
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
                    m = m * GlobalTransformationInverse;
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
        public (List<VivePose[]>, List<ViveYawPitchRoll[]>) GetTransformedPosesAndRotations(List<TrackedDevice> devices, int iterationNum = 10)
        {
            List<Matrix4x4[]> mList = this.GetTransformedMatrixs(devices, iterationNum);
            int deviceCount = devices.Count;

            List<VivePose[]> returnPosList = new List<VivePose[]>();
            List<ViveYawPitchRoll[]> returnRotList = new List<ViveYawPitchRoll[]>();

            for (int i = 0; i < deviceCount; ++i)
            {
                returnPosList.Add(new VivePose[iterationNum]);
                returnRotList.Add(new ViveYawPitchRoll[iterationNum]);
                for (int j = 0; j < iterationNum; ++j)
                {
                    returnPosList[i][j] = GetPosition(mList[i][j]);
                    returnRotList[i][j] = GetYawPitchRoll(mList[i][j]);
                }
            }

            return (returnPosList, returnRotList);
        }
        public (List<VivePose>, List<ViveYawPitchRoll>) GetTransformedMeanPosesAndRotations(List<TrackedDevice> devices, int iterationNum = 10)
        {
            (var posList, var rotList) = GetTransformedPosesAndRotations(devices, iterationNum);
            int deviceCount = devices.Count;
            List<VivePose> returnMeanPosList = new List<VivePose>();
            List<ViveYawPitchRoll> returnMeanRotList = new List<ViveYawPitchRoll>();

            for (int i = 0; i < deviceCount; ++i)
            {
                returnMeanPosList[i] = new VivePose(0,0,0);
                returnMeanRotList[i] = new ViveYawPitchRoll(0,0,0);
                int trueIterationNum = iterationNum;
                for (int j = 0; j < iterationNum; ++j)
                {
                    VivePose pos = posList[i][j];
                    ViveYawPitchRoll rot = rotList[i][j];
                    
                    if (pos == VivePose.Zero() && rot == ViveYawPitchRoll.Zero())
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
                    m = m * GlobalTransformationInverse;
                    //pos[j] = (pos[j] * i + GetPosition(m)) / (i + 1);
                    pos[j][i] = m;
                }
            }

            return pos;
        }

        /// <summary>
        /// Matrix4x4 is column major transformation matrix.
        /// m11, m12, m13, m14
        /// m21, m22, m23, m24
        /// m31, m32, m33, m34
        /// m41, m42, m43, m44
        /// Transformation = (m14, m24, m34)
        /// newT = translation * rotation * scale * currentT
        /// Vivetracker is right-hand system
        /// Angle system trun z (roll) -> x yaw -> y pitch
        /// Ref:http://planning.cs.uiuc.edu/node102.html
        /// </summary>
        /// <param name="pose">HmdMatrix</param>
        /// <returns></returns>
        private static Matrix4x4 GetMatrix4x4FromHmdMatrix(HmdMatrix34_t pose)
        {
            Matrix4x4 m = Matrix4x4.Identity;

            m.M11 = pose.m0;
            m.M12 = pose.m1;
            m.M13 = pose.m2;
            m.M14 = pose.m3;             
            m.M21 = pose.m4;
            m.M22 = pose.m5;
            m.M23 = pose.m6;
            m.M24 = pose.m7;             
            m.M31 = pose.m8;
            m.M32 = pose.m9;
            m.M33 = pose.m10;
            m.M34 = pose.m11;
            return m;
        }
        public static VivePose GetPosition(Matrix4x4 m)
        {
            return new VivePose(m.M14, m.M24, m.M34);
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




        //public static Matrix4x4 GetMatrixFromPosAndTrueYawPitchRoll(Vector3 p, Vector3 yawPitchRoll)
        //{
        //    // A - theta, B - Psi, C - Pi
        //    float alpha = yawPitchRoll.Y * MathF.PI / 180; // alpha(Y): theta Pitch
        //    float cosA = MathF.Cos(alpha);      // cos(theta)
        //    float sinA = MathF.Sin(alpha);      // sin(theta)
        //    float beta = yawPitchRoll.Z * MathF.PI / 180;  // beta(Z): psi 
        //    float cosB = MathF.Cos(beta);       // cos(psi)
        //    float sinB = MathF.Sin(beta);       // sin(psi)
        //    float gamma = yawPitchRoll.X * MathF.PI / 180; // gamma(X): pi
        //    float cosC = MathF.Cos(gamma);      // cos(pi)
        //    float sinC = MathF.Sin(gamma);      // sin(pi)
        //    Matrix4x4 m = Matrix4x4.Identity;
        //    m.M11 = cosA * cosB;
        //    m.M12 = cosA * sinB * sinC - sinA * cosC;
        //    m.M13 = cosA * sinB * cosC + sinA * sinC;
        //    m.M41 = p.X;
        //    m.M21 = sinA * cosB;
        //    m.M22 = sinA * sinB * sinC + cosA * cosC;
        //    m.M23 = sinA * sinB * cosC - cosA * sinC;
        //    m.M42 = p.Y;
        //    m.M31 = -sinB;
        //    m.M32 = cosB * sinC;
        //    m.M33 = cosB * cosC;
        //    m.M43 = p.Z;
        //    return m;
        //}


        /// <summary>
        /// Matrix4x4 is column major transformation matrix.
        /// m11, m12, m13, m14
        /// m21, m22, m23, m24
        /// m31, m32, m33, m34
        /// m41, m42, m43, m44
        /// Transformation = (m14, m24, m34)
        /// newT = translation * rotation * scale * currentT
        /// Vivetracker is right-hand system
        /// Angle system trun x (roll) -> y pitch -> z yaw
        /// Ref:http://planning.cs.uiuc.edu/node102.html
        /// </summary>
        /// <param name="p"></param>
        /// <param name="yawPitchRoll"></param>
        /// <returns></returns>
        public static Matrix4x4 GetMatrixFromPosAndTrueYawPitchRoll(VivePose p, ViveYawPitchRoll yawPitchRoll)
        {
            // (A,Vector3[0]) - Yaw, (B,Vector3[1]) - Pitch, (C,Vector3[2]) - roll
            float alpha = yawPitchRoll.Yaw * MathF.PI / 180;
            float cosA = MathF.Cos(alpha);      
            float sinA = MathF.Sin(alpha);      
            float beta = yawPitchRoll.Pitch * MathF.PI / 180;
            float cosB = MathF.Cos(beta);       
            float sinB = MathF.Sin(beta);       
            float gamma = yawPitchRoll.Roll * MathF.PI / 180; 
            float cosC = MathF.Cos(gamma);
            float sinC = MathF.Sin(gamma);
            Matrix4x4 m = Matrix4x4.Identity;
            m.M11 = cosA * cosB;
            m.M12 = cosA * sinB * sinC - sinA * cosC;
            m.M13 = cosA * sinB * cosC + sinA * sinC;
            m.M14 = p.X;
            m.M21 = sinA * cosB;
            m.M22 = sinA * sinB * sinC + cosA * cosC;
            m.M23 = sinA * sinB * cosC - cosA * sinC;
            m.M24 = p.Y;
            m.M31 = -sinB;
            m.M32 = cosB * sinC;
            m.M33 = cosB * cosC;
            m.M34 = p.Z;

            return m;
        }


        /// <summary>
        /// Matrix4x4 is column major transformation matrix.
        /// m11, m12, m13, m14
        /// m21, m22, m23, m24
        /// m31, m32, m33, m34
        /// m41, m42, m43, m44
        /// Transformation = (m14, m24, m34)
        /// newT = translation * rotation * scale * currentT
        /// Vivetracker is right-hand system
        /// Angle system trun z (roll) -> y pitch -> x yaw
        /// Ref:http://planning.cs.uiuc.edu/node102.html
        /// </summary>
        /// <param name="m"></param>
        /// <returns>yawPitchRoll (X,Y,Z) in degree</returns>
        public static ViveYawPitchRoll GetYawPitchRoll(Matrix4x4 m)
        {
            //return new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);

            // (A,X) - roll, (B,Y) - Pitch, (C,Z) - Yaw
            // A = 2(m21,m11)
            // B = 2(-m31,sqrt(m32^2 + m33^2)
            // C = 2(m32,m33)
            float roll = MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI;
            float pitch = MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI;
            float yaw = MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI;
            

            return new ViveYawPitchRoll(yaw, pitch, roll);
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

    public record VivePose(float X, float Y, float Z) : IEquatable<VivePose>
    {
        public static VivePose operator +(VivePose p1, VivePose p2)
        {
            return new VivePose(p1.X + p2.X, p1.Y + p2.Y, p1.Z + p2.Z);
        }

        public static VivePose operator -(VivePose p1, VivePose p2)
        {
            return new VivePose(p1.X - p2.X, p1.Y - p2.Y, p1.Z - p2.Z);
        }

        public static VivePose operator /(VivePose p1, float div)
        {
            return new VivePose(p1.X / div, p1.Y / div, p1.Z / div);
        }

        public static VivePose Zero()
        {
            return new VivePose(0,0,0);
        }

        public float Length()
        {
            return MathF.Sqrt(X * X + Y * Y + Z * Z);
        }

        //public static bool operator ==(VivePose p1, VivePose p2)
        //{
        //    return (p1.X == p2.X) && (p1.Y == p2.Y) && (p1.Z == p2.Z);
        //}
    }

    public record ViveYawPitchRoll(float Yaw, float Pitch, float Roll) : IEquatable<ViveYawPitchRoll>
    {
        public static ViveYawPitchRoll operator +(ViveYawPitchRoll p1, ViveYawPitchRoll p2)
        { 
            return new ViveYawPitchRoll(p1.Yaw + p2.Yaw, p1.Pitch + p2.Pitch, p1.Roll + p2.Roll);
        }

        public static ViveYawPitchRoll operator -(ViveYawPitchRoll p1, ViveYawPitchRoll p2)
        {
            return new ViveYawPitchRoll(p1.Yaw - p2.Yaw, p1.Pitch - p2.Pitch, p1.Roll - p2.Roll);
        }

        public static ViveYawPitchRoll operator /(ViveYawPitchRoll p1, float div)
        {
            return new ViveYawPitchRoll(p1.Yaw / div, p1.Pitch / div, p1.Roll / div);
        }
        public static ViveYawPitchRoll Zero()
        {
            return new ViveYawPitchRoll(0, 0, 0);
        }
    }

}
