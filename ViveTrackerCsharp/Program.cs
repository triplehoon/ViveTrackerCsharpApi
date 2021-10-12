using Hurel.PG.Positioning;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;

namespace ViveTrackerCsharp
{
    class Program
    {


        static void Main(string[] args)
        {
            ViveTrackerApi viveAPI = new ViveTrackerApi();
            var trackerList = viveAPI.GetTrackers;
            var trackerRefDevice = trackerList.Find(x => x.Serial == "LHR-D31C35BD");
            var test1 = trackerList.Find(x => x.Serial == "LHR-D31C35BD");
            var test2 = trackerList.Find(x => x.Serial == "LHR-3743F54D");
            var test3 = trackerList.Find(x => x.Serial == "LHR-90B84552");
            var Tracker1 = test1;
            var Tracker2 = test2;
            var Tracker3 = test3;
            var Trackers = new List<ViveTrackerApi.TrackedDevice> { Tracker1, Tracker2, Tracker3 };
            int iterationCount = 1;
            Matrix4x4 T1 = Matrix4x4.CreateFromYawPitchRoll(-90 * MathF.PI / 180, 90 * MathF.PI / 180, 0);
            Matrix4x4 T2 = Matrix4x4.CreateFromYawPitchRoll(-90 * MathF.PI / 180, -180 * MathF.PI / 180, 0);
            Matrix4x4 T3 = Matrix4x4.CreateFromYawPitchRoll(-90 * MathF.PI / 180, -90 * MathF.PI / 180, 0);
            float width = 0.246f;
            float height = 0.12f;
            Matrix4x4 OtoT1 = Matrix4x4.CreateTranslation(0, -width / 2, 0) *  T1;
            Matrix4x4 OtoT2 = Matrix4x4.CreateTranslation(0, 0, height)     *  T2;
            Matrix4x4 OtoT3 = Matrix4x4.CreateTranslation(0, +width / 2, 0) *  T3; 

            Matrix4x4 Tdevice1;
            Matrix4x4 Tdevice2;
            Matrix4x4 Tdevice3;
            Matrix4x4.Invert(OtoT1, out Tdevice1);
            Matrix4x4.Invert(OtoT2, out Tdevice2);
            Matrix4x4.Invert(OtoT3, out Tdevice3);

            viveAPI.SetGlobalTransformation(trackerRefDevice, OtoT1);


            while (true)
            {
               
                List<Matrix4x4[]> mList = viveAPI.GetTransformedMatirx(Trackers, iterationCount);
                Matrix4x4[] returnMList = new Matrix4x4[iterationCount * 3];
                for (int i = 0; i < iterationCount; ++i)
                {
                    returnMList[3 * i + 0] = mList[0][i] ;
                    returnMList[3 * i + 1] = mList[1][i] ;
                    returnMList[3 * i + 2] = mList[2][i] ;
                }

                Matrix4x4[] mTs = returnMList;

                Vector3 meanPos = Vector3.Zero;
                Vector3 meanROT = Vector3.Zero;

                foreach (var m in mTs)
                {
                    Vector3 pos = m.Translation;
                    Vector3 rot = new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);

                    Vector3 v = pos;
                    Vector3 r = rot;
                    Console.WriteLine($"{v.X * 1000:+0000.0;-0000.0;+0000.0}, {v.Y * 1000:+0000.0;-0000.0;+0000.0}, {v.Z * 1000:+0000.0;-0000.0;+0000.0} [mm], yaw: {r.Y:f0}, pitch: {r.Z:f0} roll: {r.X:f0}");
                    

                    meanPos += pos;
                    meanROT += rot;
                }
                meanPos = meanPos / (3 * iterationCount);
                meanROT = meanROT / (3 * iterationCount);

                Console.SetCursorPosition(0, Console.CursorTop - 3 * iterationCount);

            }

        }
    }




}
