using Hurel.PG.Positioning;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Text;
using System.Threading;

namespace ViveTrackerCsharp
{
    class Program
    {


        static void Main(string[] args)
        {
            ViveTrackerApi viveAPI = new ViveTrackerApi();
            var trackerList = viveAPI.GetTrackers;
            
            var test1 = trackerList.Find(x => x.Serial == "LHR-D31C35BD");
            var test2 = trackerList.Find(x => x.Serial == "LHR-3743F54D");
            var test3 = trackerList.Find(x => x.Serial == "LHR-90B84552");
            var Tracker1 = test1;
            var Tracker2 = test2;
            var Tracker3 = test3;
            var Trackers = new List<ViveTrackerApi.TrackedDevice> { Tracker1, Tracker2, Tracker3 };
            int iterationCount = 300;
            float width = 0.246f;
            float height = 0.12f;
            ///Vector3 zaxis = Vector3.Normalize(-forward);
            ///Vector3 xaxis = Vector3.Normalize(Vector3.Cross(up, zaxis));
            ///Vector3 yaxis = Vector3.Cross(zaxis, xaxis);
            Matrix4x4 OtoT1 = Matrix4x4.CreateWorld(new Vector3(0, -width / 2, 0), new Vector3(0, -1, 0), new Vector3(1, 0, 0));
            Matrix4x4 OtoT2 = Matrix4x4.CreateWorld(new Vector3(0,0, height), new Vector3(0, 0, 1), new Vector3(1, 0, 0));
            Matrix4x4 OtoT3 = Matrix4x4.CreateWorld(new Vector3(0, width / 2, 0), new Vector3(0, 1, 0), new Vector3(1, 0, 0));


            Matrix4x4 Tdevice1;
            Matrix4x4 Tdevice2;
            Matrix4x4 Tdevice3;
            Matrix4x4.Invert(OtoT1, out Tdevice1);
            Matrix4x4.Invert(OtoT2, out Tdevice2);
            Matrix4x4.Invert(OtoT3, out Tdevice3);

            var trackerRefDevice = trackerList.Find(x => x.Serial == "LHR-3743F54D");
            Matrix4x4 trackerRefPos = OtoT2;
            viveAPI.SetGlobalTransformation(trackerRefDevice, trackerRefPos);

            Matrix4x4 DetectorPose = new Matrix4x4();

            while (true)
            {
               
                List<Matrix4x4[]> mList = viveAPI.GetTransformedMatirx(Trackers, iterationCount);
                Matrix4x4[] returnMList = new Matrix4x4[iterationCount * 3];
                for (int i = 0; i < iterationCount; ++i)
                {
                    returnMList[3 * i + 0] = Tdevice1 * mList[0][i];
                    returnMList[3 * i + 1] = Tdevice2 * mList[1][i];
                    returnMList[3 * i + 2] = Tdevice3 * mList[2][i];
                }

                Matrix4x4[] mTs = returnMList;

                Vector3 meanPos = Vector3.Zero;
                Vector3 meanROT = Vector3.Zero;

                for (int i = 0; i < iterationCount; ++i)
                {
                    Matrix4x4 m = mTs[3 * i + 1];
                    Vector3 pos = m.Translation;
                    Vector3 rot = new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);
                    Vector3 v = pos;
                    Vector3 r = rot;
                    //Console.WriteLine($"{v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");


                    meanPos += pos;
                    meanROT += rot;
                }
                meanPos = meanPos / iterationCount;// / (3 * iterationCount);
                meanROT = meanROT / iterationCount;// / (3 * iterationCount);
                {
                    Vector3 v = meanPos;
                    Vector3 r = meanROT;
                    Console.WriteLine($"Mean: {v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");
                    Matrix4x4 Md = ViveTrackerApi.PosAndTrueYawPitchRoll(v, r);
                    DetectorPose = Md;

                }

                Console.WriteLine("Press enter to set current position as 0000000 And strat saving poses");
                if (Console.KeyAvailable)
                {
                    ConsoleKeyInfo c = Console.ReadKey(true);
                    
                    if (c.Key == ConsoleKey.Enter)
                    {
                        break;
                    }
                }

                Console.SetCursorPosition(0, Console.CursorTop - 2);
                
            }

            Matrix4x4 IDetectorPose;
            Matrix4x4.Invert(DetectorPose, out IDetectorPose);
            iterationCount = 60;
            //viveAPI.SetGlobalTransformation(trackerRefDevice, OtoT1);
            string strFilePath = $@"{DateTime.Now.ToString("HHmmss")}_Data.csv";
            string strSeperator = ",";
            StringBuilder sbOutput = new StringBuilder();
            DateTime start = DateTime.Now;

            while (true)
            {

                List<Matrix4x4[]> mList = viveAPI.GetTransformedMatirx(Trackers, iterationCount);
                Matrix4x4[] returnMList = new Matrix4x4[iterationCount * 3];
                for (int i = 0; i < iterationCount; ++i)
                {
                    returnMList[3 * i + 0] = IDetectorPose * Tdevice1 * mList[0][i];
                    returnMList[3 * i + 1] = IDetectorPose * Tdevice2 * mList[1][i];
                    returnMList[3 * i + 2] = IDetectorPose * Tdevice3 * mList[2][i];
                }

                Matrix4x4[] mTs = returnMList;

                Vector3 meanPos = Vector3.Zero;
                Vector3 meanROT = Vector3.Zero;

                for (int i = 0; i < iterationCount; ++i)
                {
                    Matrix4x4 m = mTs[3 * i + 1];
                    Vector3 pos = m.Translation;
                    Vector3 rot = new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);
                    Vector3 v = pos;
                    Vector3 r = rot;
                    //Console.WriteLine($"{v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");


                    meanPos += pos;
                    meanROT += rot;
                }
                meanPos = meanPos / iterationCount;// / (3 * iterationCount);
                meanROT = meanROT / iterationCount;// / (3 * iterationCount);
                {
                    Vector3 v = meanPos;
                    Vector3 r = meanROT;
                    Console.WriteLine($"Mean: {v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");
                    //Matrix4x4 Md = ViveTrackerApi.PosAndTrueYawPitchRoll(v, r);
                    //DetectorPose = Md;

                    sbOutput.AppendLine($"{(DateTime.Now - start).TotalSeconds} ,{v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00}, yaw, {r.Y:+000.00;-000.00;+000.00}, pitch, {r.Z:+000.00;-000.00;+000.00}, roll, {r.X:+000.00;-000.00;+000.00}");
                    

                }

                if (Console.KeyAvailable)
                {
                    ConsoleKeyInfo c = Console.ReadKey(true);

                    if (c.Key == ConsoleKey.Enter)
                    {
                        File.WriteAllText(strFilePath, sbOutput.ToString());
                        break;
                    }
                }

                Console.SetCursorPosition(0, Console.CursorTop - 1);

            }
        }


        
    }




}
