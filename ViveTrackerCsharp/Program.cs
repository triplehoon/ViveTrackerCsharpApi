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

            ViveTrackerApi.TrackedDevice trackerIECY = trackerList.Find(x => x.Serial == "LHR-D31C35BD");
            var trackerOrigin = trackerList.Find(x => x.Serial == "LHR-3743F54D");
            var trackerIECX = trackerList.Find(x => x.Serial == "LHR-90B84552");
            var trackerIECZ = trackerList.Find(x => x.Serial == "LHR-7E9098E0");
            var trackerTestRef = trackerList.Find(x => x.Serial == "LHR-A07E549B");

            var trackersIEC = new List<ViveTrackerApi.TrackedDevice> { trackerOrigin, trackerIECX, trackerIECY, trackerIECZ };

            if (!viveAPI.SetGlobalTransformation(trackerOrigin, trackerIECX, trackerIECY, trackerIECZ))
            {
                Console.WriteLine("Fail to set global transformation");
                return;
            }

            int iterationCount = 1;

            //while (true)
            //{
            //    Matrix4x4[] returnMList = viveAPI.GetTransformedMatrix(trackerTestRef, iterationCount);
            //    Matrix4x4[] mTs = returnMList;
            //    Vector3 meanPos = Vector3.Zero;
            //    Vector3 meanROT = Vector3.Zero;
            //    for (int i = 0; i < iterationCount; ++i)
            //    {
            //        Matrix4x4 m = mTs[i];
            //        Vector3 pos = m.Translation;
            //        Vector3 rot = ViveTrackerApi.GetRotation(m);
            //        Vector3 v = pos;
            //        Vector3 r = rot;
            //        //Console.WriteLine($"{v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");
            //        meanPos += pos;
            //        meanROT += rot;
            //    }
            //    meanPos = meanPos / iterationCount;// / (3 * iterationCount);
            //    meanROT = meanROT / iterationCount;// / (3 * iterationCount);
            //    {
            //        Vector3 v = meanPos;
            //        Vector3 r = meanROT;
            //        Console.WriteLine($"Mean: {v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.X:+000.00;-000.00;+000.00}, pitch: {r.Y:+000.00;-000.00;+000.00} roll: {r.Z:+000.00;-000.00;+000.00}");
            //        Console.SetCursorPosition(0, Console.CursorTop - 1);
            //    }
            //}
            ViveTrackerApi.TrackedDevice trackerT1 = trackerList.Find(x => x.Serial == "LHR-FD4470E3");
            ViveTrackerApi.TrackedDevice trackerT2 = trackerList.Find(x => x.Serial == "LHR-FEA44485");
            ViveTrackerApi.TrackedDevice trackerT3 = trackerList.Find(x => x.Serial == "LHR-CA21B862");

            float width = 0.256f;
            float height = 0.120f;
            float behind = 0.243f;

            Matrix4x4 Tdevice1 = new Matrix4x4(0, 0, -1, 0, 0, 1, 0, behind, 1, 0, 0, 0, 0, 0, 0, 1);
            Matrix4x4 Tdevice2 = new Matrix4x4(1, 0, 0, 0,    0, 1, 0, behind,      0, 0, 1, 0, 0, 0, 0, 1);
            Matrix4x4 Tdevice3 = new Matrix4x4(0, 0, 1, 0, 0, 1, 0, behind, -1, 0, 0, 0, 0, 0, 0, 1);

            Matrix4x4 Tdevice1Trans =  Matrix4x4.CreateTranslation(-width / 2, behind, 0);
            Matrix4x4 Tdevice2Trans = Matrix4x4.CreateTranslation(0, behind, -height);
            Matrix4x4 Tdevice3Trans =  Matrix4x4.CreateTranslation(width / 2, behind, 0);

            List<ViveTrackerApi.TrackedDevice> multiSlitTracker = new List<ViveTrackerApi.TrackedDevice>() { trackerT1, trackerT2, trackerT3 };

            while (true)
            {
                List<Matrix4x4[]> mList = viveAPI.GetTransformedMatrixs(multiSlitTracker, iterationCount);
                Matrix4x4[] returnMList = new Matrix4x4[iterationCount * 3]; // 900개
                for (int i = 0; i < iterationCount; ++i)
                {
                    returnMList[3 * i + 0] = Tdevice1 * mList[0][i] * Tdevice1Trans;
                    returnMList[3 * i + 1] = Tdevice2 * mList[1][i] * Tdevice2Trans;
                    returnMList[3 * i + 2] = Tdevice3 * mList[2][i] * Tdevice3Trans;
                }

                Matrix4x4[] mTs = returnMList;

                Vector3 meanPos = Vector3.Zero;
                Vector3 meanROT = Vector3.Zero;

                for (int i = 0; i < iterationCount * 3; ++i)
                {
                    Matrix4x4 m = mTs[i];
                    Vector3 pos = m.Translation;
                    Vector3 rot = ViveTrackerApi.GetRotation(m);
                    Vector3 v = pos;
                    Vector3 r = rot;
                    Console.WriteLine($"{v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");


                    meanPos += pos;
                    meanROT += rot;
                }
                meanPos = meanPos / iterationCount / 3;// / (3 * iterationCount);
                meanROT = meanROT / iterationCount / 3;// / (3 * iterationCount);
                {
                    Vector3 v = meanPos;
                    Vector3 r = meanROT;
                    Console.WriteLine($"Mean: {v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Y:+000.00;-000.00;+000.00}, pitch: {r.Z:+000.00;-000.00;+000.00} roll: {r.X:+000.00;-000.00;+000.00}");

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
                Console.SetCursorPosition(0, Console.CursorTop - 5);
            }


        }



    }




}
