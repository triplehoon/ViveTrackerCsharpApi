using Hurel.PG.Positioning;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Hurel.PG.Multislit
{
    public class CalibrationWithViveTracker
    {

        /// <summary>
        ///           
        ///                  <FrontView>
        ///                   tracker2  
        ///           ____________________________
        ///           |                          |                       
        ///           |                          |                           
        ///           |                          |
        ///  tracker1 |            O             |tracker3
        ///           |                          |
        ///           |                          |
        ///           |__________________________|
        /// Point all usb type-c port toward front (y-axis set as front for each tracker)
        /// </summary>
        /// <param name="tracker1"></param>
        /// <param name="tracker2"></param>
        /// <param name="tracker3"></param>
        /// <param name="trackerRef"></param>
        /// <param name="trackerRefPose"></param>
        public CalibrationWithViveTracker()
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
                WriteLine("Fail to set global transformation");
                return;
            }


            ViveTrackerApi.TrackedDevice trackerT1 = trackerList.Find(x => x.Serial == "LHR-FD4470E3");
            ViveTrackerApi.TrackedDevice trackerT2 = trackerList.Find(x => x.Serial == "LHR-FEA44485");
            ViveTrackerApi.TrackedDevice trackerT3 = trackerList.Find(x => x.Serial == "LHR-CA21B862");

            float width = 0.135f;
            float height = 0.121f;
            float behind = 0.2425f;

            Matrix4x4 Tdevice1ToTdevice2 = ViveTrackerApi.GetMatrixFromPosAndTrueYawPitchRoll(new VivePose(+width / 2, height, 0), new ViveYawPitchRoll(90, 0, 0));
            Matrix4x4 Tdevice2ToTdevice2 = ViveTrackerApi.GetMatrixFromPosAndTrueYawPitchRoll(new VivePose(0, 0, 0), new ViveYawPitchRoll(0, 0, 0));
            Matrix4x4 Tdevice3ToTdevice2 = ViveTrackerApi.GetMatrixFromPosAndTrueYawPitchRoll(new VivePose(-width / 2, height, 0), new ViveYawPitchRoll(-90, 0, 0));
            Matrix4x4 Tdevice2ToMultslit = ViveTrackerApi.GetMatrixFromPosAndTrueYawPitchRoll(new VivePose(0, behind, -height), new ViveYawPitchRoll(-90, 0, 180));
            List<ViveTrackerApi.TrackedDevice> multiSlitTracker = new List<ViveTrackerApi.TrackedDevice>() { trackerT1, trackerT2, trackerT3 };

            int iterationCount = 1;

            while (true)
            {
                List<Matrix4x4[]> mList = viveAPI.GetTransformedMatrixs(multiSlitTracker, iterationCount);
                Matrix4x4[] returnMList = new Matrix4x4[iterationCount * 3];
                for (int i = 0; i < iterationCount; ++i)
                {
                    returnMList[3 * i + 0] = mList[0][i];
                    returnMList[3 * i + 1] = mList[1][i];
                    returnMList[3 * i + 2] = mList[2][i];
                }

                Matrix4x4[] mTs = returnMList;

                VivePose meanPos = VivePose.Zero();
                ViveYawPitchRoll meanROT = ViveYawPitchRoll.Zero();

                for (int i = 0; i < iterationCount * 3; ++i)
                {
                    Matrix4x4 m = mTs[i];
                    VivePose pos = ViveTrackerApi.GetPosition(m);
                    ViveYawPitchRoll rot = ViveTrackerApi.GetYawPitchRoll(m);
    
                    Console.WriteLine($"{pos.X * 1000:+0000.00;-0000.00;+0000.00}, {pos.Y * 1000:+0000.00;-0000.00;+0000.00}, {pos.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {rot.Yaw:+000.00;-000.00;+000.00}, pitch: {rot.Pitch:+000.00;-000.00;+000.00} roll: {rot.Roll:+000.00;-000.00;+000.00}");


                    meanPos += pos;
                    meanROT += rot;
                }
                meanPos = meanPos / iterationCount / 3;// / (3 * iterationCount);
                meanROT = meanROT / iterationCount / 3;// / (3 * iterationCount);
                {
                    VivePose v = meanPos;
                    ViveYawPitchRoll r = meanROT;
                    Console.WriteLine($"Mean: {v.X * 1000:+0000.00;-0000.00;+0000.00}, {v.Y * 1000:+0000.00;-0000.00;+0000.00}, {v.Z * 1000:+0000.00;-0000.00;+0000.00} [mm], yaw: {r.Yaw:+000.00;-000.00;+000.00}, pitch: {r.Pitch:+000.00;-000.00;+000.00} roll: {r.Roll:+000.00;-000.00;+000.00}");

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


            IsInitiated = true;
        }

        public readonly bool IsInitiated = false;



        private static void WriteLine(string msg)
        {
            try
            {
                Console.WriteLine($"CalibrationWithViveTracker: {msg}");
            }
            catch
            {
                System.Diagnostics.Debug.WriteLine($"CalibrationWithViveTracker: {msg}");
            }
        }
    }
}
