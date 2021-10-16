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
        public struct TrackerSerials
        {
            public const string LHR_90B84552 = "LHR-90B84552";
            public const string LHR_D31C35BD = "LHR-D31C35BD";
            public const string LHR_3743F54D = "LHR-3743F54D";
            //public const string LHR_90B84552 = "LHR-90B84552";
        }

        public readonly bool IsInitiated = false;
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
        public CalibrationWithViveTracker(string tracker1, string tracker2, string tracker3, string trackerRef, Matrix4x4 trackerRefPose)
        {
            viveAPI = new ViveTrackerApi();
            if (!viveAPI.IsInitiated)
            {
                IsInitiated = false;
                return;
            }
            var trackerList = viveAPI.GetTrackers;
            var trackerRefDevice = trackerList.Find(x => x.Serial == trackerRef);
            var test1 = trackerList.Find(x => x.Serial == tracker1);
            var test2 = trackerList.Find(x => x.Serial == tracker2);
            var test3 = trackerList.Find(x => x.Serial == tracker3);
            if (test1 == null || test2 == null || trackerRefDevice == null || viveAPI.SetGlobalTransformation(trackerRefDevice, trackerRefPose))
            {
                IsInitiated = false;
                WriteLine("Can't find tracker or set global transformation");
                return;
            }
            Tracker1 = test1;
            Tracker2 = test2;
            Tracker3 = test3;
            Trackers = new List<ViveTrackerApi.TrackedDevice> { Tracker1, Tracker2, Tracker3 };
            IsInitiated = true;
        }
        private ViveTrackerApi viveAPI;
        private ViveTrackerApi.TrackedDevice Tracker1;
        private ViveTrackerApi.TrackedDevice Tracker2;
        private ViveTrackerApi.TrackedDevice Tracker3;
        private List<ViveTrackerApi.TrackedDevice> Trackers;

        public (Vector3, Vector3) GetMultiSlitPosAndRot(int iterationCount = 10)
        {
            Matrix4x4[] mTs = GetMultiSlitPosMat(iterationCount);

            Vector3 meanPos = Vector3.Zero;
            Vector3 meanROT = Vector3.Zero;

            foreach (var m in mTs)
            {
                Vector3 pos = m.Translation;
                Vector3 rot = new Vector3(MathF.Atan2(m.M32, m.M33) * 180 / MathF.PI, MathF.Atan2(m.M21, m.M11) * 180 / MathF.PI, MathF.Atan2(-m.M31, MathF.Sqrt(m.M32 * m.M32 + m.M33 * m.M33)) * 180 / MathF.PI);
                meanPos += pos;
                meanROT += rot;
            }
            return (meanPos / (3 * iterationCount), meanROT / (3 * iterationCount));
        }
        public Matrix4x4[] GetMultiSlitPosMat(int iterationCount = 10)
        {
            if (viveAPI.IsInitiated == false)
            {
                return new Matrix4x4[0];
            }
           float width = 0.246f;
            float height = 0.12f;
            Matrix4x4 OtoT1 = Matrix4x4.CreateWorld(new Vector3(0, -width / 2, 0), new Vector3(0, -1, 0), new Vector3(1, 0, 0));
            Matrix4x4 OtoT2 = Matrix4x4.CreateWorld(new Vector3(0, 0, height), new Vector3(0, 0, 1), new Vector3(1, 0, 0));
            Matrix4x4 OtoT3 = Matrix4x4.CreateWorld(new Vector3(0, width / 2, 0), new Vector3(0, 1, 0), new Vector3(1, 0, 0));


            Matrix4x4 Tdevice1;
            Matrix4x4 Tdevice2;
            Matrix4x4 Tdevice3;
            Matrix4x4.Invert(OtoT1, out Tdevice1);
            Matrix4x4.Invert(OtoT2, out Tdevice2);
            Matrix4x4.Invert(OtoT3, out Tdevice3);

            List<Matrix4x4[]> mList = viveAPI.GetTransformedMatirx(Trackers, iterationCount);
            Matrix4x4[] returnMList = new Matrix4x4[iterationCount * 3];
            for (int i = 0; i < iterationCount; ++i)
            {
                returnMList[3 * i + 0] = Tdevice1 * mList[0][i];
                returnMList[3 * i + 1] = Tdevice2 * mList[1][i];
                returnMList[3 * i + 2] = Tdevice3 * mList[2][i];
            }


            return returnMList;
        }
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
