using Hurel.PG.Positioning;
using System;
using System.Numerics;
using System.Threading;

namespace ViveTrackerCsharp
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Init openVR");
            ViveTrackerApi viveAPI = new ViveTrackerApi();

            viveAPI.GetPosesAndWrite();

            var trackerList = viveAPI.GetTrackers;
            var trackerMiddle = trackerList.Find(x => x.Serial == "LHR-90B84552");
            viveAPI.SetGlobalTransformation(trackerMiddle, Vector3.Zero);
           
            while(true)
            {
                viveAPI.GetTransPosesAndWrite();
                Thread.Sleep(0);
            }
            

        }
    }
}
