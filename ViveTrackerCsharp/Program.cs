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
            Console.WriteLine("Init openVR");
            ViveTrackerApi viveAPI = new ViveTrackerApi();

            
            var trackerList = viveAPI.GetTrackers;
            var trackerMiddle = trackerList.Find(x => x.Serial == "LHR-90B84552");
            viveAPI.SetGlobalTransformation(trackerMiddle, Vector3.Zero);
           
            while(true)
            {
                Thread.Sleep(0);
                List<Matrix4x4> m = viveAPI.GetTransformedMatrix(viveAPI.GetTrackers);
                Vector3 tr = ViveTrackerApi.GetPosition(m[0]);
                Vector3 euler = ViveTrackerApi.GetEulerXYZ(m[0]);
                Console.WriteLine($"Pos   x: {tr.X} y: {tr.Y} z: {tr.Z}");
                Console.WriteLine($"Euler x: {euler.X / 3.14f * 360} y: {euler.Y / 3.14f * 360} z: {euler.Z / 3.14f * 360}");
                Console.SetCursorPosition(0, Console.CursorTop - 2);

            }


        }
    }
}
