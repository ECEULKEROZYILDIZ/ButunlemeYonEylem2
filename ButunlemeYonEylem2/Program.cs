using System;
using System.Collections.Generic;
using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes;

/// <summary>
///   Minimal TSP using distance matrix.
/// </summary>
public class VrpCapacity
{
    class DataModel
    {
        public long[,] DistanceMatrix = {
            { 37,23,28,19,35 },
            { 36,25,20,26,29},
            { 26,19,17,27,33},
        };
        public long[] Demands = { 45, 52, 38, 62, 163 };
        public long[] VehicleCapacities = { 120, 110, 130 };
        public int VehicleNumber = 3;
        public int Depot = 1;
    };

    /// <summary>
    ///   Print the solution.
    /// </summary>
    static void PrintSolution(in DataModel data, in RoutingModel routing, in RoutingIndexManager manager,
                              in Assignment solution)
    {
        Console.WriteLine($"Objective {solution.ObjectiveValue()}:");

        // Inspect solution.
        long totalDistance = 0;
        long totalLoad = 0;
        for (int i = 0; i < data.VehicleNumber; ++i)
        {
            Console.WriteLine("Route for Vehicle {0}:", i);
            long routeDistance = 0;
            long routeLoad = 0;
            var index = routing.Start(i);
            while (routing.IsEnd(index) == false)
            {
                long nodeIndex = manager.IndexToNode(index);
                routeLoad += data.Demands[nodeIndex];
                Console.Write("{0} Load({1}) -> ", nodeIndex, routeLoad);
                var previousIndex = index;
                index = solution.Value(routing.NextVar(index));
                routeDistance += routing.GetArcCostForVehicle(previousIndex, index, 0);
            }
            Console.WriteLine("{0}", manager.IndexToNode((int)index));
            Console.WriteLine("Distance of the route: {0}m", routeDistance);
            totalDistance += routeDistance;
            totalLoad += routeLoad;
        }
        Console.WriteLine("Total distance of all routes: {0}m", totalDistance);
        Console.WriteLine("Total load of all routes: {0}m", totalLoad);
    }

    public static void Main(String[] args)
    {
        // Instantiate the data problem.
        DataModel data = new DataModel();

        // Create Routing Index Manager
        RoutingIndexManager manager =
            new RoutingIndexManager(data.DistanceMatrix.GetLength(0), data.VehicleNumber, data.Depot);

        // Create Routing Model.
        RoutingModel routing = new RoutingModel(manager);

        // Create and register a transit callback.
        int transitCallbackIndex = routing.RegisterTransitCallback((long fromIndex, long toIndex) =>
        {
            // Convert from routing variable Index to
            // distance matrix NodeIndex.
            var fromNode = manager.IndexToNode(fromIndex);
            var toNode = manager.IndexToNode(toIndex);
            return data.DistanceMatrix[fromNode, toNode];
        });

        // Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

        // Add Capacity constraint.
        int demandCallbackIndex = routing.RegisterUnaryTransitCallback((long fromIndex) =>
        {
            // Convert from routing variable Index to
            // demand NodeIndex.
            var fromNode =
                manager.IndexToNode(fromIndex);
            return data.Demands[fromNode];
        });
        routing.AddDimensionWithVehicleCapacity(demandCallbackIndex, 0, // null capacity slack
                                                data.VehicleCapacities, // vehicle maximum capacities
                                                true,                   // start cumul to zero
                                                "Capacity");

        // Setting first solution heuristic.
        RoutingSearchParameters searchParameters =
            operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
        searchParameters.LocalSearchMetaheuristic = LocalSearchMetaheuristic.Types.Value.GuidedLocalSearch;
        searchParameters.TimeLimit = new Duration { Seconds = 1 };

        // Solve the problem.
        Assignment solution = routing.SolveWithParameters(searchParameters);

        // Print solution on console.
        PrintSolution(data, routing, manager, solution);
    }
}
