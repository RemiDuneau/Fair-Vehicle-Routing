import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class SimLoop {

    public static int NUM_ITERATIONS = 100;
    public static boolean isTrackingVehicles = false;

    private static int incrementCount;
    public static int getIncrementCount() {
        return incrementCount;
    }

    public static void main(String[] args) throws IOException {
        FileWriter fileWriter = new FileWriter("results.csv", true);
        BufferedWriter writer = new BufferedWriter(fileWriter);

        for (int asdf = 0; asdf < 5; asdf += 2) {
            int routingType = asdf;
            //Routing.least_density_safe_threshold = asdf;
            int initial = 500;
            int numCycles = 9;
            int vehiclesIncrement = 250;
            for (int numVehiclesBase = initial; numVehiclesBase < numCycles * vehiclesIncrement + initial; numVehiclesBase += vehiclesIncrement) {

                //init variables
                int numVehicles = numVehiclesBase * 5;
                TimeController.NUM_VEHICLES = (numVehicles);
                Graph graph = XMLParser.parseXML(new File("StreetGridGraph.xml"));
                TimeController controller = new TimeController(graph);
                controller.createRandomNodeVehicles();
                Vehicle[] vehicles = controller.getVehicles();
                ArrayList<Vehicle> trackedVehicles = controller.getTrackedVehicles();
                ArrayList<Vehicle> activeVehicles = controller.getActiveVehicles();
                Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = controller.getAllPathsMap();
                ArrayList<Road> roads = controller.getRoads();

                //find all paths from every vehicle's start node to end node
                for (Vehicle v : vehicles) {
                    v.setRoutingType(routingType);
                    Node startNode = v.getStartNode();
                    Node endNode = v.getEndNode();
                    boolean contains = false;
                    for (Tuple<Node, Node> tuple : allPathsMap.keySet()) {
                        if ((tuple.x == startNode) && (tuple.y == endNode)) {
                            contains = true;
                        }
                    }
                    if (!contains) {
                        //ArrayList<Stack<Road>> allPaths = Routing.dfsFindAllPaths(startNode, endNode);
                        //allPathsMap.put(new Tuple<>(startNode, endNode), allPaths);
                    }
                }
                System.out.println("all paths map size: " + allPathsMap.size());

                //POPULATE NETWORK
                System.out.println("Populating Network...");
                isTrackingVehicles = false;
                int initIterations = 50;
                for (int i = 0; i < initIterations; i++) {
                    controller.incrementTime(0, 0);
                }

                isTrackingVehicles = true;

                ArrayList<ArrayList<Integer>> roadSizesSimLoop = new ArrayList<>();
                for (Road road : roads) {
                    roadSizesSimLoop.add(new ArrayList<>());
                }

                //MAIN LOOP
                for (int i = 0; i < NUM_ITERATIONS; i++) {
                    incrementCount = i;
                    System.out.print("TIME: " + i);
                    controller.incrementTime();
    /*
    if (incrementCount >= controller.ADDEDVEHICLES) {
        for (int j = 0; j < roads.size(); j++) {
            Road r = roads.get(j);
            roadSizesSimLoop.get(j).add(r.getVehicles().size());
        }
    }
     */
                    System.out.println(" (Active Vehicles: " + activeVehicles.size() + ")" + " (Tracked Vehicles: " + trackedVehicles.size() + ")");
                }

                //keep incrementing time until all tracked vehicles are finished
                isTrackingVehicles = false;
                boolean isAllVehiclesFinished = false;
                while (!isAllVehiclesFinished) {
                    controller.incrementTime();

                    //check if all vehicles are finished
                    boolean isFinished = true;
                    for (Vehicle v : trackedVehicles) {
                        if (!v.isFinished()) {
                            isFinished = false;
                        }
                    }
                    isAllVehiclesFinished = isFinished;
                }

                //--------- STATS ---------

                System.out.println();

                //vehicle/road tracking stats stuff
                int totalDistance = 0;
                int totalTimeTakenToFinishTrip = 0;
                int numFinishedTrips = 0;
                int nTrackedVehicles = trackedVehicles.size();
                int totalTime = 0;
                for (Vehicle vehicle : trackedVehicles) {
                    totalDistance += vehicle.getTotalDistance();
                    totalTime += vehicle.getActualTripTime();
                    if (vehicle.isFinished()) {
                        totalTimeTakenToFinishTrip += vehicle.getActualTripTime();
                        numFinishedTrips++;
                    }
                }

                double proportionOfFinishedTrips = numFinishedTrips / (double) nTrackedVehicles;


                //average trips
                double avgTimeTakenToFinishTrip = totalTimeTakenToFinishTrip / (double) numFinishedTrips;

                System.out.println("Average time taken to complete a trip: " + avgTimeTakenToFinishTrip + "(proportion of trips finished: " + proportionOfFinishedTrips + ")");
                double avgDistance = totalDistance / (double) nTrackedVehicles;
                double averageSpeed = avgDistance / (double) (totalTime / (double) nTrackedVehicles);
                System.out.println("Average speed: " + (averageSpeed));

                //average density
                double averageDensitySum = 0;
                for (Road road : roads) {
                    double averageDensity = road.getDensitySum() / NUM_ITERATIONS;
                    //System.out.println("Road " + road.getNodesAsString() + " average density: " + averageDensity);
                    averageDensitySum += averageDensity;
                }
                double totalAverageDensity = averageDensitySum / roads.size();
                System.out.println("Total average density = " + totalAverageDensity);

                System.out.println("Vehicles sent out: " + controller.getVehiclesSentOut());

                //------- time differences -------
                ArrayList<Double> optimalTimeDifferenceList = new ArrayList<>();
                ArrayList<Double> dijkstraTimeDifferenceList = new ArrayList<>();
                for (int i = 0; i < trackedVehicles.size(); i++) {
                    Vehicle vehicle = trackedVehicles.get(i);
                    //for (Vehicle vehicle : trackedVehicles) {
                    if (vehicle.isFinished()) {
                        optimalTimeDifferenceList.add(vehicle.calculateOptimalTimeDifference());
                        double dijTime = vehicle.getDijkstraTripTime();
                        if (dijTime < Integer.MAX_VALUE) {
                            dijkstraTimeDifferenceList.add(vehicle.calculateDijkstraTimeDifference());
                            if (dijTime > vehicle.getActualTripTime()) {
                                //System.out.println(i);
                            }
                        }
                    }
                }

                Collections.sort(optimalTimeDifferenceList);
                System.out.println(optimalTimeDifferenceList);

                Collections.sort(dijkstraTimeDifferenceList);
                double optimalDiffAverage = AverageUtil.calcAverageDouble(optimalTimeDifferenceList);
                System.out.println("optimalDiff avg " + optimalDiffAverage);

                int size = (int) ((0.9) * (double) optimalTimeDifferenceList.size());
                ArrayList<Double> optimalTimeDifferenceWorst10Pct = new ArrayList<>();
                for (int i = size; i < optimalTimeDifferenceList.size(); i++) {
                    optimalTimeDifferenceWorst10Pct.add(optimalTimeDifferenceList.get(i));
                }
                double optimalDiffWorst10PctAvg = AverageUtil.calcAverageDouble(optimalTimeDifferenceWorst10Pct);
                System.out.println("worst 10% average: " + optimalDiffWorst10PctAvg);


                System.out.println(dijkstraTimeDifferenceList);
                double dijDiffAverage = AverageUtil.calcAverageDouble(dijkstraTimeDifferenceList);
                System.out.println("dijTimeDiff avg " + dijDiffAverage);

                ArrayList<Double> dijTimDifferenceWorst10Pct = new ArrayList<>();
                for (int i = size; i < optimalTimeDifferenceList.size(); i++) {
                    dijTimDifferenceWorst10Pct.add(dijkstraTimeDifferenceList.get(i));
                }
                double dijDiffWorst10PctAvg = AverageUtil.calcAverageDouble(dijTimDifferenceWorst10Pct);
                System.out.println("worst 10% average: " + dijTimDifferenceWorst10Pct);

                System.out.println("least density threshold " + Routing.least_density_safe_threshold);

                writer.write(routingType + ", " + numVehicles + ", " + averageSpeed + ", " + avgTimeTakenToFinishTrip + ", " + proportionOfFinishedTrips + ", "
                        + optimalDiffAverage + ", " + optimalDiffWorst10PctAvg + ", " + dijDiffAverage + ", " + dijDiffWorst10PctAvg + ","
                        + controller.getProcessingTime() + ", " + Routing.least_density_safe_threshold + "\n");
                writer.flush();


/*
        //PRINT OUT WHAT'S GOING WRONG
        ArrayList<ArrayList<Integer>> controllerSizes = controller.roadSizes;
        for (int i = 0; i < controllerSizes.size(); i++) {
            ArrayList<Integer> dijSize = controllerSizes.get(i);
            ArrayList<Integer> actualSize = roadSizesSimLoop.get(i);
            boolean isBad = false;
            for (int j = 0; j < dijSize.size(); j++) {
                int dij = dijSize.get(j);
                int actual = actualSize.get(j);
                if (dij - actual != 0) isBad = true;
            }
            if (isBad) {
                Road road = roads.get(i);
                System.out.println("ROAD " + i + " (" + road.getNodesAsString() + "): ");
                for (int j = 0; j < dijSize.size(); j++) {
                    int dij = dijSize.get(j);
                    int actual = actualSize.get(j);
                    System.out.print(dij - actual + ", ");
                }
                System.out.println();
            }
        }
*/

                System.out.println("----------------------------");
/*
        //write/append to file
        try {
            FileWriter fileWriter = new FileWriter("results.csv", true);
            BufferedWriter writer = new BufferedWriter(fileWriter);
            writer.write(routingType + ", " + TimeController.NUM_VEHICLES + ", " + avgTimeTakenToFinishTrip + ", "
                    + tripsCompletedStdDev + ", " + totalAverageDensity + ", " + controller.getVehiclesSentOut() + "\n");
            writer.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
*/
            }
        }
    }
}