import java.io.*;
import java.util.*;

public class SimLoop {

    public static int NUM_ITERATIONS = 100, INIT_ITERATIONS = 50;
    private static int LONG_TERM_ROUTING_NUM_TRIPS = 7;
    public static boolean isTrackingVehicles = false;



    private static int incrementCount;
    public static int getIncrementCount() {
        return incrementCount;
    }

    public static boolean isLongTermRouting = true;


    public static void main(String[] args) throws IOException {
        FileWriter fileWriter = new FileWriter("longTermStatic.csv", true);
        BufferedWriter writer = new BufferedWriter(fileWriter);
        boolean isWrite = false;

        int[] routingTypes = {2};
        for (int type = 0; type < routingTypes.length; type++) {

            //tracking stats
            double avgTimeTakenToFinishTrip = 0;

            int routingType = routingTypes[type];
            int initial = 500;
            int numCycles = 9;
            int vehiclesIncrement = 250;
            for (int numVehiclesBase = initial; numVehiclesBase < numCycles * vehiclesIncrement + initial; numVehiclesBase += vehiclesIncrement) {
                System.out.println("TRACKED VEHICLES: " + numVehiclesBase);

                //init variables
                int numVehicles = numVehiclesBase * 5;
                TimeController.NUM_VEHICLES = (numVehicles);
                Graph graph = XMLParser.parseXML(new File("Berlin Example.xml"));
                TimeController controller = new TimeController(graph);
                controller.createRandomNodeVehicles();
                Vehicle[] vehicles = controller.getVehicles();
                ArrayList<Vehicle> trackedVehicles = controller.getTrackedVehicles();
                ArrayList<Vehicle> activeVehicles = controller.getActiveVehicles();
                Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = controller.getAllPathsMap();
                ArrayList<Road> roads = controller.getRoads();
                ArrayList<ArrayList<Double>> dijDiffLists = new ArrayList();

                controller.setLongTermRouting(isLongTermRouting);
                Routing.resetDijkstra_diff_thresholdStats();

                //find all paths from every vehicle's start node to end node
                for (Vehicle v : vehicles) {
                    v.setRoutingType(routingType);

                    if (Routing.isDynamicRouting) {
                        v.setDynamicRouting(true);
                        v.setTimeController(controller);
                    }

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

                int nTrips = 1;
                if (isLongTermRouting) nTrips = LONG_TERM_ROUTING_NUM_TRIPS;
                for (int day = 0; day < nTrips; day++) {
                    System.out.println("DAY: " + day);
                    controller.resetForNextDay();

                    //POPULATE NETWORK
                    //System.out.println("Populating Network...");
                    isTrackingVehicles = false;
                    for (int i = 0; i < INIT_ITERATIONS; i++) {
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
                        //if (day == 2 && i == 99) {
                        //    System.out.print("");
                        //}
                        //System.out.print("TIME: " + i);
                        controller.incrementTime();
                    /*
                    if (incrementCount >= controller.ADDEDVEHICLES) {
                        for (int j = 0; j < roads.size(); j++) {
                            Road r = roads.get(j);
                            roadSizesSimLoop.get(j).add(r.getVehicles().size());
                        }
                    }
                     */
                        double totalDensityPerIteration = 0;
                        for (Road road : roads) {
                            totalDensityPerIteration += road.getDensity();
                        }

                        //System.out.println(" (Active Vehicles: " + activeVehicles.size() + ")" + " (Tracked Vehicles: " + trackedVehicles.size() + ")" + " (Avg Density: " + totalDensityPerIteration / (double) roads.size()
                        //        + ") (DijDiff Threshold: " + Routing.dijkstra_diff_threshold + ") (Avg unfairness: " + ListsUtil.calcAverageDouble(controller.getVehicleUnfairnessList()) + ")");
                    }

                    //keep incrementing time until all tracked vehicles are finished
                    isTrackingVehicles = false;
                    boolean isAllVehiclesFinished = false;
                    while (!isAllVehiclesFinished) {
                        controller.incrementTime();

                        //check if all vehicles are finished
                        boolean isFinished = true;
                        isFinishedLoop:
                        for (Vehicle v : trackedVehicles) {
                            if (!v.isFinished()) {
                                isFinished = false;
                                break isFinishedLoop;
                            }
                        }
                        isAllVehiclesFinished = isFinished;
                    }


                    //------------------
                    //      STATS
                    //------------------
                    //System.out.println();

                    //vehicle/road tracking stats stuff
                    int totalDistance = 0;
                    int totalTimeTakenToFinishTrip = 0;
                    int numFinishedTrips = 0;
                    int nTrackedVehicles = trackedVehicles.size();
                    int totalTime = 0;
                    for (Vehicle vehicle : trackedVehicles) {
                        totalDistance += vehicle.getTripDistance();
                        totalTime += vehicle.getActualTripTime();
                        if (vehicle.isFinished()) {
                            totalTimeTakenToFinishTrip += vehicle.getActualTripTime();
                            numFinishedTrips++;
                        }
                    }

                    double proportionOfFinishedTrips = numFinishedTrips / (double) nTrackedVehicles;


                    //average trips
                    avgTimeTakenToFinishTrip = totalTimeTakenToFinishTrip / (double) numFinishedTrips;

                    System.out.println("Average time taken to complete a trip: " + avgTimeTakenToFinishTrip + "(proportion of trips finished: " + proportionOfFinishedTrips + ")");
                    double avgDistance = totalDistance / (double) nTrackedVehicles;
                    double averageSpeed = avgDistance / (totalTime / (double) nTrackedVehicles);
                    //System.out.println("Average speed: " + (averageSpeed));

                    //average density
                    double averageDensitySum = 0;
                    for (Road road : roads) {
                        double averageDensity = road.getDensitySum() / NUM_ITERATIONS;
                        //System.out.println("Road " + road.getNodesAsString() + " average density: " + averageDensity);
                        averageDensitySum += averageDensity;
                    }
                    double totalAverageDensity = averageDensitySum / roads.size();
                    //System.out.println("Total average density = " + totalAverageDensity);

                    //System.out.println("Vehicles sent out: " + trackedVehicles.size());

                    //------- time differences -------
                    ArrayList<Double> optimalTimeDifferenceList = new ArrayList<>();
                    ArrayList<Double> dijkstraTimeDifferenceList = new ArrayList<>();
                    for (int i = 0; i < trackedVehicles.size(); i++) {
                        Vehicle vehicle = trackedVehicles.get(i);
                        //for (Vehicle vehicle : trackedVehicles) {
                        if (vehicle.isFinished()) {
                            double optimalTimeDifference = vehicle.calculateOptimalTimeDifference();
                            optimalTimeDifferenceList.add(optimalTimeDifference);
                            if (optimalTimeDifference < 0) {
                                int optimalTime = vehicle.getOptimalTripTime();
                                int actualTime = vehicle.getActualTripTime();
                                //System.out.println("IMPOSSIBLE TRIP TIME FOR VEHICLE: " + i + "(actual trip time is lower than optimal trip time -> actual: " + actualTime + ", optimal: " + optimalTime + ")");
                            }
                            double dijTime = vehicle.getDijkstraTripTime();
                            if (dijTime < Integer.MAX_VALUE) {
                                dijkstraTimeDifferenceList.add(vehicle.calculateDijkstraTimeDifference());
                                if (dijTime > vehicle.getActualTripTime()) {
                                    //System.out.println(i);
                                }
                            }
                        }
                    }

                    ArrayList<Double> orderedDijkstraTimeDifferenceList = (ArrayList<Double>) dijkstraTimeDifferenceList.clone();
                    dijDiffLists.add(orderedDijkstraTimeDifferenceList);

                    Collections.sort(optimalTimeDifferenceList);
                    Collections.sort(dijkstraTimeDifferenceList);
                    //System.out.println("optimalDiff size: " + optimalTimeDifferenceList.size());
                    //System.out.println(optimalTimeDifferenceList);

                    double optimalDiffAverage = ListsUtil.calcAverageDouble(optimalTimeDifferenceList);
                    //System.out.println("optimalDiff avg " + optimalDiffAverage);

                    ArrayList<Double> optimalTimeDifferenceWorst10Pct = (ArrayList<Double>) ListsUtil.findWorstNPercent(0.9, optimalTimeDifferenceList);
                    double optimalDiffWorst10PctAvg = ListsUtil.calcAverageDouble(optimalTimeDifferenceWorst10Pct);
                    //System.out.println("worst 10% average: " + optimalDiffWorst10PctAvg);


                    //System.out.println(dijkstraTimeDifferenceList);
                    //System.out.println("dijDiff size: " + dijkstraTimeDifferenceList.size());
                    double dijDiffAverage = ListsUtil.calcAverageDouble(dijkstraTimeDifferenceList);
                    //System.out.println("dijTimeDiff avg " + dijDiffAverage);

                    ArrayList<Double> dijTimDifferenceWorst10Pct = (ArrayList<Double>) ListsUtil.findWorstNPercent(0.9, optimalTimeDifferenceList);
                    double dijDiffWorst10PctAvg = ListsUtil.calcAverageDouble(dijTimDifferenceWorst10Pct);
                    //System.out.println("worst 10% average: " + dijTimDifferenceWorst10Pct);

                    //System.out.println("least density threshold " + Routing.least_density_safe_threshold);

/*
                    //compare paths
                    File file = new File("dij" + numVehiclesBase);
                    BufferedReader reader = new BufferedReader(new FileReader(file));
                    ArrayList<Integer> dijDiffActualTripTimes = new ArrayList<>();
                    String value;
                    while ((value = reader.readLine()) != null) {
                        dijDiffActualTripTimes.add(Integer.parseInt(value));
                    }

                    ArrayList<Integer> actualTripTimes = new ArrayList<>();
                    for (Vehicle v : controller.trackedVehiclesOrdered) {
                        actualTripTimes.add(v.getActualTripTime());
                    }

                    ArrayList<Double> dijDiffTripTimes = new ArrayList<>();
                    for (int i = 0; i < controller.trackedVehiclesOrdered.size(); i++) {
                        int actualTripTime = actualTripTimes.get(i);
                        int dijTripTime = dijDiffActualTripTimes.get(i);
                        double diff = (actualTripTime - dijTripTime);
                        Vehicle v = controller.trackedVehiclesOrdered.get(i);
                        if (diff > 100) {
                            System.out.print("");
                        }
                        dijDiffTripTimes.add(diff);
                    }
                    Collections.sort(dijDiffTripTimes);
                    //System.out.println(dijDiffTripTimes);

                    double dijDiffTripTimesAvg = ListsUtil.calcAverageDouble(dijDiffTripTimes);
                    //System.out.println("mean diff: " + dijDiffTripTimesAvg); //mean difference between actual time and time when all are doing dij

                    double dijAvg = ListsUtil.calcAverageInt(dijDiffActualTripTimes);
                    //System.out.println("dij average: " + dijAvg); //average time taken when everyone does dij
                    double proportionMean = (dijDiffTripTimesAvg) / dijAvg * 100;
                    //System.out.println("proportion mean: " + proportionMean); //how much slower/faster actual is to everyone doing dij

                    double median = dijDiffTripTimes.get(dijDiffTripTimes.size() / 2);
                    //System.out.println("median: " + median); //median difference

*/


    /*
                if (numVehiclesBase <= 1000) {
                    int totalDiff = 0;
                    for (int i = 0; i < graph.getNodes().size(); i++) {
                        //String actual = actualTripTimes.get(i);
                        String dijDiff = dijDiffActualTripTimes.get(i);
                        //System.out.println("vehicle " + i + "\nactual : " + actual +"\ndijDiff: " + dijDiff);
                        //System.out.println();
                        if (!actual.equals(dijDiff)) {
                            //System.out.println("vehicle " + i + "\nactual : " + actual +"\ndijDiff: " + dijDiff);
                            //System.out.println();
                            totalDiff ++;
                        }


                    }
                    System.out.println("total: " + totalDiff);
                }

     */
    /*
                for (Vehicle vehicle : vehicles) {
                    for (Road road : vehicle.actualPath) {
                        writer.write(road.getNodesAsString() + ", ");
                    }
                    writer.write("\n");
                    writer.flush();
                }

     */
                    if (isWrite && !isLongTermRouting) {
                        //append to file
                        writer.write(routingType + ", " + numVehicles + ", " + averageSpeed + ", " + avgTimeTakenToFinishTrip + ", " + proportionOfFinishedTrips + ", "
                                + optimalDiffAverage + ", " + optimalDiffWorst10PctAvg + ", " + dijDiffAverage + ", " + dijDiffWorst10PctAvg + ","
                                + controller.getProcessingTime() + ", " + Routing.least_density_safe_threshold + "\n");
                        writer.flush();
                    }

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
                /*
                        BufferedWriter dijBWriter = new BufferedWriter( new FileWriter("dij" + numVehiclesBase, true));
                        for (Vehicle v : controller.trackedVehiclesOrdered) {
                            dijBWriter.write(v.getActualTripTime() + "\n");
                            dijBWriter.flush();
                        }
                */

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

                ArrayList<Double> vehicleUnfairnessList = controller.getVehicleUnfairnessList();
                Collections.sort(vehicleUnfairnessList);
                double avgUnfairness = ListsUtil.calcAverageDouble(vehicleUnfairnessList);

                ArrayList<Double> unfairnessWorst10PctList = ListsUtil.findWorstNPercent(0.9, vehicleUnfairnessList);
                double avgUnfairnessWorst10Pct = ListsUtil.calcAverageDouble(unfairnessWorst10PctList);


                //get worst trips
                ArrayList<Double> worstTrips = new ArrayList<>();
                for (Vehicle v : trackedVehicles) {
                    if (v.getWorstTrip() < Double.MAX_VALUE)
                        worstTrips.add(v.getWorstTrip());
                }
                double avgWorstTrip = ListsUtil.calcAverageDouble(worstTrips);

                double avgDijDiffThreshold = Routing.getAvgDijDiffThreshold();

                ArrayList<Double> dijDiffLongTermAverageList = new ArrayList<>();
                for (int i = 0; i < dijDiffLists.get(0).size(); i++) {
                    double dijDiffIndividualSum = 0;
                    for (int j = 0; j < nTrips; j++) {
                        dijDiffIndividualSum += dijDiffLists.get(j).get(i);
                    }
                    double avg = dijDiffIndividualSum / nTrips;
                    dijDiffLongTermAverageList.add(avg);
                }
                Collections.sort(dijDiffLongTermAverageList);
                double dijDiffAvg = ListsUtil.calcAverageDouble(dijDiffLongTermAverageList);
                ArrayList<Double> dijDiffWorst10PctList = ListsUtil.findWorstNPercent(0.9, dijDiffLongTermAverageList);
                double dijDiffWorst10PctAvg = ListsUtil.calcAverageDouble(dijDiffWorst10PctList);
                System.out.println("Avg unfairness " + avgUnfairness + " (Worst 10%: " + avgUnfairnessWorst10Pct + ")");
                System.out.println("avg worst trip: " + avgWorstTrip);


                //WRITE TO FILE
                if (isWrite && isLongTermRouting) {
                    writer.write(routingType + ", " + numVehiclesBase + ", " + nTrips + ", " + avgTimeTakenToFinishTrip + ", " + dijDiffAvg + ", "
                            + dijDiffWorst10PctAvg + ", " + avgUnfairness + ", " + avgUnfairnessWorst10Pct + ", " + avgWorstTrip + ", " + avgDijDiffThreshold + "\n");
                    writer.flush();
                }

                System.out.println("----------------------------\n");
            }
        }
    }
}