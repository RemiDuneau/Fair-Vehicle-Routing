import java.io.*;
import java.util.*;

public class SimLoop {

    public static int NUM_ITERATIONS = 100, INIT_ITERATIONS = 50;
    private static int LONG_TERM_ROUTING_NUM_TRIPS = 14;
    public static double DIJKSTRA_ONLY_PROPORTION = 0.1;
    public static boolean isTrackingVehicles = false;



    private static int incrementCount;
    public static int getIncrementCount() {
        return incrementCount;
    }


    public static void main(String[] args) throws IOException {
        for (double proportion = 0.1; proportion < 1; proportion+=0.2) {
            //FileWriter fileWriter = new FileWriter("longTermDijOnly" + (int) ((proportion*100 + 0.1)) + "scatter.csv", true);
            FileWriter fileWriter = new FileWriter("longTermDijOnlyRoadDensityCheck.csv", true);
            BufferedWriter writer = new BufferedWriter(fileWriter);
            boolean isWrite = true;
            int[] routingTypes = {0};

            DIJKSTRA_ONLY_PROPORTION = proportion;
            //int routingType = routingTypes[type];
            int routingType = routingTypes[0];
            int initial = 500;
            int numCycles = 8;
            int vehiclesIncrement = 250;
            for (int numVehiclesBase = initial; numVehiclesBase < numCycles * vehiclesIncrement + initial; numVehiclesBase += vehiclesIncrement) {
                System.out.println("TRACKED VEHICLES: " + numVehiclesBase);

                //init variables
                int numVehicles = numVehiclesBase * 5;
                TimeController.NUM_VEHICLES = (numVehicles);
                Graph graph = XMLParser.parseXML(new File("Berlin Example.xml"));
                TimeController controller = new TimeController(graph);
                controller.createRandomNodeVehicles(true);
                Vehicle[] vehicles = controller.getVehicles();
                ArrayList<Vehicle> allTrackedVehicles = controller.getAllTrackedVehicles(),
                        trackedVehicles = controller.getTrackedVehicles(),
                        dijkstraOnlyTrackedVehicles = controller.getDijkstraOnlyTrackedVehicles(),
                        activeVehicles = controller.getActiveVehicles(),
                        inactiveVehicles = controller.getInactiveVehicles();
                Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = controller.getAllPathsMap();
                ArrayList<Road> roads = controller.getRoads();
                ArrayList<ArrayList<Double>> dijDiffLists = new ArrayList();

                controller.setLongTermRouting(Routing.isLongTermRouting);
                Routing.resetDijkstra_diff_thresholdStats();

                //find all paths from every vehicle's start node to end node
                for (Vehicle v : vehicles) {
                    if (v.isDijkstraOnly()) v.setRoutingType(Routing.TYPE_DIJKSTRA);
                    else v.setRoutingType(routingType);

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

                double longTermTimeToFinishTrip = 0;
                double dijOnlyLongTermTimeToFinishTrip = 0;
                ArrayList<Double> longTermDensities = new ArrayList<>();
                for (Road rd : roads) {
                    longTermDensities.add(0.0);
                }

                int nTrips = 1;
                if (Routing.isLongTermRouting) nTrips = LONG_TERM_ROUTING_NUM_TRIPS;
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
                    //for (int i = 0; i < NUM_ITERATIONS; i++) {
                    int t = 0;
                    boolean bAllStarted = false;
                    while (!bAllStarted) {
                        incrementCount = t;
                        //if (day == 2 && i == 99) {
                        //    System.out.print("");
                        //}
                        //System.out.print("TIME: " + t);
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

                        int activeTrackedVehicles = 0;
                        bAllStarted = true;
                        for (Vehicle v : allTrackedVehicles) {
                            if (v.isStarted()) activeTrackedVehicles++;
                            else bAllStarted = false;
                        }

                        //System.out.println(" (Active Vehicles: " + activeVehicles.size() + ") (Inactive Vehicles: " + inactiveVehicles.size() + ")  (Tracked Vehicles: " + activeTrackedVehicles + ")" + " (Avg Density: " + totalDensityPerIteration / (double) roads.size()
                        //        + ") (DijDiff Threshold: " + Routing.dijkstra_diff_threshold + ") (Avg unfairness: " + ListsUtil.calcAverageDouble(controller.getTrackedVehicleUnfairnessList()) + ")");
                        t++;
                    }

                    //keep incrementing time until all tracked vehicles are finished
                    isTrackingVehicles = false;
                    boolean isAllVehiclesFinished = false;
                    ArrayList<Vehicle> unfinishedVehicles = new ArrayList<>();
                    while (!isAllVehiclesFinished) {
                        controller.incrementTime();

                        //check if all vehicles are finished
                        boolean isFinished = true;
                        isFinishedLoop:
                        for (Vehicle v : allTrackedVehicles) {
                            if (!v.isFinished()) {
                                unfinishedVehicles.add(v);
                                isFinished = false;
                                break isFinishedLoop;
                            }
                        }
                        isAllVehiclesFinished = isFinished;
                        unfinishedVehicles.clear();
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
                    double avgTimeTakenToFinishTrip = totalTimeTakenToFinishTrip / (double) numFinishedTrips;
                    longTermTimeToFinishTrip += avgTimeTakenToFinishTrip;

                    System.out.println("Average time taken to complete a trip: " + avgTimeTakenToFinishTrip + "(proportion of trips finished: " + proportionOfFinishedTrips + ")");
                    double avgDistance = totalDistance / (double) nTrackedVehicles;
                    double averageSpeed = avgDistance / (totalTime / (double) nTrackedVehicles);
                    //System.out.println("Average speed: " + (averageSpeed));

                    //average density
                    double averageDensitySum = 0;
                    for (int i = 0; i < roads.size(); i++) {
                        double add = roads.get(i).getDensitySum() / (double) controller.getTrackedTimeIncrements();
                        longTermDensities.set(i, longTermDensities.get(i) + add);
                    }
                    //System.out.println("Total average density = " + totalAverageDensity);

                    //System.out.println("Vehicles sent out: " + trackedVehicles.size());

                    //DIJ ONLY TIME STUFF
                    double dijOnlyTimeTaken = 0;
                    int dijOnlyNumFinishedTrips = 0;
                    for (Vehicle v : dijkstraOnlyTrackedVehicles) {
                        if (v.isFinished()) {
                            dijOnlyTimeTaken += v.getActualTripTime();
                            dijOnlyNumFinishedTrips ++;
                        }
                    }
                    dijOnlyLongTermTimeToFinishTrip += dijOnlyTimeTaken / (double) dijOnlyNumFinishedTrips;

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

                    ArrayList<Double> dijTimDifferenceWorst10Pct = (ArrayList<Double>) ListsUtil.findWorstNPercent(0.9, dijkstraTimeDifferenceList);
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
                    for (Vehicle v : controller.allTrackedVehicles) {
                        actualTripTimes.add(v.getActualTripTime());
                    }

                    ArrayList<Double> dijDiffTripTimes = new ArrayList<>();
                    for (int i = 0; i < controller.allTrackedVehicles.size(); i++) {
                        int actualTripTime = actualTripTimes.get(i);
                        int dijTripTime = dijDiffActualTripTimes.get(i);
                        double diff = (actualTripTime - dijTripTime);
                        Vehicle v = controller.allTrackedVehicles.get(i);
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
                    if (isWrite && !Routing.isLongTermRouting) {
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
                        for (Vehicle v : controller.allTrackedVehicles) {
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

                ArrayList<Double> vehicleUnfairnessList = controller.getTrackedVehicleUnfairnessList();
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

                double longTermAvgTimeToFinishTrip = longTermTimeToFinishTrip/(double)nTrips;
                System.out.println("Avg time to finish trip: " + longTermAvgTimeToFinishTrip);
                int avgTotal = 0;
                for (Vehicle v : dijkstraOnlyTrackedVehicles) {
                    avgTotal += v.getAverageTripTime();
                }
                double dijOnlyAverageTripTime = avgTotal/ (double) dijkstraOnlyTrackedVehicles.size();
                //double dijOnlyAverageTripTime = totalTime / (double) (dijkstraOnlyTrackedVehicles.size() * nTrips);

                System.out.println("Average DIJ ONLY time: " + dijOnlyAverageTripTime);
                System.out.println("Avg unfairness " + avgUnfairness + " (Worst 10%: " + avgUnfairnessWorst10Pct + ")");
                System.out.println("avg worst trip: " + avgWorstTrip);

                System.out.println(trackedVehicles.size());
                System.out.println(dijkstraOnlyTrackedVehicles.size());

                //WRITE TO FILE
                if (isWrite && Routing.isLongTermRouting) {
                    writer.write(routingType + ", " + trackedVehicles.size() + ", " + nTrips + ", " + longTermAvgTimeToFinishTrip + ", " + dijOnlyAverageTripTime + ", "
                            + dijDiffAvg + ", " + dijDiffWorst10PctAvg + ", " + avgUnfairness + ", " + avgUnfairnessWorst10Pct + ", " + avgWorstTrip + ", "
                            + avgDijDiffThreshold + ", " + DIJKSTRA_ONLY_PROPORTION + "\n");
                    writer.flush();
                }

                writer.write("\n\n\n");
                writer.flush();


                //init maps
                Map<Road, Integer> roadFrequenciesDijOnlyMap = new HashMap<>();
                Map<Road, Integer> roadFrequenciesTrackedMap = new HashMap<>();
                for (Road rd : roads) {
                    roadFrequenciesDijOnlyMap.put(rd, 0);
                    roadFrequenciesTrackedMap.put(rd, 0);
                }

                //add vehicle frequencies to maps
                for (Vehicle v : dijkstraOnlyTrackedVehicles) {
                    Map<Road, Integer> roadsMap = v.getRoadsTakenMap();
                    for (Road rd : roadsMap.keySet()) {
                        roadFrequenciesDijOnlyMap.put(rd, roadFrequenciesDijOnlyMap.get(rd) + roadsMap.get(rd));
                    }
                }
                for (Vehicle v : trackedVehicles) {
                    Map<Road, Integer> roadsMap = v.getRoadsTakenMap();
                    for (Road rd : roadsMap.keySet()) {
                        roadFrequenciesTrackedMap.put(rd, roadFrequenciesTrackedMap.get(rd) + roadsMap.get(rd));
                    }
                }

                //get average
                Map<Road, Double> averageRoadFrequenciesDijOnlyMap = new HashMap<>();
                Map<Road, Double> averageRoadFrequenciesTrackedMap = new HashMap<>();
                for (Road rd : roads) {
                    averageRoadFrequenciesDijOnlyMap.put(rd, roadFrequenciesDijOnlyMap.get(rd)/(double)(nTrips*dijkstraOnlyTrackedVehicles.size()));
                    averageRoadFrequenciesTrackedMap.put(rd, roadFrequenciesTrackedMap.get(rd)/(double)(nTrips*trackedVehicles.size()));
                }

                //write to file
                for (int i = 0; i < roads.size(); i++) {
                    Double d = longTermDensities.get(i);
                    Road rd = roads.get(i);
                    d = d/nTrips;
                    String toWrite = rd.getNodesAsString() + ", " + d + ", " + averageRoadFrequenciesTrackedMap.get(rd) + ", " + averageRoadFrequenciesDijOnlyMap.get(rd) + "\n";
                    System.out.print(toWrite);
                    writer.write(toWrite);
                    writer.flush();
                }
/*
                for (Vehicle v : trackedVehicles) {
                    writer.write(v.getAverageTripTime() + ", " + v.getOptimalTripTime() + "\n");
                }
                writer.flush();

                for (Vehicle v : dijkstraOnlyTrackedVehicles) {
                    writer.write(v.getAverageTripTime() + ", " + v.getOptimalTripTime() + "\n");
                }
                writer.write("\n\n\n\n\n");
                writer.flush();

 */

                System.out.println("----------------------------\n");
            }
        }
    }
}