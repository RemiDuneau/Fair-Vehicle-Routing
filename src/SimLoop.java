import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Map;
import java.util.Stack;

public class SimLoop {

    public static int NUM_ITERATIONS = 900;
    public static boolean isPopulated = false;

    private static int incrementCount;
    public static int getIncrementCount() {
        return incrementCount;
    }

    public static void main(String[] args) {
        int routingType = Routing.TYPE_FUTURE_DIJKSTRA;
        int initial = 5;
        int numCycles = 9;
        int vehiclesIncrement = 500;
        for (int loop = initial; loop < numCycles*vehiclesIncrement+initial; loop+= vehiclesIncrement) {

            //init variables
            TimeController.NUM_VEHICLES = loop;
            Graph graph = XMLParser.parseXML(new File("StreetGridGraph.xml"));
            TimeController controller = new TimeController(loop, graph);
            controller.createRandomNodeVehicles();
            Vehicle[] vehicles = controller.getVehicles();
            ArrayList<Vehicle> trackedVehicles = controller.getTrackedVehicles();
            ArrayList<Vehicle> activeVehicles = controller.getActiveVehicles();
            Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = controller.getAllPathsMap();
            ArrayList<Road> roads = controller.getRoads();

            //find all paths from every vehicle's start node to every vehicle's end node
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
                    ArrayList<Stack<Road>> allPaths = Routing.dfsFindAllPaths(startNode, endNode);
                    allPathsMap.put(new Tuple<Node, Node>(startNode, endNode), allPaths);
                }
            }
            System.out.println("all paths map size: " + allPathsMap.size());

            System.out.println("Populating Network...");
            //POPULATE NETWORK
            isPopulated = false;
            int initIterations = 0;
            for (int i = 0; i < initIterations; i++) {
                controller.incrementTime(0,0);
            }

            isPopulated = true;

            ArrayList<ArrayList<Integer>> roadSizesSimLoop = new ArrayList<>();
            for (Road road : roads) {
                roadSizesSimLoop.add(new ArrayList<>());
            }

            //MAIN LOOP
            for (int i = 0; i < NUM_ITERATIONS; i++) {
                incrementCount = i;
                System.out.print("TIME: " + i);
                controller.incrementTime(0,0);
                if (incrementCount >= controller.ITERATION) {
                    for (int j = 0; j < roads.size(); j++) {
                        Road r = roads.get(j);
                        roadSizesSimLoop.get(j).add(r.getVehicles().size());
                    }
                }
                System.out.println(" (Active Vehicles: " + activeVehicles.size() + ")");
            }

            //--------- STATS ---------

            System.out.println();

            //vehicle/road tracking stats stuff
            int totalDistance = 0;
            int totalTimeTakenToFinishTrip = 0;
            int numFinishedTrips = 0;
            int nVehicles = trackedVehicles.size();
            for (Vehicle vehicle : trackedVehicles) {
                totalDistance += vehicle.getTotalDistance();
                if (vehicle.isFinished()) {
                    totalTimeTakenToFinishTrip += vehicle.getActualTripTime();
                    numFinishedTrips++;
                }
            }

            double proportionOfFinishedTrips = numFinishedTrips / (double) trackedVehicles.size();


            //average trips
            double avgTimeTakenToFinishTrip = totalTimeTakenToFinishTrip / ((double) nVehicles * proportionOfFinishedTrips);

            System.out.println("Average time taken to complete a trip: " + avgTimeTakenToFinishTrip + "(proportion of trips finished: " + proportionOfFinishedTrips + ")");
            double avgDistance = totalDistance / (double) nVehicles;
            System.out.println("Average speed: " + (avgDistance / avgTimeTakenToFinishTrip));

            //average density
            double averageDensitySum = 0;
            for (Road road : roads) {
                double averageDensity = road.getDensitySum() / NUM_ITERATIONS;
                System.out.println("Road " + road.getNodesAsString() + " average density: " + averageDensity);
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
                    }
                }
            }

            Collections.sort(optimalTimeDifferenceList);
            System.out.println(optimalTimeDifferenceList);
            /*
            int size = optimalTimeDifferenceList.size();
            for (int i = size/4*3; i < size; i++) {
                System.out.print(optimalTimeDifferenceList.get(i) + ", ");
            }
            System.out.println();
             */

            Collections.sort(dijkstraTimeDifferenceList);
            System.out.println(dijkstraTimeDifferenceList);
            int dijTotal = 0;
            for (double d : dijkstraTimeDifferenceList) {
                dijTotal += d;
            }
            System.out.println("avg: " + dijTotal/(double) dijkstraTimeDifferenceList.size());

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
                FileWriter fileWriter = new FileWriter("test.csv", true);
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