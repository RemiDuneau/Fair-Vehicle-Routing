import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;
import java.util.Stack;

public class SimLoop {

    public static int NUM_ITERATIONS = 400;
    public static boolean isPopulated = false;

    public static void main(String[] args) {
        int routingType = Routing.TYPE_DIJKSTRA;
        int initial = 1000;
        int numCycles = 9;
        int vehiclesIncrement = 1000;
        for (int loop = initial; loop < numCycles*vehiclesIncrement+initial; loop+= vehiclesIncrement) {
            //init variables
            TimeController.NUM_VEHICLES = loop;
            Graph graph = XMLParser.parseXML(new File("Berlin Example.xml"));
            Graph.calculateConnectedNodes(graph);
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

            //POPULATE NETWORK
            isPopulated = false;
            int initIterations = 0;
            for (int i = 0; i < initIterations; i++) {
                controller.incrementTime();
            }

            isPopulated = true;

            for (int i = 0; i < NUM_ITERATIONS; i++) {
                System.out.println("----------------------\n" +
                        "TIME: " + i);
                controller.incrementTime();
                System.out.println("(Active Vehicles: " + activeVehicles.size() + ")");
                /*
                for (Road road : roads) {
                    int totalSpeed = 0;
                    for (Vehicle vehicle : road.getVehicles()) {
                        totalSpeed += vehicle.getCurrentSpeed();
                    }
                    double avgSpeed = (double) totalSpeed / (double) road.getVehicles().size();
                    System.out.println("ROAD: " + road.getNodesAsString()
                            + "; Density = " + road.getDensity() + " (" + road.getVehicles().size() + " vehicles, avg speed: " + avgSpeed + ", time: " + road.getTimeToTraverse() + ")");
                }
            */
            }


            //vehicle/road tracking stats stuff
            int totalDistance = 0;
            int totalTimeTakenToFinishTrip = 0;
            int numFinishedTrips = 0;
            int nVehicles = trackedVehicles.size();
            for (Vehicle vehicle : trackedVehicles) {
                if (vehicle.isFinished()) {
                    totalDistance += vehicle.getTotalDistance();
                    totalTimeTakenToFinishTrip += vehicle.getActualTripTime();
                    numFinishedTrips++;
                }
            }
            double avgDistance = totalDistance / (double) nVehicles;

            double proportionOfFinishedTrips = numFinishedTrips / (double) trackedVehicles.size();

            System.out.println("Average distance travelled by vehicles per time iteration: " + (avgDistance / (double) NUM_ITERATIONS));

            //average trips
            double avgTimeTakenToFinishTrip = totalTimeTakenToFinishTrip / ((double) nVehicles * proportionOfFinishedTrips);

            System.out.println("Average time taken to complete a trip: " + avgTimeTakenToFinishTrip + "(proportion of trips finished: " + proportionOfFinishedTrips + ")");
            for (Road road : roads) {
                System.out.println("Road " + road.getNodesAsString() + " total vehicles added: " + road.getTotalVehiclesAdded());
            }

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