import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;
import java.util.Stack;

public class SimLoop {
    //TODO: change least density to be average!!! not cumulative...
    public static void main(String[] args) {
        int routingType = RoutingType.TYPE_LEAST_DENSITY;
        int initial = 500;
        int numCycles = 10;
        int vehiclesIncrement = 500;
        for (int loop = initial; loop < numCycles*vehiclesIncrement+initial; loop+= vehiclesIncrement) {
            //init variables
            TimeController.NUM_VEHICLES = loop;
            TimeController controller = new TimeController();
            controller.createVehicles();
            controller.initNetworkForSimpleExample();
            Vehicle[] vehicles = controller.getVehicles();
            ArrayList<Vehicle> activeVehicles = controller.getActiveVehicles();
            Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = controller.getAllPathsMap();
            ArrayList<Road> roads = controller.getRoads();

            //find all paths every vehicle's start node to every vehicle's end node
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
                    ArrayList<Stack<Road>> allPaths = RoutingType.dfsFindAllPaths(startNode, endNode);
                    allPathsMap.put(new Tuple<Node, Node>(startNode, endNode), allPaths);
                }
            }
            System.out.println(allPathsMap.size());


            for (int i = 0; i < TimeController.NUM_ITERATIONS; i++) {
                System.out.println("----------------------\n" +
                        "TIME: " + i);
                controller.incrementTime();
                System.out.println("(Active Vehicles: " + activeVehicles.size() + ")");
                for (Road road : roads) {
                    int totalSpeed = 0;
                    for (Vehicle vehicle : road.getVehicles()) {
                        totalSpeed += vehicle.getCurrentSpeed();
                    }
                    double avgSpeed = (double) totalSpeed / (double) road.getVehicles().size();
                    System.out.println("ROAD: " + road.getNodesAsString()
                            + "; Density = " + road.getDensity() + " (" + road.getVehicles().size() + " vehicles, avg speed: " + avgSpeed + ", time: " + road.getTimeToTraverse() + ")");
                }
            }

            //vehicle/road tracking stats stuff
            int totalDistance = 0;
            int totalTripsFinished = 0;
            int nVehicles = vehicles.length;
            for (Vehicle vehicle : vehicles) {
                totalDistance += vehicle.getTotalDistance();
                totalTripsFinished += vehicle.getTripsFinished();
            }
            double avgDistance = totalDistance / (double) nVehicles;

            System.out.println("Average distance travelled by vehicles per time iteration: " + (avgDistance / (double) TimeController.NUM_ITERATIONS));

            //average trips
            double avgTripsFinished = totalTripsFinished / (double) nVehicles;

            //calc standard deviation
            int varianceTopBit = 0;
            for (Vehicle vehicle : vehicles) {
                varianceTopBit += Math.pow(vehicle.getTripsFinished() - avgTripsFinished, 2);
            }
            double tripsCompletedStdDev = Math.sqrt(varianceTopBit / (double) (nVehicles - 1));

            System.out.println("Average number of trips completed: " + (avgTripsFinished) + " (standard deviation: " + tripsCompletedStdDev + ")");

            for (Road road : roads) {
                System.out.println("Road " + road.getNodesAsString() + " total vehicles added: " + road.getTotalVehiclesAdded());
            }

            //average density
            double averageDensitySum = 0;
            for (Road road : roads) {
                double averageDensity = road.getDensitySum() / TimeController.NUM_ITERATIONS;
                System.out.println("Road " + road.getNodesAsString() + " average density: " + averageDensity);
                averageDensitySum += averageDensity;
            }
            double totalAverageDensity = averageDensitySum / roads.size();
            System.out.println("Total average density = " + totalAverageDensity);

            System.out.println("Vehicles sent out: " + controller.getVehiclesSentOut());


            //write/append to file
            try {
                FileWriter fileWriter = new FileWriter("test.csv", true);
                BufferedWriter writer = new BufferedWriter(fileWriter);
                writer.write(TimeController.NUM_VEHICLES + ", " + routingType + ", " + avgTripsFinished + ", "
                        + tripsCompletedStdDev + ", " + totalAverageDensity + ", " + controller.getVehiclesSentOut() + "\n");
                writer.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }

        }
    }
}