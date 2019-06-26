import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.Stack;

public class TimeController {
    public static int NUM_VEHICLES = 2500;
    public static int NUM_ITERATIONS = 400;
    public static final int SEED = 1234567890;
    public static final Random RANDOM = new Random();
    public static final int MAX_VEHICLES_ADDED_PER_TIME_INCREMENT = 15;

    public TimeController() {
        RANDOM.setSeed(SEED);
    }


    private Vehicle[] vehicles = new Vehicle[NUM_VEHICLES];
    private ArrayList<Vehicle> inactiveVehicles = new ArrayList(NUM_VEHICLES);
    private ArrayList<Vehicle> activeVehicles = new ArrayList<>();
    private ArrayList<Node> nodes = new ArrayList<>();
    private ArrayList<Road> roads = new ArrayList<>();


    public static void main(String[] args) {
        TimeController controller = new TimeController();
        controller.createVehicles();
        controller.initNetworkForSimpleExample();

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            System.out.println("----------------------\n" +
                    "TIME: " + i);
            controller.incrementTime();
            System.out.println( "(Active Vehicles: " + controller.activeVehicles.size() + ")");
            for (Road road : controller.roads) {
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
        int nVehicles = controller.vehicles.length;
        for (Vehicle vehicle : controller.vehicles) {
            totalDistance += vehicle.getTotalDistance();
            totalTripsFinished += vehicle.getTripsFinished();
        }
        double avgDistance = totalDistance / (double) nVehicles;

        System.out.println("Average distance travelled by vehicles per time iteration: " + (avgDistance/(double) NUM_ITERATIONS));

        //average trips
        double avgTripsFinished = totalTripsFinished / (double) nVehicles;

        //calc standard deviation
        int varianceTopBit = 0;
        for (Vehicle vehicle : controller.vehicles) {
            varianceTopBit += Math.pow(vehicle.getTripsFinished() - avgTripsFinished, 2);
        }
        double tripsCompletedStdDev = Math.sqrt(varianceTopBit/(double) (nVehicles-1));

        System.out.println("Average number of trips completed: " + (avgTripsFinished) + " (standard deviation: " + tripsCompletedStdDev + ")");

        for (Road road : controller.roads) {
            System.out.println("Road " + road.getNodesAsString() + " total vehicles added: " + road.getTotalVehiclesAdded());
        }
    }

    private void incrementTime() {
        addVehicle();
        moveVehicles();
        removeVehicles();
        updateSpeed();
    }

    /**
     * Attempts to add a vehicle to the graph at its start node. For every node, at most one vehicle is added.
     * Sets the current speed to the maximum speed of the road
     */
    private void addVehicle() {
        HashMap<Node, Integer> assignedNodes = new HashMap<>();
        ArrayList<Vehicle> vehiclesToRemove = new ArrayList<>();
        for (Vehicle vehicle : inactiveVehicles) {

            //check if node has already been assigned vehicle and the first cell on vehicle's path is free
            if (!assignedNodes.containsKey(vehicle.getStartNode()) || assignedNodes.get(vehicle.getStartNode()) < MAX_VEHICLES_ADDED_PER_TIME_INCREMENT) {

                //set path
                switch (vehicle.getRoutingType()) {

                    case RoutingType.TYPE_DIJKSTRA:
                        Stack<Road> pathDijkstra = RoutingType.dijkstra(vehicle.getStartNode(), vehicle.getEndNode(), nodes);
                        vehicle.setPath(pathDijkstra);
                        break;

                    case RoutingType.TYPE_FAIR:
                        Stack<Road> pathFair = new Stack<>();
                        pathFair.push(roads.get(3));
                        pathFair.push(roads.get(1));
                        vehicle.setPath(pathFair);
                        break;

                    case RoutingType.TYPE_LEAST_DENSITY:
                        Stack<Road> pathLeastDensity = RoutingType.leastDensity(vehicle.getStartNode(), vehicle.getEndNode(), nodes);
                        vehicle.setPath(pathLeastDensity);
                        break;

                    case RoutingType.TYPE_GREEDY:
                        Stack<Road> pathGreedy = RoutingType.greedy(vehicle.getStartNode(),vehicle.getEndNode(),nodes,roads,activeVehicles);
                        vehicle.setPath(pathGreedy);
                        break;

                    default:
                        throw new IllegalStateException("Unexpected value: " + vehicle.getRoutingType());
                }

                //check if density < 1
                Road road = vehicle.getPath().peek();
                if (road.getDensity() < 1) {

                    //set other state variables and add to activeVehicles
                    activeVehicles.add(vehicle);
                    road = vehicle.getPath().pop();
                    vehicle.setCurrentRoad(road);
                    road.addVehicle(vehicle);
                    vehicle.setCurrentSpeed(road.calculateCurrentSpeed());
                    vehiclesToRemove.add(vehicle);

                    //increment nodes
                    Node node = vehicle.getStartNode();
                    if (assignedNodes.containsKey(node)) {
                        assignedNodes.put(node, assignedNodes.get(node) + 1);
                    } else assignedNodes.put(node, 1);
                }
            }
        }
        for (Vehicle vehicle : vehiclesToRemove) {
            inactiveVehicles.remove(vehicle);
        }
    }

    private void moveVehicles() {
        for (Vehicle vehicle : activeVehicles) {
            vehicle.move();
        }
    }

    private void removeVehicles() {
        ArrayList<Vehicle> vehiclesToRemove = new ArrayList<>();
        for (Vehicle vehicle : activeVehicles) {
            if (vehicle.isFinished()) {
                vehicle.setFinished(false);
                vehiclesToRemove.add(vehicle);
                inactiveVehicles.add(vehicle);
                vehicle.getCurrentRoad().removeVehicle(vehicle);
                vehicle.resetRoadDistance();
                vehicle.resetTripDistance();
            }
        }

        //remove from activeVehicles
        for (Vehicle vehicle : vehiclesToRemove) {
            activeVehicles.remove(vehicle);
        }
    }

    private void updateSpeed() {
        for (Road road : roads) {
            road.calculateCurrentSpeed();
        }
    }


    private void createVehicles() {
        for (int i = 0; i < NUM_VEHICLES; i++) {
            Vehicle vehicle = new Vehicle(RoutingType.TYPE_GREEDY);
            vehicles[i] = vehicle;
            inactiveVehicles.add(vehicle);
        }
    }

    private void initNetworkForSimpleExample() {

        /*------- NETWORK -------
                     L
                0 ------- 1
                | \       |
              M |   \M    |L
                |     \   |
                |       \ |
        4 ----- 2 ------- 3
            S        L
        */

        final int veryLargeRoad = 10000;
        final int largeRoad = 4000;
        final int mediumRoad = 3000;
        final int smallRoad = 2000;
        final int maxSpeed = 70;

        Node node0 = new Node(0);
        Node node1 = new Node(1);
        Node node2 = new Node(2);
        Node node3 = new Node(3);
        Node node4 = new Node(4);

        Road road03A = new Road(node0, node3, mediumRoad, maxSpeed);
        Road road01 = new Road(node0, node1, largeRoad, maxSpeed);
        Road road02 = new Road(node0, node2, mediumRoad, maxSpeed);
        Road road13 = new Road(node1, node3, largeRoad, maxSpeed);
        Road road23 = new Road(node2, node3, largeRoad, maxSpeed);
        Road road42 = new Road(node4, node2, smallRoad, maxSpeed);

        node0.addNeighbour(node3, road03A);
        node0.addNeighbour(node1, road01);
        node0.addNeighbour(node2, road02);
        node1.addNeighbour(node3, road13);
        node2.addNeighbour(node3, road23);
        node4.addNeighbour(node2, road42);

        roads.add(road03A);
        roads.add(road01);
        roads.add(road02);
        roads.add(road13);
        roads.add(road23);
        roads.add(road42);

        nodes.add(node0);
        nodes.add(node1);
        nodes.add(node2);
        nodes.add(node3);
        nodes.add(node4);

        //update 1500 vehicles with start node 0 and end node 3, 1000 vehicles with start node 4 end node 3.
        for (int i = 0; i < 1500; i++){
            vehicles[i].setStartNode(node0);
            vehicles[i].setEndNode(node3);
        }
        for (int i = 1500; i < 2500; i++){
            vehicles[i].setStartNode(node4);
            vehicles[i].setEndNode(node3);
            vehicles[i].setRoutingType(RoutingType.TYPE_DIJKSTRA);
        }
    }
}