import java.util.*;

public class TimeController {
    public static int NUM_VEHICLES = 2500;
    public static int NUM_ITERATIONS = 400;
    public static final int SEED = 1234567890;
    public static final Random RANDOM = new Random();
    public int MAX_VEHICLES_ADDED_PER_TIME_INCREMENT = Math.max(NUM_VEHICLES/200, 1);

    public TimeController() {
        RANDOM.setSeed(SEED);
    }


    private Vehicle[] vehicles = new Vehicle[NUM_VEHICLES];
    private ArrayList<Vehicle> inactiveVehicles = new ArrayList(NUM_VEHICLES);
    private ArrayList<Vehicle> activeVehicles = new ArrayList<>();
    private ArrayList<Node> nodes = new ArrayList<>();
    private ArrayList<Road> roads = new ArrayList<>();
    private Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = new HashMap<>();


    public void incrementTime() {
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

                    case RoutingType.TYPE_FUTURE_DIJKSTRA:
                        Stack<Road> pathFutureTime = RoutingType.future(vehicle.getStartNode(),vehicle.getEndNode(),nodes,roads,activeVehicles, allPathsMap, false);
                        vehicle.setPath(pathFutureTime);
                        break;

                    case RoutingType.TYPE_FUTURE_LEAST_DENSITY:
                        Stack<Road> pathFutureDensity = RoutingType.future(vehicle.getStartNode(),vehicle.getEndNode(),nodes,roads,activeVehicles, allPathsMap, true);
                        vehicle.setPath(pathFutureDensity);
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
            road.incrementDensitySum();
        }
    }


    public void createVehicles() {
        for (int i = 0; i < NUM_VEHICLES; i++) {
            Vehicle vehicle = new Vehicle(RoutingType.TYPE_FUTURE_LEAST_DENSITY);
            vehicles[i] = vehicle;
            inactiveVehicles.add(vehicle);
        }
    }

    public void initNetworkForSimpleExample() {

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

        //update 3/4 of vehicles to start at node0, and 1/4 to start at node4
        int j = 0;
        for (int i = 0; i < 0.75*NUM_VEHICLES; i++){
            vehicles[i].setStartNode(node0);
            vehicles[i].setEndNode(node3);
            j++;
        }
        for (int i = j; i < NUM_VEHICLES; i++){
            vehicles[i].setStartNode(node4);
            vehicles[i].setEndNode(node3);
        }
    }

    //----------- ACCESSOR METHODS


    public Vehicle[] getVehicles() {
        return vehicles;
    }

    public ArrayList<Vehicle> getInactiveVehicles() {
        return inactiveVehicles;
    }

    public ArrayList<Vehicle> getActiveVehicles() {
        return activeVehicles;
    }

    public ArrayList<Road> getRoads() {
        return roads;
    }

    public Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> getAllPathsMap() {
        return allPathsMap;
    }
}