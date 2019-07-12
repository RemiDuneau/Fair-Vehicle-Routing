import java.lang.reflect.Array;
import java.util.*;
public class TimeController {
    public static int NUM_VEHICLES = 2500;
    public static final int SEED = 1234567890;
    public static final Random RANDOM = new Random();
    public int MAX_VEHICLES_ADDED_PER_TIME_INCREMENT = Math.max(NUM_VEHICLES/100, 1);
    public static boolean isFutureSim = false;

    public TimeController(int numVehicles, Graph graph) {
        RANDOM.setSeed(SEED);
        NUM_VEHICLES = numVehicles;
        this.nodes = graph.getNodes();
        this.roads = graph.getRoads();

        for (Road road : roads) {
            roadSizes.add(new ArrayList<>());
        }
    }

    private Vehicle[] vehicles = new Vehicle[NUM_VEHICLES];
    private ArrayList<Vehicle> inactiveVehicles = new ArrayList(NUM_VEHICLES);
    private ArrayList<Vehicle> activeVehicles = new ArrayList<>();
    private ArrayList<Vehicle> trackedVehicles = new ArrayList<>();
    private ArrayList<Node> nodes;
    private ArrayList<Road> roads;
    private Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = new HashMap<>();
    private int vehiclesSentOut = 0, vehiclesAddedThisIncrement, vehiclesCheckedThisIncrement;

    private ArrayList<Vehicle> tempInactiveVehicles = inactiveVehicles, tempActiveVehicles = activeVehicles;




    //CEHCKING WHY ITS NOT WORKING DELET LATER
    public ArrayList<ArrayList<Integer>> roadSizes = new ArrayList<>();
    public int ITERATION = 67;
    public int INCREMENT = 23;

    public ArrayList<Vehicle> vehiclesFutureSim = new ArrayList<>();
    public ArrayList<Vehicle> vehiclesNormal = new ArrayList<>();






    public void incrementTime(int vehiclesAddedThisIncrement, int totalVehiclesChecked) {
        addVehicles(vehiclesAddedThisIncrement, totalVehiclesChecked);
        moveVehicles();
        removeVehicles();
        updateSpeed();
    }

    /**
     * Attempts to add a vehicle to the graph at its start node. at most {@code MAX_VEHICLES_ADDED_PER_TIME_INCREMENT} vehicles are added.
     * Calculates the optimal time of the vehicle, and also the time taken had the routing type been dijkstra (if it is not dijkstra).
     * Sets the current speed to the speed of the road.
     * @param vehiclesAdded the number of vehicles already added within this time increment. (used during future simulations).
     *                      Generally this is set to 0.
     */
    private void addVehicles(int vehiclesAdded, int vehiclesChecked) {
        ArrayList<Vehicle> vehiclesToRemove = new ArrayList<>();

        for (Vehicle vehicle : tempInactiveVehicles) {
        vehiclesAddedThisIncrement = vehiclesAdded;
        vehiclesCheckedThisIncrement = vehiclesChecked;

            //check if node has already been assigned vehicle and the first cell on vehicle's path is free
            if (vehiclesAdded < MAX_VEHICLES_ADDED_PER_TIME_INCREMENT) {

                //get a dijkstra path if we are in a future simulation with a future algorithm to not enter a recursive loop of simulations.
                if (isFutureSim && vehicle.getRoutingType() == Routing.TYPE_FUTURE_DIJKSTRA)
                   vehicle.setPath(getPathFromRoutingType(Routing.TYPE_DIJKSTRA, vehicle.getStartNode(), vehicle.getEndNode()));

                else
                    vehicle.setPath(getPathFromRoutingType(vehicle.getRoutingType(), vehicle.getStartNode(), vehicle.getEndNode()));

                //check if path not empty
                if (vehicle.getPath().size() > 0) {
                    Road road = vehicle.getPath().peek();

                    //check if density < 1
                    if (road.getDensity() < Road.MAX_DENSITY) {

                        //set other state variables and add to activeVehicles
                        if (SimLoop.isPopulated && !isFutureSim) {

                            //calc optimal time
                            Stack<Road> optimalPath = (Routing.dijkstra(vehicle.getStartNode(), vehicle.getEndNode(), nodes, false));
                            int numRoads = optimalPath.size();
                            int length = 0;
                            int totalSpeed = 0;
                            for (Road optimalRoad : optimalPath) {
                                length += optimalRoad.getLength();
                                totalSpeed += optimalRoad.getMaxSpeed();
                            }
                            vehicle.setOptimalTripTime((int) Math.ceil(length/(totalSpeed/(double) numRoads)));

                            //future simulation using dijkstra
                            Stack<Road> path = Routing.dijkstra(vehicle.getStartNode(), vehicle.getEndNode(), nodes, true);
                            int dijTime = futureSim(vehicle, vehiclesAdded, vehiclesChecked, path);
                            vehicle.setDijkstraTripTime(dijTime);

                            vehiclesSentOut++;
                            trackedVehicles.add(vehicle);
                        }
                        tempActiveVehicles.add(vehicle);

                        //init vehicle/ update road
                        road = vehicle.getPath().pop();
                        vehicle.setCurrentRoad(road);
                        road.addVehicle(vehicle);
                        vehicle.setCurrentSpeed(road.calculateCurrentSpeed());

                        vehiclesAdded++;
                        vehiclesToRemove.add(vehicle);
                    }
                }
                vehiclesChecked++;
            }
        }

        //remove vehicles from inactiveVehicles
        for (Vehicle vehicle : vehiclesToRemove) {
            tempInactiveVehicles.remove(vehicle);
        }
    }


    private void moveVehicles() {
        for (Vehicle vehicle : tempActiveVehicles) {
            vehicle.move();
        }
    }


    private void removeVehicles() {
        ArrayList<Vehicle> vehiclesToRemove = new ArrayList<>();
        for (Vehicle vehicle : tempActiveVehicles) {
            if (vehicle.isFinished()) {
                vehiclesToRemove.add(vehicle);
                vehicle.getCurrentRoad().removeVehicle(vehicle);
                vehicle.resetRoadDistance();
                vehicle.resetTripDistance();
            }
        }

        //remove from activeVehicles
        for (Vehicle vehicle : vehiclesToRemove) {
            tempActiveVehicles.remove(vehicle);
        }
    }


    private void updateSpeed() {
        for (Road road : roads) {
            road.calculateCurrentSpeed();
            if (SimLoop.isPopulated && !isFutureSim)
                road.incrementDensitySum();
        }
    }



    public int futureSim(Vehicle vehicle, int vehiclesAddedThisIncrement, int totalVehiclesChecked, Stack<Road> path) {

        if (path.size() < 1) return Integer.MAX_VALUE;

        //enable future sim
        isFutureSim = true;

        //clone inactive vehicles
        ArrayList<Vehicle> oldInactiveVehicles = tempInactiveVehicles;

        //create modified list which doesn't contain current vehicle so it isn't copied into inactive vehicles, as it is copied separately later
        ArrayList<Vehicle> modifiedInactiveVehicles = new ArrayList<>(tempInactiveVehicles);
        for (int i = 0; i <= totalVehiclesChecked; i++) {
            if (i < tempInactiveVehicles.size()) { //avoid IndexOutOfBound
                Vehicle v = tempInactiveVehicles.get(i);
                if (tempActiveVehicles.contains(v))
                    modifiedInactiveVehicles.remove(v);
            }
        }
        modifiedInactiveVehicles.remove(vehicle);

        ArrayList<Vehicle> newInactiveVehicles = new ArrayList<>();
        for (Vehicle v : modifiedInactiveVehicles) {
            Vehicle newVehicle = (Vehicle) v.clone();
            newInactiveVehicles.add(newVehicle);
        }

        //clone activeVehicles
        ArrayList<Vehicle> oldActiveVehicles = tempActiveVehicles;
        ArrayList<Vehicle> newActiveVehicles = new ArrayList<>();
        for (Vehicle v : tempActiveVehicles) {
            Vehicle newVehicle = (Vehicle) v.clone();
            newActiveVehicles.add(newVehicle);
        }

        //change tempVehicles so incrementTime will work with new lists instead of original ones
        tempInactiveVehicles = newInactiveVehicles;
        tempActiveVehicles = newActiveVehicles;

        //temporarily clear all roads of traffic
        ArrayList<ArrayList<Vehicle>> roadVehicles = new ArrayList<>();
        for (Road r : roads) {
            roadVehicles.add(r.getVehicles());
            r.setVehicles(new ArrayList<>());
        }

        //populate roads with cloned vehicles
        for (Vehicle v : tempActiveVehicles) {
            v.getCurrentRoad().addVehicle(v);
        }

        //----- simulate future -----
        Vehicle vehicleCopy = (Vehicle) vehicle.clone();
        vehicleCopy.setPath(path);
        Road rd = path.pop();
        vehicleCopy.setCurrentRoad(rd);
        if (rd.getDensity() >= Road.MAX_DENSITY) return Integer.MAX_VALUE; //assume infinite time if first road is full
        rd.addVehicle(vehicleCopy);
        vehicleCopy.setCurrentSpeed(rd.calculateCurrentSpeed());
        tempActiveVehicles.add(vehicleCopy);

        //increment time
        incrementTime(vehiclesAddedThisIncrement + 1, totalVehiclesChecked + 1);
        int loopCount = SimLoop.getIncrementCount() + 1; //+1 because we have just incremented time once
        checkDensities(vehiclesAddedThisIncrement);

        while (!vehicleCopy.isFinished() && loopCount < SimLoop.NUM_ITERATIONS) {
            incrementTime(0, 0);
            checkDensities(vehiclesAddedThisIncrement);
            loopCount++;
        }

        //----- reset back to original -----

        //reset roads
        for (int i = 0; i < roads.size(); i++) {
            Road r = roads.get(i);
            r.setVehicles(roadVehicles.get(i));
            r.calculateCurrentSpeed();
        }

        //reset vehicle lists
        tempActiveVehicles = oldActiveVehicles;
        tempInactiveVehicles = oldInactiveVehicles;
        isFutureSim = false;

        if (vehicleCopy.isFinished()) return vehicleCopy.getActualTripTime();

        //if vehicle is not finished return how long it has travelled + how long it would take to finish trip assuming road densities do not change
        else {
            int extraTime = vehicleCopy.calculateTimeIncrementsToFinishRoad();
            for (Road road : vehicleCopy.getPath()) {
                extraTime += road.getTimeToTraverse();
            }
            return vehicleCopy.getActualTripTime() + extraTime;
        }
        //else return Integer.MAX_VALUE;
    }


    private void checkDensities(int vehiclesAdded) {
        if (SimLoop.getIncrementCount() == ITERATION && vehiclesAdded == INCREMENT) {
            for (int i = 0; i < roads.size(); i++) {
                Road r = roads.get(i);
                roadSizes.get(i).add(r.getVehicles().size());
            }
        }
    }


    public Stack<Road> getPathFromRoutingType(int routingType, Node startNode, Node endNode) {

        Stack<Road> path;
        switch (routingType) {

            case Routing.TYPE_DIJKSTRA:
                Stack<Road> pathDijkstra = Routing.dijkstra(startNode, endNode, nodes, true);
                path = pathDijkstra;
                break;

            case Routing.TYPE_FAIR:
                Stack<Road> pathFair = new Stack<>();
                pathFair.push(roads.get(3));
                pathFair.push(roads.get(1));
                path = pathFair;
                break;

            case Routing.TYPE_LEAST_DENSITY:
                Stack<Road> pathLeastDensity = Routing.leastDensity(startNode, endNode, nodes);
                path = pathLeastDensity;
                break;

            case Routing.TYPE_FUTURE_DIJKSTRA:
                Stack<Road> pathFutureTime = Routing.future(startNode, endNode, this, false);
                path = pathFutureTime;
                break;

            case Routing.TYPE_FUTURE_LEAST_DENSITY:
                Stack<Road> pathFutureDensity = Routing.future(startNode, endNode, this, true);
                path = pathFutureDensity;
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + routingType);
        }
        return path;
    }


    public void createVehicles() {
        for (int i = 0; i < NUM_VEHICLES; i++) {
            Vehicle vehicle = new Vehicle(Routing.TYPE_FUTURE_LEAST_DENSITY);
            vehicles[i] = vehicle;
            inactiveVehicles.add(vehicle);
        }

        //update 3/4 of vehicles to start at node0, and 1/4 to start at node4
        int j = 0;
        for (int i = 0; i < 0.75*NUM_VEHICLES; i++){
            vehicles[i].setStartNode(nodes.get(0));
            vehicles[i].setEndNode(nodes.get(3));
            j++;
        }
        for (int i = j; i < NUM_VEHICLES; i++){
            vehicles[i].setStartNode(nodes.get(4));
            vehicles[i].setEndNode(nodes.get(3));
        }
    }

    public void createRandomNodeVehicles() {
        for (int i = 0; i < NUM_VEHICLES; i++) {
            Vehicle vehicle = new Vehicle();
            vehicles[i] = vehicle;
            inactiveVehicles.add(vehicle);

            //randomise start and end nodes
            int startNodeId = RANDOM.nextInt(nodes.size());
            int endNodeId = RANDOM.nextInt(nodes.size());

            //ensure start and end aren't same node
            while (startNodeId == endNodeId) {
                endNodeId = RANDOM.nextInt(nodes.size());
            }

            Node startNode = null;
            Node endNode = null;
            for (Node node : nodes) {
                if (node.getId() == startNodeId) startNode = node;
                if (node.getId() == endNodeId) endNode = node;
            }
            vehicle.setStartNode(startNode);
            vehicle.setEndNode(endNode);
        }
    }

    //----------- ACCESSOR METHODS -----------


    public Vehicle[] getVehicles() {
        return vehicles;
    }

    public ArrayList<Vehicle> getInactiveVehicles() {
        return inactiveVehicles;
    }

    public ArrayList<Vehicle> getActiveVehicles() {
        return activeVehicles;
    }

    public ArrayList<Vehicle> getTrackedVehicles() {
        return trackedVehicles;
    }

    public ArrayList<Road> getRoads() {
        return roads;
    }

    public Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> getAllPathsMap() {
        return allPathsMap;
    }

    public int getVehiclesSentOut() {
        return vehiclesSentOut;
    }

    public int getVehiclesAddedThisIncrement() {
        return vehiclesAddedThisIncrement;
    }

    public int getVehiclesCheckedThisIncrement() {
        return vehiclesCheckedThisIncrement;
    }
}