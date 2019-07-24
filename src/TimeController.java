import java.util.*;
public class TimeController {
    public static int NUM_VEHICLES;
    public static final int SEED = 1234567890;
    public static final Random RANDOM = new Random();
    public int MAX_VEHICLES_ADDED_PER_TIME_INCREMENT = Math.max(NUM_VEHICLES / 1000, 1);

    public static boolean isFutureSim = false;
    private static int futureSimCounter = 0;

    public TimeController(Graph graph) {
        RANDOM.setSeed(SEED);
        this.nodes = graph.getNodes();
        this.roads = graph.getRoads();

        for (Road road : roads) {
            roadSizes.add(new ArrayList<>());
        }
    }

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
    private double processingTime = 0;

    private ArrayList<Vehicle> tempInactiveVehicles = inactiveVehicles, tempActiveVehicles = activeVehicles;
    private Vehicle vehicleBeingAdded;


    //CEHCKING WHY ITS NOT WORKING DELET LATER
    public ArrayList<ArrayList<Integer>> roadSizes = new ArrayList<>();
    public int INCREMENT = 6;
    public int ADDEDVEHICLES = 1;

    public ArrayList<Vehicle> vehiclesFutureSim = new ArrayList<>();
    public ArrayList<Vehicle> vehiclesNormal = new ArrayList<>();


    public void incrementTime() {
        incrementTime(0,0);
    }

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
     *
     * @param vehiclesAdded the number of vehicles already added within this time increment. (used during future simulations).
     *                      Generally this is set to 0.
     */
    private void addVehicles(int vehiclesAdded, int vehiclesChecked) {
        ArrayList<Vehicle> vehiclesToRemove = new ArrayList<>();

        for (Vehicle vehicle : tempInactiveVehicles) {
            vehiclesAddedThisIncrement = vehiclesAdded;
            vehiclesCheckedThisIncrement = vehiclesChecked;
            vehicleBeingAdded = vehicle;

            //check if node has already been assigned vehicle and the first cell on vehicle's path is free
            if (vehiclesAdded < MAX_VEHICLES_ADDED_PER_TIME_INCREMENT) {

                // if we are in a future simulation with a future algorithm don't add any vehicles.
                if (isFutureSim && vehicle.getRoutingType() == Routing.TYPE_FUTURE_FASTEST)
                    vehicle.setPath(new Stack<>());

                else if (!isFutureSim) {
                    double startTime = System.nanoTime();
                    vehicle.setPath(getPathFromRoutingType(vehicle.getRoutingType(), vehicle.getStartNode(), vehicle.getEndNode()));
                    double timeTaken = System.nanoTime() - startTime;
                    processingTime += timeTaken / 1000000.0; //get processingTime in ms

                    //if the path is empty and routing type is TYPE_LEAST_DENSITY_SAFE then use LEAST_DENSITY instead
                    if (vehicle.getRoutingType() == Routing.TYPE_LEAST_DENSITY_SAFE && vehicle.getPath().size() == 0) {
                        vehicle.setPath(getPathFromRoutingType(Routing.TYPE_LEAST_DENSITY, vehicle.getStartNode(), vehicle.getEndNode()));
                    }
                }

                else vehicle.setPath(getPathFromRoutingType(vehicle.getRoutingType(), vehicle.getStartNode(), vehicle.getEndNode()));

                //check if path not empty
                if (vehicle.getPath().size() > 0) {
                    Road road = vehicle.getPath().peek();
                    vehicle.actualPath = (Stack<Road>) vehicle.getPath().clone();

                    //check if density < 1
                    if (road.getDensity() < Road.MAX_DENSITY) {

                        //set other state variables and add to activeVehicles
                        if (SimLoop.isTrackingVehicles && !isFutureSim) {

                            //calc optimal time
                            Stack<Road> optimalPath = getPathFromRoutingType(Routing.TYPE_DIJKSTRA_NO_CONGESTION, vehicle.getStartNode(), vehicle.getEndNode());
                            vehicle.optimalPath = (Stack<Road>) optimalPath.clone();
                            double tripTime = 0;
                            double remainder = 0.0;
                            for (Road optimalRoad : optimalPath) {
                                double timeToTraverse = optimalRoad.getTimeToTraverseNoCongestion(remainder);
                                tripTime += timeToTraverse;
                                remainder = 1 - (timeToTraverse % 1);
                            }

                            vehicle.setOptimalTripTime((int) Math.ceil(tripTime));

                            //future simulation using dijkstra
                            Stack<Road> path = getPathFromRoutingType(Routing.TYPE_DIJKSTRA, vehicle.getStartNode(), vehicle.getEndNode());
                            vehicle.dijkstraPath = (Stack<Road>) path.clone();
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
            if (SimLoop.isTrackingVehicles && !isFutureSim)
                road.incrementDensitySum();
        }
    }


    public int futureSim(Vehicle vehicle, int vehiclesAddedThisIncrement, int totalVehiclesChecked, Stack<Road> path) {

        if (path.size() < 1) return Integer.MAX_VALUE;

        //enable future sim
        enableFutureSim();

        //clone inactive vehicles
        ArrayList<Vehicle> oldInactiveVehicles = tempInactiveVehicles;

        //create modified list which only contains vehicles not visited by addVehicle yet
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
        if (rd.getDensity() >= Road.MAX_DENSITY)  {
            tempActiveVehicles = oldActiveVehicles;
            tempInactiveVehicles = oldInactiveVehicles;
            disableFutureSim();
            return Integer.MAX_VALUE; //assume infinite time if first road is full
        }
        rd.addVehicle(vehicleCopy);
        vehicleCopy.setCurrentSpeed(rd.calculateCurrentSpeed());
        tempActiveVehicles.add(vehicleCopy);

        //increment time
        incrementTime(vehiclesAddedThisIncrement + 1, totalVehiclesChecked + 1); //+1 because we have just added a new vehicle (vehicleCopy)
        int loopCount = SimLoop.getIncrementCount() + 1; //+1 because we have just incremented time once
        //checkDensities(vehiclesAddedThisIncrement);

        while (!vehicleCopy.isFinished()) {
            incrementTime();
            //checkDensities(vehiclesAddedThisIncrement);
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
        disableFutureSim();

        if (vehicleCopy.isFinished()) return vehicleCopy.getActualTripTime();

        //if vehicle is not finished return how long it has travelled + how long it would take to finish trip assuming road densities do not change
        else {
            double extraTime = vehicleCopy.calculateTimeIncrementsToFinishRoad();
            double remainder = 0.0;
            for (Road road : vehicleCopy.getPath()) {
                double timeToTraverse = road.getTimeToTraverse(remainder);
                extraTime += timeToTraverse;
                remainder = timeToTraverse % 1;
            }
            return vehicleCopy.getActualTripTime() + (int) Math.ceil(extraTime);
        }
        //else return Integer.MAX_VALUE;
    }


    private void checkDensities(int vehiclesAdded) {
        if (SimLoop.getIncrementCount() == ADDEDVEHICLES && vehiclesAdded == INCREMENT) {
            for (int i = 0; i < roads.size(); i++) {
                Road r = roads.get(i);
                roadSizes.get(i).add(r.getVehicles().size());
            }
        }
    }


    public Stack<Road> getPathFromRoutingType(int routingType, Node startNode, Node endNode) {

        Stack<Road> path;
        switch (routingType) {
            case Routing.TYPE_FUTURE_FASTEST:
                Stack<Road> pathFutureTime = Routing.future(startNode, endNode, this, false);
                path = pathFutureTime;
                break;

            case Routing.TYPE_FUTURE_LEAST_DENSITY:
                Stack<Road> pathFutureDensity = Routing.future(startNode, endNode, this, true);
                path = pathFutureDensity;
                break;

            case Routing.TYPE_NULL:
                throw new IllegalStateException("Unexpected value: " + routingType + ". This routing type cannot be assigned a path.");

            default:
                path = Routing.dijkstraGeneral(startNode, endNode, nodes, routingType);
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
        for (int i = 0; i < 0.75 * NUM_VEHICLES; i++) {
            vehicles[i].setStartNode(nodes.get(0));
            vehicles[i].setEndNode(nodes.get(3));
            j++;
        }
        for (int i = j; i < NUM_VEHICLES; i++) {
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

    public Vehicle getVehicleBeingAdded() {
        return vehicleBeingAdded;
    }


    public void enableFutureSim() {
        futureSimCounter++;
        isFutureSim = true;
    }

    public void disableFutureSim() {
        futureSimCounter--;
        if (futureSimCounter == 0) isFutureSim = false;
    }

    public double getProcessingTime() {
        return processingTime;
    }
}