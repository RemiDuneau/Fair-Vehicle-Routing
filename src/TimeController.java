import java.util.*;
public class TimeController {
    public static int NUM_VEHICLES;
    public static final int SEED = 1234567890;
    public static Random RANDOM = new Random();
    public int MAX_VEHICLES_ADDED_PER_TIME_INCREMENT = Math.max(NUM_VEHICLES / 500, 1);

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
    private ArrayList<Vehicle> inactiveVehicles = new ArrayList(NUM_VEHICLES),
            activeVehicles = new ArrayList<>(),
            trackedVehicles = new ArrayList<>(),
            dijkstraOnlyTrackedVehicles = new ArrayList<>(),
            unfairnessTrackedVehicles = new ArrayList<>();
    private ArrayList<Node> nodes;
    private ArrayList<Road> roads;
    private Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPathsMap = new HashMap<>();
    private int vehiclesSentOut = 0, vehiclesAddedThisIncrement, vehiclesCheckedThisIncrement;
    private double processingTime = 0;

    private boolean isLongTermRouting;
    private ArrayList<Double> trackedVehicleUnfairnessList = new ArrayList<>();
    private Map<Vehicle, Double> vehicleUnfairnessMap = new HashMap<>();
    private ArrayList<Vehicle> tempInactiveVehicles = inactiveVehicles, tempActiveVehicles = activeVehicles;
    private Vehicle vehicleBeingAdded;


    //CEHCKING WHY ITS NOT WORKING DELET LATER
    public ArrayList<ArrayList<Integer>> roadSizes = new ArrayList<>();
    public int INCREMENT = 6;
    public int ADDEDVEHICLES = 1;

    public ArrayList<Vehicle> vehiclesFutureSim = new ArrayList<>();
    public ArrayList<Vehicle> vehiclesNormal = new ArrayList<>();
    public ArrayList<Vehicle> trackedVehiclesOrdered = new ArrayList<>();


    public void incrementTime() {
        incrementTime(0,0);
    }

    public void incrementTime(int vehiclesAddedThisIncrement, int totalVehiclesChecked) {
        addVehicles(vehiclesAddedThisIncrement, totalVehiclesChecked);
        moveVehicles();
        removeVehicles();
        updateSpeed();
        if (isLongTermRouting) updateTrackedVehicleUnfairnessList();
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

                //set diff paths
                if (SimLoop.isTrackingVehicles && !isFutureSim && !vehicle.isStarted()) {
///*
                    //calc optimal time
                    Stack<Road> optimalPath = getPathFromRoutingType(Routing.TYPE_DIJKSTRA_NO_CONGESTION, vehicle.getStartNode(), vehicle.getEndNode());
                    vehicle.optimalPath = (Stack<Road>) optimalPath.clone();
                    double optimalTripTime = 0;
                    double optimalRemainder = 0.0;
                    for (Road optimalRoad : optimalPath) {
                        double timeToTraverse = optimalRoad.getTimeToTraverseNoCongestion(optimalRemainder);
                        optimalTripTime += timeToTraverse;
                        optimalRemainder = 1 - (timeToTraverse % 1);
                    }

                    vehicle.setOptimalTripTime((int) Math.ceil(optimalTripTime));

                    //calc dijkstra estimate
                    Stack<Road> dijkstraPath = getPathFromRoutingType(Routing.TYPE_DIJKSTRA, vehicle.getStartNode(), vehicle.getEndNode());
                    double dijkstraTripTime = 0;
                    double dijkstraRemainder = 0.0;
                    for (Road dijRoad : dijkstraPath) {
                        double timeToTraverse = dijRoad.getTimeToTraverse(dijkstraRemainder);
                        dijkstraTripTime += timeToTraverse;
                        dijkstraRemainder = 1 - (timeToTraverse % 1);
                    }

                    vehicle.setEstimatedDijkstraTime((int) Math.ceil(dijkstraTripTime));

                    //future simulation using dijkstra
                    vehicle.dijkstraPath = (Stack<Road>) dijkstraPath.clone();
                    //int dijTime = futureSim(vehicle, vehiclesAdded, vehiclesChecked, dijkstraPath);
                    //vehicle.setDijkstraTripTime(dijTime);
//*/
                }

                //   VEHICLE ROUTING
                vehicle.setPath(findPath(vehicle));
                if (!Routing.isDynamicRouting) vehicle.actualPath = (Stack<Road>) vehicle.getPath().clone();

                //check if path not empty
                if (vehicle.getPath().size() > 0) {

                    //unfairness stuff
                    if (!isFutureSim && !vehicle.isDijkstraOnly()) {
                        //estimate trip time
                        Stack<Road> pathCopy = (Stack<Road>) vehicle.getPath().clone();
                        double tripTime = 0;
                        double remainder = 0;
                        for (Road road : pathCopy) {
                            double timeToTraverse = road.getTimeToTraverse(remainder);
                            tripTime += timeToTraverse;
                            remainder = 1 - (timeToTraverse % 1);
                        }
                        vehicle.setEstimatedTripTime((int) Math.ceil(tripTime));

                        //add unfairness based on how different estimated trip time is compared to estimated dijkstra
                        double proportion = vehicle.getEstimatedTripTime() / (double) vehicle.getEstimatedDijkstraTime();
                        if (vehicle.getWorstTrip() < proportion) vehicle.setWorstTrip(proportion);
                        vehicle.setUnfairness(vehicle.getUnfairness() + proportion);
                        if (vehicle.getUnfairness() < Double.MAX_VALUE)
                            vehicleUnfairnessMap.put(vehicle, vehicle.getUnfairness());
                        Routing.updateDijkstra_diff_thresholdStats();
                    }

                    Road road = vehicle.getPath().peek();

                    //check if density < 1
                    if (road.getDensity() < Road.MAX_DENSITY) {
                        if (SimLoop.isTrackingVehicles && !isFutureSim)
                            vehiclesSentOut++;
                        tempActiveVehicles.add(vehicle);
                        vehicle.setStarted(true);

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


    public Stack<Road> findPath(Vehicle vehicle) {
        return findPath(vehicle, vehicle.getStartNode(), vehicle.getEndNode());
    }

    public Stack<Road> findPath(Vehicle vehicle, Node startNode, Node endNode) {
        Stack<Road> path;

        // if we are in a future simulation with a future algorithm don't add any vehicles.
        if (isFutureSim && vehicle.getRoutingType() == Routing.TYPE_FUTURE_FASTEST) {
            vehicle.setFinished(true);
            path = new Stack<>();
        }

        //time how long algo takes if !isFutureSim
        else if (!isFutureSim) {
            double startTime = System.nanoTime();
            path = (isLongTermRouting) ? getPathFromRoutingType(vehicle.getRoutingType(), startNode, endNode, vehicle.getUnfairness()) : getPathFromRoutingType(vehicle.getRoutingType(), startNode, endNode);
            double timeTaken = System.nanoTime() - startTime;

            processingTime += timeTaken / 1000000.0; //get time in ms

            //if the path is empty and routing type is TYPE_LEAST_DENSITY_SAFE then use LEAST_DENSITY instead
            if (vehicle.getRoutingType() == Routing.TYPE_LEAST_DENSITY_SAFE && vehicle.getPath().size() == 0)
                path = getPathFromRoutingType(Routing.TYPE_LEAST_DENSITY, startNode, endNode);
        }

        /* Should only happen if isFutureSim, and none of the above conditions are met
         * Note that while in a future simulation, long term routing is ignored to not update vehicle unfairness.
         */
        else path = getPathFromRoutingType(vehicle.getRoutingType(), startNode, endNode);

        if (Routing.isDynamicRouting && !isFutureSim && !path.isEmpty()) vehicle.actualPath.add(path.peek());
        if (path.isEmpty()) {
            System.out.print("");
        }

        return path;
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
                vehicle.setStarted(false);
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

    private void updateTrackedVehicleUnfairnessList() {
        trackedVehicleUnfairnessList.clear();
        for (Vehicle v : trackedVehicles) {
            if (v.getUnfairness() < Double.MAX_VALUE)
                trackedVehicleUnfairnessList.add(v.getUnfairness());
        }
    }


    public int futureSim(Vehicle vehicle, int vehiclesAddedThisIncrement, int totalVehiclesChecked, Stack<Road> path) {

        if (path.size() < 1) return Integer.MAX_VALUE;

        //enable future sim
        incrementFutureSim();

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
        boolean isVehicleCopyAdded = false;
        Vehicle vehicleCopy = (Vehicle) vehicle.clone();
        int addAttempts = 0;
        while (!isVehicleCopyAdded) {
            Stack<Road> pathCopy = (Stack<Road>) path.clone();
            vehicleCopy.setPath(pathCopy);
            Road rd = pathCopy.pop();
            vehicleCopy.setCurrentRoad(rd);
            vehicleCopy.setDynamicRouting(false);
            if (rd.getDensity() < Road.MAX_DENSITY) {
                isVehicleCopyAdded = true;
                rd.addVehicle(vehicleCopy);
                vehicleCopy.setCurrentSpeed(rd.calculateCurrentSpeed());
                tempActiveVehicles.add(vehicleCopy);

                if (addAttempts == 0) {
                    //increment time
                    incrementTime(vehiclesAddedThisIncrement + 1, totalVehiclesChecked + 1); //+1 because we have just added a new vehicle (vehicleCopy)
                    //checkDensities(vehiclesAddedThisIncrement);
                }
                else incrementTime(1,1);
            }
            else {
                if (addAttempts == 0) incrementTime(vehiclesAddedThisIncrement, totalVehiclesChecked + 1);
                else incrementTime(0,1);
            }
            addAttempts++;
        }

        while (!vehicleCopy.isFinished()) {
            incrementTime();
            //checkDensities(vehiclesAddedThisIncrement);
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
        decrementFutureSim();

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
    }


    //for debugging
    private void checkDensities(int vehiclesAdded) {
        if (SimLoop.getIncrementCount() == ADDEDVEHICLES && vehiclesAdded == INCREMENT) {
            for (int i = 0; i < roads.size(); i++) {
                Road r = roads.get(i);
                roadSizes.get(i).add(r.getVehicles().size());
            }
        }
    }

    public Stack<Road> getPathFromRoutingType(int routingType, Node startNode, Node endNode) {
        return getPathFromRoutingType(routingType, startNode, endNode, 0);
    }

    public Stack<Road> getPathFromRoutingType(int routingType, Node startNode, Node endNode, double unfairness) {
        Stack<Road> path;
        switch (routingType) {

            //NOT ADAPTED FOR LONG TERM ROUTING
            case Routing.TYPE_FUTURE_FASTEST:
                Stack<Road> pathFutureTime = Routing.future(startNode, endNode, this, false);
                path = pathFutureTime;
                break;

            //NOT ADAPTED FOR LONG TERM ROUTING
            case Routing.TYPE_FUTURE_LEAST_DENSITY:
                Stack<Road> pathFutureDensity = Routing.future(startNode, endNode, this, true);
                path = pathFutureDensity;
                break;

            case Routing.TYPE_NULL:
                throw new IllegalStateException("Unexpected value: " + routingType + ". This routing type cannot be assigned a path.");

            default:
                path  = (isLongTermRouting)
                        ? Routing.longTermRouting(startNode, endNode, nodes, routingType, ListsUtil.calcAverageDouble(vehicleUnfairnessMap.values()), unfairness)
                        : Routing.dijkstraGeneral(startNode, endNode, nodes, routingType);
        }
        return path;
    }

    /**
     * Resets elements of vehicle lists, roads and vehicles so they can perform more trips.
     */
    public void resetForNextDay() {
        resetVehicleLists();
        resetRoads();
        resetVehicles();
    }

    /**
     * clears {@Code inactiveVehicles} and {@Code activeVehicles}, and repopulates inactive vehicles using the {@Code vehicles} array.
     */
    private void resetVehicleLists() {
        inactiveVehicles.clear();
        activeVehicles.clear();
        for (Vehicle v : vehicles) {
            inactiveVehicles.add(v);
        }

        tempInactiveVehicles = inactiveVehicles;
        tempActiveVehicles = activeVehicles;
    }

    private void resetRoads() {
        for (Road road : roads) {
            road.getVehicles().clear();
            road.calculateCurrentSpeed();
        }
    }

    private void resetVehicles() {
        for (Vehicle v : vehicles) {
            v.setFinished(false);
            v.setActualTripTime(0);
            v.resetTripDistance();
        }
    }


    public void createRandomNodeVehicles(boolean isDijkstraOnly) {
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

        if (isDijkstraOnly) setVehiclesDijkstraOnly(SimLoop.DIJKSTRA_ONLY_PROPORTION);

        int initial = MAX_VEHICLES_ADDED_PER_TIME_INCREMENT*SimLoop.INIT_ITERATIONS;
        for (int n = initial; n < initial + MAX_VEHICLES_ADDED_PER_TIME_INCREMENT*SimLoop.NUM_ITERATIONS; n++) {
            Vehicle v = vehicles[n];
            trackedVehiclesOrdered.add(v);
            if (v.isDijkstraOnly())
                dijkstraOnlyTrackedVehicles.add(v);
            else trackedVehicles.add(v);
        }
    }

    public void setVehiclesDijkstraOnly(double proportion) {
        Random r = new Random(654321);
        for (Vehicle v : vehicles) {
            if (r.nextDouble() < proportion) {
                v.setDijkstraOnly(true);
            }
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

    public ArrayList<Vehicle> getDijkstraOnlyTrackedVehicles() {
        return dijkstraOnlyTrackedVehicles;
    }

    public ArrayList<Double> getTrackedVehicleUnfairnessList() {
        return trackedVehicleUnfairnessList;
    }

    public ArrayList<Node> getNodes() {
        return nodes;
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


    public void incrementFutureSim() {
        futureSimCounter++;
        isFutureSim = true;
    }

    public void decrementFutureSim() {
        futureSimCounter--;
        if (futureSimCounter == 0) isFutureSim = false;
    }

    public double getProcessingTime() {
        return processingTime;
    }

    public boolean isLongTermRouting() {
        return isLongTermRouting;
    }

    public void setLongTermRouting(boolean longTermRouting) {
        isLongTermRouting = longTermRouting;
    }
}