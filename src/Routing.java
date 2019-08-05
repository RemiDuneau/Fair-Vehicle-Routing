import java.util.*;

public class Routing {

    public static final int TYPE_NULL = -1;
    public static final int TYPE_DIJKSTRA = 0;
    public static final int TYPE_DIJKSTRA_NO_CONGESTION = 1;
    public static final int TYPE_LEAST_DENSITY = 2;
    public static final int TYPE_LEAST_DENSITY_SAFE = 3;
    public static final int TYPE_LEAST_DENSITY_ROAD_LENGTH = 4;
    public static final int TYPE_LEAST_DENSITY_EXPONENTIAL = 5;
    public static final int TYPE_GREATEST_SPEED_ROAD_LENGTH = 6;
    public static final int TYPE_FUTURE_FASTEST = 10;
    public static final int TYPE_FUTURE_LEAST_DENSITY = 11;

    public static double least_density_safe_threshold = 0.1;

    public static boolean isDijkstraDiffThresholdEnabled = false;
    public static double dijkstra_diff_threshold = 1.1;

    public static boolean enableDynamicRouting = true;



    public static Stack<Road> dijkstraGeneral(Node startNode, Node endNode, ArrayList<Node> nodes, int routingType) {
        return dijkstraGeneral(startNode, endNode, nodes, routingType, Integer.MAX_VALUE);
    }

    /**
     * Finds the least cost path from a given start node to a given end node based on the routing type.
     * @param startNode The starting node
     * @param endNode The destination node
     * @param nodes A list of all reachable nodes
     * @param routingType The routing type, which will determine how each path's cost is calculated.
     *                    The routing type constants in the {@code Routing} class should be used here (e.g. {@code TYPE_DIJKSTRA}).
     * @param dijkstraTripTime the {@code Vehicle}'s {@code dijkstraTripTime} (ensure this is already calculated).
     * @return A least cost path ({@code Stack<Road>}) between the nodes given.
     */
    public static Stack<Road> dijkstraGeneral(Node startNode, Node endNode, ArrayList<Node> nodes, int routingType, int dijkstraTripTime) {

        //initialisation
        ArrayList<Node> unvisited = new ArrayList<>(nodes);
        Map<Node, Double> costMap = new LinkedHashMap<>();
        Map<Node, Node> previousNodeMap = new HashMap<>();

        //set initial costs
        for (int i = 0; i < nodes.size(); i++) {
            Node node = nodes.get(i);
            costMap.put(node, Double.MAX_VALUE);
            previousNodeMap.put(node, null);
        }
        costMap.put(startNode, 0.0); //set start node to 0

        //loop through nodes
        while (unvisited.size() > 0) {

            //find closest node
            double minCost = Double.MAX_VALUE;
            Node currentNode = null;
            for (Node node : costMap.keySet()) {
                double cost = costMap.get(node);
                if (cost < minCost) {
                    minCost = cost;
                    currentNode = node;
                }
            }

            //return road path if currentNode is endNode
            if (currentNode == endNode) {
                return createStack(endNode, previousNodeMap);
            }

            if (currentNode == null)  {
                return new Stack<>();
            }


            //calculate if current estimated trip time is greater than dijDiff
            int estimatedTime = 0;
            if (isDijkstraDiffThresholdEnabled) {
                Stack<Road> currentPath = createStack(currentNode, previousNodeMap);
                double currentRemainder = 0.0;
                for (Road rd : currentPath) {
                    estimatedTime += rd.getTimeToTraverse(currentRemainder);
                    currentRemainder = calculateRemainder(rd, true);
                }
            }
            if (estimatedTime < dijkstraTripTime * dijkstra_diff_threshold) {

                //visit neighbours and add to known nodes
                for (Node neighbour : currentNode.getNeighbours().keySet()) {
                    if (unvisited.contains(neighbour)) {
                        Road road = currentNode.getRoad(neighbour);
                        double cost = Double.MAX_VALUE;
                        double remainder;


                        //determine what the cost is based on routing type
                        switch (routingType) {
                            case TYPE_DIJKSTRA:
                                remainder = calculateRemainder(currentNode, previousNodeMap, true);
                                if (road.getDensity() < Road.MAX_DENSITY)
                                    cost = costMap.get(currentNode) + road.getTimeToTraverse(remainder);
                                else cost = Integer.MAX_VALUE - 10;
                                break;

                            case TYPE_DIJKSTRA_NO_CONGESTION:
                                remainder = calculateRemainder(currentNode, previousNodeMap, false);
                                cost = costMap.get(currentNode) + road.getTimeToTraverseNoCongestion(remainder);
                                break;

                            case TYPE_LEAST_DENSITY:
                                double least_densityDensity = road.calculateDensity();
                                if (least_densityDensity < Road.MAX_DENSITY)
                                    cost = costMap.get(currentNode) + least_densityDensity;
                                break;

                            case TYPE_LEAST_DENSITY_SAFE:
                                double currentDensity = road.getDensity();
                                boolean isRoutable = true;
                                for (Road rd : currentNode.getNeighbours().values()) {
                                    if (currentDensity > rd.getDensity() && unvisited.contains(rd.getEndNode()) && rd.getEndNode().getNeighbours().size() > 0) {
                                        isRoutable = false;
                                    }
                                }

                                //if road is least dense road or it's density is below the threshold, AND the road doesn't lead to a visited node (avoids loops) then assign new cost
                                if ((isRoutable || currentDensity < least_density_safe_threshold) && unvisited.contains(road.getEndNode())) {
                                    remainder = calculateRemainder(currentNode, previousNodeMap, true);
                                    //if (road.getDensity() < Road.MAX_DENSITY)
                                        cost = costMap.get(currentNode) + road.getTimeToTraverse(remainder);
                                }
                                break;

                            case TYPE_LEAST_DENSITY_ROAD_LENGTH:
                                double least_density_road_lengthDensity = road.calculateDensity();
                                //if (least_density_road_lengthDensity < Road.MAX_DENSITY)
                                    cost = costMap.get(currentNode) + least_density_road_lengthDensity * road.getLength();
                                break;

                            case TYPE_LEAST_DENSITY_EXPONENTIAL:
                                double least_density_exponentialDensity = road.calculateDensity();
                                //if (least_density_exponentialDensity < Road.MAX_DENSITY)
                                    cost = costMap.get(currentNode) + Math.exp(least_density_exponentialDensity);

                                break;

                            case TYPE_GREATEST_SPEED_ROAD_LENGTH:
                                double greatest_speed_road_lengthSpeed = 1 / (double) road.calculateCurrentSpeed();
                                //if (road.getDensity() < Road.MAX_DENSITY)
                                    cost = costMap.get(currentNode) + greatest_speed_road_lengthSpeed * road.getLength();
                                break;

                            default:
                                throw new IllegalStateException("Unexpected value: " + routingType);
                        }

                        //update maps
                        if (cost < costMap.get(neighbour)) {
                            costMap.put(neighbour, cost);
                            previousNodeMap.put(neighbour, currentNode);
                        }
                    }
                }
            }
            unvisited.remove(currentNode);
            costMap.remove(currentNode);
        }
        return null;
    }

    private static double calculateRemainder(Node currentNode, Map<Node, Node> previousNodeMap, boolean isConsideringCongestion) {
        if (previousNodeMap.get(currentNode) != null) {
            Node prevNode = previousNodeMap.get(currentNode);
            Road prevRoad = prevNode.getRoad(currentNode);
            return calculateRemainder(prevRoad, isConsideringCongestion);
        }
        else return 0.0;
    }

    private static double calculateRemainder(Road road, boolean isConsideringCongestion) {
        return (isConsideringCongestion)
                ? 1 - road.getTimeToTraverse() % 1
                : 1 - road.getTimeToTraverseNoCongestion() % 1;
    }

/*
    /**
     * Finds the shortest path from the start node to the end node taking the path of shortest overall time
     * (i.e. the least time to get to end node).
     * @param startNode the start node
     * @param endNode the end node
     * @param nodes a list of all nodes in the graph
     * @param isConsideringCongestion true: current road density is taken into account when finding optimal path. false: assume road density is 0.0 when finding optimal path.
     * @return a path ({@code Stack<Road>}) of the optimal route.

    public static Stack<Road> dijkstra(Node startNode, Node endNode, List<Node> nodes, boolean isConsideringCongestion) {
        //initialisation
        ArrayList<Node> unvisited = new ArrayList<>(nodes);
        Map<Node, Integer> timesMap = new HashMap<>();
        Map<Node, Node> previousNodeMap = new HashMap<>();

        for (Node node : nodes) {
            timesMap.put(node, Integer.MAX_VALUE);
            previousNodeMap.put(node, null);
        }
        timesMap.put(startNode, 0); //set start node to 0

        while (unvisited.size() > 0) {

            //find closest node
            int minTime = Integer.MAX_VALUE;
            Node currentNode = null;
            for (Node node : timesMap.keySet()) {
                int time = timesMap.get(node);
                if (time < minTime) {
                    minTime = time;
                    currentNode = node;
                }
            }

            //return road path if currentNode is endNode
            if (currentNode == endNode) {
                return createStack(endNode, previousNodeMap);
            }

            if (currentNode == null) return new Stack<>();

            //visit neighbours and add to known nodes
            for (Node neighbour : currentNode.getNeighbours().keySet()) {
                if (unvisited.contains(neighbour)) {
                    Road road = currentNode.getRoad(neighbour);

                    //determine what the time is based on whether road density is being considered (assume road is empty if not)
                    double time;

                    //calculate remainder
                    double remainder = 0.0;
                    if (previousNodeMap.get(currentNode) != null) {
                        Node prevNode = previousNodeMap.get(currentNode);
                        Road prevRoad = prevNode.getRoad(currentNode);
                        remainder = (isConsideringCongestion) ? prevRoad.getTimeToTraverse() % 1 : prevRoad.getTimeToTraverseNoCongestion() % 1;
                    }
                    if (isConsideringCongestion)
                        time = (road.getDensity() >= Road.MAX_DENSITY) ? Integer.MAX_VALUE : timesMap.get(currentNode) + road.getTimeToTraverse(remainder);
                    else time = timesMap.get(currentNode) + road.getTimeToTraverseNoCongestion(remainder);

                    if (time < timesMap.get(neighbour)) {
                        timesMap.put(neighbour, (int) time);
                        previousNodeMap.put(neighbour, currentNode);
                    }
                }
            }
            unvisited.remove(currentNode);
            timesMap.remove(currentNode);
        }
        return null;
    }

    /**
     * Finds the shortest path from the start node to the end node taking the lowest road density path
     * (i.e. the least congested roads).
     * @param startNode vehicle's starting node
     * @param endNode vehicle's end node
     * @param nodes a list of all the nodes in the graph
     * @return a stack of the roads the vehicle should travel on.

    public static Stack<Road> leastDensity(Node startNode, Node endNode, List<Node> nodes) {
        //initialisation
        ArrayList<Node> unvisited = new ArrayList<>(nodes);
        Map<Node, Double> densitiesMap = new HashMap<>();
        Map<Node, Node> previousNodeMap = new HashMap<>();

        for (Node node : nodes) {
            densitiesMap.put(node, Double.MAX_VALUE);
            previousNodeMap.put(node, null);
        }
        densitiesMap.put(startNode, 0.0); //set start node to 0

        while (unvisited.size() > 0) {

            //find closest node
            double minDensity = Double.MAX_VALUE;
            Node currentNode = null;
            for (Node node : densitiesMap.keySet()) {
                double density = densitiesMap.get(node);
                if (density < minDensity) {
                    minDensity = density;
                    currentNode = node;
                }
            }

            //return road path if currentNode is endNode
            if (currentNode == endNode) return createStack(endNode, previousNodeMap);

            if (currentNode == null) return new Stack<>();
            //visit neighbours and add to known nodes
            for (Node neighbour : currentNode.getNeighbours().keySet()) {
                if (unvisited.contains(neighbour)) {
                    Road road = currentNode.getRoad(neighbour);
                    double density = (road.getDensity() >= Road.MAX_DENSITY) ? Double.MAX_VALUE : densitiesMap.get(currentNode) + road.calculateDensity();
                    if (density < densitiesMap.get(neighbour)) {
                        densitiesMap.put(neighbour, density);
                        previousNodeMap.put(neighbour, currentNode);
                    }
                }
            }
            unvisited.remove(currentNode);
            densitiesMap.remove(currentNode);
        }
        return null;
    }
    */

    private static Stack<Road> createStack(Node currentNode, Map<Node, Node> previousNodeMap) {
        Stack<Road> path = new Stack<>();
        Node node = currentNode;
        while (true) {
            if (previousNodeMap.get(node) != null) {
                Node prevNode = previousNodeMap.get(node);
                Road road = prevNode.getRoad(node);
                node = previousNodeMap.get(node);
                path.push(road);
            } else return path;
        }
    }

    /**
     * Finds all the acyclic paths to get from the specified start node to the specified end node
     * @param startNode the starting node
     * @param endNode the destination node
     * @return a list of acyclic paths ({@code Stack<Road>}) to get from the start node to the end node
     */
    public static ArrayList<Stack<Road>> dfsFindAllPaths(Node startNode, Node endNode) {
        ArrayList<ArrayList<Node>> nodePaths = new ArrayList<>();
        Stack<ArrayList<Node>> searchStack = new Stack<>();
        ArrayList<Node> start = new ArrayList<>();
        start.add(startNode);
        searchStack.add(start);
        while (!searchStack.isEmpty()) {
            ArrayList<Node> path = searchStack.pop();
            if (path.get(path.size()-1) == endNode && path.get(0) == startNode) {
                nodePaths.add(path);
            }
            else {
                for (Node node : path.get(path.size()-1).getNeighbours().keySet()) {
                    ArrayList<Node> child = (ArrayList<Node>) path.clone();
                    if (!child.contains(node)) {
                        child.add(node);
                        searchStack.push(child);
                    }
                }
            }
        }

        //convert node stack into road stack
        ArrayList<Stack<Road>> paths = new ArrayList<>();
        for (ArrayList<Node> nodePath : nodePaths) {
            Stack<Road> inversePath = new Stack<>();

            Node previous = nodePath.get(0);
            nodePath.remove(0);

            for (Node node : nodePath) {
                Road road = previous.getRoad(node);
                inversePath.push(road);
                previous = node;
            }

            Stack<Road> path = new Stack<>();
            int size = inversePath.size();
            for (int i = 0; i < size; i++) {
                path.push(inversePath.pop());
            }
            paths.add(path);
        }
        return paths;
    }

    /**
     *
     * @param startNode
     * @param endNode
     * @param isLeastDensity is true when measuring least density (i.e. {@code TYPE_FUTURE_LEAST_DENSITY}), and false otherwise
     * @return the least cost path (i.e. {@code Stack<Road>})
     */
    public static Stack<Road> future(Node startNode, Node endNode, TimeController timeController, boolean isLeastDensity) {

        ArrayList<Stack<Road>> paths = new ArrayList<>();
        List<Vehicle> activeVehicles = timeController.getActiveVehicles();
        Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPaths = timeController.getAllPathsMap();
        Vehicle vehicle = timeController.getVehicleBeingAdded();

        //find list of paths wanted
        for (Tuple tuple : allPaths.keySet()) {
            if (tuple.x == startNode && tuple.y == endNode) {

                //make a copy of every path (so subsequent vehicles can use the path without some of the roads missing due to previous vehicles)
                for (Stack<Road> path : allPaths.get(tuple)) {
                    Stack<Road> pathCopy = new Stack<>();
                    pathCopy.addAll(path);
                    paths.add(pathCopy);
                }
            }
        }

        //return if only one path can be taken (no need to find fastest)
        if (paths.size() == 1) return paths.get(0);

        //find optimal path
        Stack<Road> bestPath = new Stack<>();
        double best = Double.MAX_VALUE;
        for (Stack<Road> path : paths) {
            double timeTaken = calculateFuture(startNode, endNode, path, vehicle, timeController, isLeastDensity);
            if (timeTaken < best) {
                best = timeTaken;
                bestPath = path;
                vehicle.estimatedFutureTime = timeTaken;
            }
        }
        return bestPath;
    }
    // Mark is the best programmer ever

    public static double calculateFuture(Node startNode, Node endNode, Stack<Road> path, Vehicle vehicle, TimeController timeController, boolean isLeastDensity) {
    //TODO i don't think this works with future density because I'm a numpty
        //-------------- CALCULATING TIME TAKEN --------------

        int maxTimeSteps = 600;
        int currentTimeSteps = 0;

        //copy path to new stack
        Stack<Road> pathCopy = new Stack<>();
        for (Road road : path) {
            pathCopy.push(road);
        }

        return timeController.futureSim(vehicle, timeController.getVehiclesAddedThisIncrement(), timeController.getVehiclesCheckedThisIncrement(), pathCopy);
    }

    public static Stack<Road> fair() {
        return  new Stack<>();
    }
}
