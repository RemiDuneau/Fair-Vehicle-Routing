
import java.util.*;

public class Routing {
    public static final int TYPE_NULL = -1;
    public static final int TYPE_DIJKSTRA = 0;
    public static final int TYPE_FAIR = 1;
    public static final int TYPE_LEAST_DENSITY = 2;
    public static final int TYPE_FUTURE_DIJKSTRA = 3;
    public static final int TYPE_FUTURE_LEAST_DENSITY = 4;

    /**
     * Finds the shortest path from the start node to the end node taking the path of shortest overall time
     * (i.e. the least time to get to end node).
     * @param startNode the start node
     * @param endNode the end node
     * @param nodes a list of all nodes in the graph
     * @param isConsideringCongestion true: current road density is taken into account when finding optimal path. false: assume road density is 0.0 when finding optimal path.
     * @return a path ({@code Stack<Road>}) of the optimal route.
     */
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
                    int time;
                    if (isConsideringCongestion)
                        time = (road.getDensity() >= Road.MAX_DENSITY) ? Integer.MAX_VALUE : timesMap.get(currentNode) + road.getTimeToTraverse();
                    else time = timesMap.get(currentNode) + road.getTimeToTraverseNoCongestion();

                    if (time < timesMap.get(neighbour)) {
                        timesMap.put(neighbour, time);
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
     */
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
                    double density = (road.getDensity() >= Road.MAX_DENSITY) ? Double.MAX_VALUE-10 : densitiesMap.get(currentNode) + road.calculateDensity();
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
