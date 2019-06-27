
import java.util.*;

public class RoutingType {

    public static final int TYPE_DIJKSTRA = 0;
    public static final int TYPE_FAIR = 1;
    public static final int TYPE_LEAST_DENSITY = 2;
    public static final int TYPE_FUTURE_DIJKSTRA = 3;
    public static final int TYPE_FUTURE_LEAST_DENSITY = 4;
    public static int acc = 0;

    /**
     * Finds the shortest path from the start node to the end node taking the path of shortest overall time
     * (i.e. the least time to get to end node).
     * @param startNode the start node
     * @param endNode the end node
     * @param nodes a list of all nodes inthe graph
     * @return a path ({@code Stack<Road>}) of the optimal route.
     */
    public static Stack<Road> dijkstra(Node startNode, Node endNode, List<Node> nodes) {
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

            //visit neighbours and add to known nodes
            for (Node neighbour : currentNode.getNeighbours().keySet()) {
                if (unvisited.contains(neighbour)) {
                    int time = timesMap.get(currentNode) + currentNode.getRoad(neighbour).getTimeToTraverse();
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
            if (currentNode == endNode) {
                return createStack(endNode, previousNodeMap);
            }

            //visit neighbours and add to known nodes
            for (Node neighbour : currentNode.getNeighbours().keySet()) {
                if (unvisited.contains(neighbour)) {
                    double roadDensity = currentNode.getRoad(neighbour).calculateDensity();
                    double density = densitiesMap.get(currentNode) + roadDensity;
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
     * finds all the paths to get from the specified start node to the specified end node
     * @param startNode the starting node
     * @param endNode the destination node
     * @return a list of paths ({@code Stack<Road>}) to get from the start node to the end node
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
                    child.add(node);
                    if (!searchStack.contains(child))
                        searchStack.push(child);
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
     * @param nodes
     * @param roads
     * @param activeVehicles
     * @param allPaths
     * @param isLeastDensity is true when measuring least density (i.e. {@code TYPE_FUTURE_LEAST_DENSITY}), and false otherwise
     * @return the least cost path (i.e. {@code Stack<Road>})
     */
    public static Stack<Road> future(Node startNode, Node endNode, List<Node> nodes, List<Road> roads,
                                     List<Vehicle> activeVehicles, Map<Tuple<Node, Node>, ArrayList<Stack<Road>>> allPaths,
                                     boolean isLeastDensity) {

        ArrayList<Stack<Road>> paths = new ArrayList<>();

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
            double timeTaken = calculateFuture(startNode, endNode, nodes, roads, activeVehicles, path, isLeastDensity);
            if (timeTaken < best) {
                best = timeTaken;
                bestPath = path;
            }
        }
        return bestPath;
    }

    public static double calculateFuture(Node startNode, Node endNode, List<Node> nodes, List<Road> roads,
                                      List<Vehicle> activeVehicles, Stack<Road> path, boolean isLeastDensity) {

        //-------------- CLONING --------------
        //clone roads
        ArrayList<Road> roadsCopy = new ArrayList<>();
        Map<Road, Road> roadsToCopyMap = new HashMap<>();
        Map<Road, Road> copyToRoadsMap = new HashMap<>();

        for (Road road : roads) {
            Road copy = (Road) road.clone();
            copy.setVehicles(new ArrayList<>());
            roadsCopy.add(copy);
            roadsToCopyMap.put(road, copy);
            copyToRoadsMap.put(copy, road);
        }


        //clone nodes
        ArrayList<Node> nodesCopy = new ArrayList<>();
        Map<Node, Node> nodesToCopyMap = new HashMap<>();
        for (Node node : nodes) {
            Node copy = (Node) node.clone();
            nodesCopy.add(copy);
            nodesToCopyMap.put(node, copy);
        }

        //clone vehicles
        ArrayList<Vehicle> activeVehiclesCopy = new ArrayList<>();

        for (Vehicle vehicle : activeVehicles) {
            Vehicle copy = (Vehicle) vehicle.clone();
            Road roadCopy = roadsToCopyMap.get(copy.getCurrentRoad());
            copy.setCurrentRoad(roadCopy);
            roadCopy.addVehicle(copy);
            copy.setStartNode(nodesToCopyMap.get(copy.getStartNode()));
            copy.setEndNode(nodesToCopyMap.get(copy.getEndNode()));

            //get cloned roads for vehicle's path
            Stack<Road> pathCopy = new Stack<>();
            for (Road road : copy.getPath()) {
                pathCopy.push(roadsToCopyMap.get(road));
            }
            copy.setPath(pathCopy);

            activeVehiclesCopy.add(copy);
        }
        //clone nodes' neighbours
        for (Node copy : nodesCopy) {
            Map<Node, Road> neighboursCopy = new HashMap<>();
            for (Node end : copy.getNeighbours().keySet()) {
                Road road = copy.getRoad(end);
                neighboursCopy.put(nodesToCopyMap.get(end), roadsToCopyMap.get(road));
            }
            copy.setNeighbours(neighboursCopy);
        }

        //set cloned roads' start and end nodes to the cloned nodes
        for (Road copy : roadsCopy) {
            copy.setStartNode(nodesToCopyMap.get(copy.getStartNode()));
            copy.setEndNode(nodesToCopyMap.get(copy.getEndNode()));

        }

        //-------------- CALCULATING TIME TAKEN --------------

        double totalCost = 0;
        Node currentNode = nodesToCopyMap.get(startNode);
        Node endNodeCopy = nodesToCopyMap.get(endNode);

        Vehicle v = new Vehicle(TYPE_FUTURE_DIJKSTRA); //"dummy" of the vehicle a path will be created for

        //copy path to new stack
        Stack<Road> pathCopy = new Stack<>();
        for (Road road : path) {
            pathCopy.push(roadsToCopyMap.get(road));
        }
        breakpoint:
        //loop through vehicle path until reached endNode
        while (currentNode != endNodeCopy) {

            //determine number of timesteps to get to next node
            Road currentRoadCopy = pathCopy.pop();
            currentRoadCopy.addVehicle(v);
            v.setCurrentRoad(currentRoadCopy);
            v.setCurrentSpeed(currentRoadCopy.calculateCurrentSpeed());
            int nTimeSteps = v.calculateTimeIncrementsToFinishRoad();
            currentNode = currentRoadCopy.getEndNode();

            if (isLeastDensity) totalCost += currentRoadCopy.getDensity();
            else totalCost += nTimeSteps;

            if (currentNode == endNodeCopy) break breakpoint;


            //simulate those timesteps for every vehicle in the graph
            for (int i = 0; i < nTimeSteps; i++) {
                for (Vehicle vehicle : activeVehiclesCopy) {
                    vehicle.move();
                }
            }

        }
        return totalCost;
    }

    public static Stack<Road> fair() {
        return  new Stack<>();
    }
}
