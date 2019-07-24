import java.sql.Time;
import java.util.ArrayList;
import java.util.Random;
import java.util.Stack;

public class Graph {

    public Graph(ArrayList<Node> nodes, ArrayList<Road> roads) {
        this.nodes = nodes;
        this.roads = roads;
    }

    private ArrayList<Node> nodes;
    private ArrayList<Road> roads;
    private static Random knownSeedRandom = new Random(987654321);

    /**
     * Calculates for each node which nodes it can get to and updates that node's list
     * @param graph
     */
    public static void calculateConnectedNodes(Graph graph) {
        ArrayList<Node> nodes = new ArrayList<>();
        try {
            nodes = graph.getNodes();
        } catch (NullPointerException e) {
            System.err.println("graph must have a non-null number of nodes");
        }

        for (Node node : nodes) {
            Stack<Node> unexploredNodes = new Stack<>();
            ArrayList<Node> visited = new ArrayList<>();
            unexploredNodes.push(node);

            while (!unexploredNodes.isEmpty()) {
                Node currentNode = unexploredNodes.pop();
                ArrayList<Node> reachableNodes = currentNode.getReachableNodes();
                if (!visited.contains(currentNode) &&
                        (!reachableNodes.contains(currentNode) || node != currentNode)) {
                    reachableNodes.add(currentNode);
                }
                for (Node childNode : currentNode.getNeighbours().keySet()) {
                    if (!visited.contains(childNode)) {
                        unexploredNodes.add(childNode);
                        visited.add(childNode);
                    }
                }
            }
        }
    }

    public static Graph makeRandomGraph(int nNodes, double roadProbability) {
        return randomGraph(nNodes, roadProbability);
    }

    public static Graph makeRandomGraph() {
        return randomGraph(20, 40);
    }

    private static Graph randomGraph(int nNodes, double roadProbability) {
        ArrayList<Node> nodes = new ArrayList<>(nNodes);
        ArrayList<Road> roads = new ArrayList<>();

        //create nodes
        for (int i = 0; i < nNodes; i++) {
            Node node = new Node(i);
            nodes.add(node);
        }

        //create roads
        Random random = new Random();
        Random RANDOM = TimeController.RANDOM;
        for (int i = 0; i < nodes.size(); i++) {
            if (i > 0) {
                if (RANDOM.nextDouble() < roadProbability) {
                    Node prevNode = nodes.get(i-1);
                    Node node = nodes.get(i);
                    int maxSpeed = RANDOM.nextInt(20) + 8; // +8 so minimum speed is 8m/s, or ~ 30km/h, to make roads more realistic
                    int maxRoadLength = RANDOM.nextInt(2000) + 50; //minimum road length is 50.
                    Road road1 = new Road(node, prevNode, maxSpeed, maxRoadLength);
                    Road road2 = new Road(prevNode, node, maxSpeed, maxRoadLength);
                    node.addNeighbour(prevNode, road1);
                    prevNode.addNeighbour(node, road2);
                }
            }
        }

        return new Graph(nodes, roads);
    }


    public ArrayList<Node> getNodes() {
        return nodes;
    }

    public ArrayList<Road> getRoads() {
        return roads;
    }
}
