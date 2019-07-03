import java.util.ArrayList;
import java.util.Arrays;
import java.util.Stack;

public class Graph {

    public Graph(ArrayList<Node> nodes, ArrayList<Road> roads) {
        this.nodes = nodes;
        this.roads = roads;
    }

    private ArrayList<Node> nodes;
    private ArrayList<Road> roads;

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


    public ArrayList<Node> getNodes() {
        return nodes;
    }

    public ArrayList<Road> getRoads() {
        return roads;
    }
}
