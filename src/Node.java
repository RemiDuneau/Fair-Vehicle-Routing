import java.util.HashMap;
import java.util.Map;

public class Node implements Cloneable {

    public Node(int id) {
        this.id = id;
    }

    private int id;
    private Map<Node, Road> neighbours = new HashMap();


    public Object clone() {
        Node clone = null;
        try {
            clone = (Node) super.clone();
        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
        }
        return clone;
    }


    public int getId() {
        return id;
    }

    public void addNeighbour(Node node, Road road) {
        neighbours.putIfAbsent(node, road);
    }

    public Road getRoad(Node neighbour) {
        return neighbours.get(neighbour);
    }

    public Map<Node, Road> getNeighbours() {
        return neighbours;
    }

    public void setNeighbours(Map<Node, Road> neighbours) {
        this.neighbours = neighbours;
    }

}
