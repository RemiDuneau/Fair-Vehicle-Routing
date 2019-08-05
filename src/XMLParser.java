import org.w3c.dom.Document;
import org.w3c.dom.NodeList;
import org.w3c.dom.Element;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class XMLParser {

    /*
     * FORMAT:
     * <graph>
     *   <node>
     *     <id>3</id>
     *   </node>
     *   <road>
     *     <startnode>3</startnode>
     *     <endnode>4</endnode>
     *     <length>5000</length>
     *     <maxspeed>70</maxspeed>
     *   </road>
     * </graph>
     */

    public static Graph parseXML(File file) {
        ArrayList<Node> nodes = new ArrayList<>();
        ArrayList<Road> roads = new ArrayList<>();

        try {
            DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();
            Document document = documentBuilder.parse(file);

            //parse and create nodes
            NodeList nodesList = document.getElementsByTagName("node");
            for (int i = 0; i < nodesList.getLength(); i++) {
                int id = Integer.valueOf(((Element) nodesList.item(i)).getElementsByTagName("id").item(0).getTextContent());
                nodes.add(new Node(id));
            }

            //parse and create roads
            NodeList roadsList = document.getElementsByTagName("road");
            for (int i = 0; i < roadsList.getLength(); i++) {
                int startNodeId = Integer.valueOf(((Element) roadsList.item(i)).getElementsByTagName("startnode").item(0).getTextContent());
                int endNodeId = Integer.valueOf(((Element) roadsList.item(i)).getElementsByTagName("endnode").item(0).getTextContent());
                int length = Integer.valueOf(((Element) roadsList.item(i)).getElementsByTagName("length").item(0).getTextContent());
                int maxSpeed = Integer.valueOf(((Element) roadsList.item(i)).getElementsByTagName("maxspeed").item(0).getTextContent());

                //find nodes
                Node startNode = null;
                Node endNode = null;
                for (Node node : nodes) {
                    if (node.getId() == startNodeId) {
                        startNode = node;
                    }
                    if (node.getId() == endNodeId) {
                        endNode = node;
                    }
                }
                Road road = new Road(startNode, endNode, length, maxSpeed);
                roads.add(road);

                //update relevant nodes' neighbours
                startNode.addNeighbour(endNode, road);
            }
        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return new Graph(nodes, roads);
    }
}
