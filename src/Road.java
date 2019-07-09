import java.util.ArrayList;

public class Road {
    public static final double MAX_DENSITY  = 1.0;

    public Road(Node startNode, Node endNode, int length, int maxSpeed) {
        this.startNode = startNode;
        this.endNode = endNode;
        this.length = length;
        this.maxSpeed = maxSpeed;
        currentSpeed = maxSpeed;
    }

    private ArrayList<Vehicle> vehicles = new ArrayList<>();
    private Node startNode, endNode;
    private int length, maxSpeed, currentSpeed, timeToTraverse, totalVehiclesAdded = 0;

    //state variables
    private double density, densitySum = 0; //proportion of occupied cells

    public int calculateCurrentSpeed() {
        calculateDensity();
        currentSpeed = Math.max(1, greenbergSpeedEquation()); //minimum speed = 1, so vehicles don't get stuck
        timeToTraverse = (int) ( (double) length / (double) currentSpeed);
        return currentSpeed;
    }


    /**
     * Calculates the proportion of road occupied by a vehicle (i.e the traffic density), and returns that value.
     * @return the traffic density
     */
    public double calculateDensity() {
        return density = vehicles.size() * Vehicle.VEHICLE_LENGTH / (double) length;
    }

    /**
     * Calculates the road density, and adds it to {@code densitySum}. This should only be used at the end of the time increment.
     */
    public void incrementDensitySum() {
        calculateDensity();
        if (SimLoop.isPopulated && !TimeController.isFutureSim) densitySum += density;
    }


    private int greenbergSpeedEquation() {
        if (vehicles.size() == 0) return maxSpeed; //return max speed if road is empty
        return Math.min( maxSpeed, (int) (maxSpeed * Math.log(MAX_DENSITY/density)) );
    }

    public int getTimeToTraverseNoCongestion() {
        return (int) Math.ceil((double) length / (double) maxSpeed);
    }

    public int getMaxSpeed() {
        return maxSpeed;
    }

    public int getCurrentSpeed() {
        return currentSpeed;
    }

    public int getTimeToTraverse() {
        return timeToTraverse;
    }

    public int getLength() {
        return length;
    }

    public Node getStartNode() {
        return  startNode;
    }

    public void setStartNode(Node node) {
        startNode = node;
    }

    public Node getEndNode() {
        return  endNode;
    }

    public void setEndNode(Node node) {
        endNode = node;
    }

    public void setDensity(double density) {
        this.density = density;
    }

    public double getDensity() {
        return  density;
    }

    public double getDensitySum() {
        return densitySum;
    }

    public void addVehicle(Vehicle vehicle) {
        if (!vehicles.contains(vehicle)) {
            vehicles.add(vehicle);
            if (SimLoop.isPopulated && !TimeController.isFutureSim) totalVehiclesAdded++;
        }
    }

    public void removeVehicle(Vehicle vehicle) {
        vehicles.remove(vehicle);
    }

    public ArrayList<Vehicle> getVehicles(){
        return vehicles;
    }

    public void setVehicles(ArrayList<Vehicle> vehicles) {
        this.vehicles = vehicles;
    }

    public String getNodesAsString() {
        return (startNode.getId() + " -> " + endNode.getId());
    }

    public int getTotalVehiclesAdded() {
        return totalVehiclesAdded;
    }
}
