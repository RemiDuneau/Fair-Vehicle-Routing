import java.util.ArrayList;
import java.util.EmptyStackException;
import java.util.Stack;
public class Vehicle implements Cloneable {

    public static double VEHICLE_LENGTH = 5;

    public Vehicle(int routingType) {
        this.routingType = routingType;
        this.isDynamicRouting = false;
    }

    public Vehicle(boolean isDynamicRouting, TimeController controller) {
        this.isDynamicRouting = isDynamicRouting;
        this.controller = controller;
    }

    public Vehicle(int routingType, boolean isDynamicRouting, TimeController controller) {
        this.routingType = routingType;
        this.isDynamicRouting = isDynamicRouting;
        this.controller = controller;
    }


    public Vehicle () {}


    //DELETE LATER!!!!
    public Stack<Road> actualPath = new Stack<>();
    public Stack<Road> dijkstraPath;
    public Stack<Road> optimalPath;
    public double estimatedFutureTime;

    private int routingType;

    //state variables
    private Node startNode, endNode;
    private Stack<Road> path;
    private Road currentRoad;
    private int totalDistance = 0, tripDistance = 0, roadDistance = 0, currentSpeed, unfairnessScore, tripsFinished = 0,
            optimalTripTime, dijkstraTripTime = 0,  actualTripTime = 0;
    private boolean isFinished = false, isStarted = false, isDynamicRouting;

    private TimeController controller;

    @Override
    /**
     * Clones the instance of this Vehicle, and creates a shallow copied instance of {@code path}.
     * @return the clone
     */
    public Object clone() {
        Vehicle clone = null;
        try {
            clone = (Vehicle) super.clone();

            //create copy of path
            Stack<Road> pathCopy = new Stack<>();
            pathCopy.addAll(path);
            clone.setPath(pathCopy);
        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
        } catch (NullPointerException e) { }
        return clone;
    }

    public void move() {
        actualTripTime++;
        int oldRoadLength = currentRoad.getLength();

        //check if move onto new road
        int difference = oldRoadLength - (roadDistance + currentSpeed);
        if (difference < 0) { //move to new road

            //get new path if dynamic routing
            if (isDynamicRouting) {
                path = controller.findPath(this, currentRoad.getEndNode(), endNode);
            }

            //check if finished
            if (currentRoad.getEndNode() == endNode) {
                isFinished = true;
                tripsFinished += 1;
            }

            if (path.isEmpty() && !isFinished) {
                //this happens in dynamic routing when all roads from current node are full, so path becomes an empty stack.
            }

            else if (path.isEmpty() || path.peek().getDensity() < Road.MAX_DENSITY) { //check if new road isn't full
                difference = -difference; //turn number positive
                double proportionOfTimeLeft = (double) difference / (double) currentSpeed;

                //add distance on old road
                int distanceOnOldRoad = (oldRoadLength - roadDistance);
                incrementDistances(distanceOnOldRoad);
                currentRoad.removeVehicle(this);

                if (!isFinished) {

                    //remove from old road, add to new road
                    Road newRoad = path.pop();
                    newRoad.addVehicle(this);

                    //update vars and finish trip for time increment
                    currentSpeed = newRoad.calculateCurrentSpeed();
                    int distanceOnNewRoad = (int) (currentSpeed * proportionOfTimeLeft);
                    roadDistance = 0;
                    incrementDistances(distanceOnNewRoad);
                    currentRoad = newRoad;
                }
            }
        }
        else {
            incrementDistances(currentSpeed);
        }
    }

    /**
     * Calculates the percentage difference between the time taken if the vehicle had taken the optimal route with no congestion
     * compared to the time the vehicle took to actually complete the stuff.
     * E.g. a value of 20.0 would mean the actual time taken was 20% more than the optimal 0 congestion time.
     * @return The % difference between the optimal time taken and the actual time taken
     */
    public double calculateOptimalTimeDifference() {
        return ( (double) (actualTripTime - optimalTripTime) / (double) optimalTripTime ) * 100;
    }

    public double calculateDijkstraTimeDifference() {
        return ( (double) (actualTripTime - dijkstraTripTime) / (double) dijkstraTripTime ) * 100;
    }

    public int calculateTimeIncrementsToFinishRoad() {
        return (int) Math.ceil((double) (currentRoad.getLength()) / (double) currentSpeed);
    }
    private void incrementDistances(int distance) {
        roadDistance += distance;
        tripDistance += distance;
        totalDistance += distance;
    }

    public void resetRoadDistance() {
        roadDistance = 0;
    }

    public void resetTripDistance() {
        tripDistance = 0;
    }


    public Road getCurrentRoad() {
        return currentRoad;
    }

    public void setCurrentRoad(Road road) {
        currentRoad = road;
    }

    public void setCurrentSpeed(int speed) {
        currentSpeed = speed;
    }

    public int getCurrentSpeed() {
        return currentSpeed;
    }

    public void setStartNode(Node start) {
        this.startNode = start;
    }

    public Node getStartNode() {
        return startNode;
    }

    public void setEndNode(Node end) {
        this.endNode = end;
    }

    public Node getEndNode() {
        return endNode;
    }

    public int getRoutingType() {
        return routingType;
    }

    public void setRoutingType(int routingType) {
        this.routingType = routingType;
    }

    public int getUnfairnessScore() {
        return unfairnessScore;
    }

    public void setPath(Stack<Road> path) {
        this.path = path;
    }

    public Stack<Road> getPath() {
        return path;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public void setFinished(boolean b) {
        isFinished = b;
    }

    public boolean isStarted() {
        return isStarted;
    }

    public void setStarted(boolean started) {
        isStarted = started;
    }

    public int getTotalDistance() {
        return totalDistance;
    }

    public int getTripDistance() {
        return tripDistance;
    }

    public int getRoadDistance() {
        return roadDistance;
    }

    public int getTripsFinished() {
        return tripsFinished;
    }

    public int getOptimalTripTime() {
        return optimalTripTime;
    }

    public void setOptimalTripTime(int time) {
        optimalTripTime = time;
    }

    public int getDijkstraTripTime() {
        return dijkstraTripTime;
    }

    public void setDijkstraTripTime(int time) {
        dijkstraTripTime = time;
    }

    public int getActualTripTime() {
        return actualTripTime;
    }

    public boolean isDynamicRouting() {
        return isDynamicRouting;
    }

    public void setDynamicRouting(boolean b) {
        isDynamicRouting = b;
    }

    public void setTimeController(TimeController controller) {
        this.controller = controller;
    }
}