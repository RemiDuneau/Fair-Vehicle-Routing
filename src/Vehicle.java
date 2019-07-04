import java.util.Stack;

public class Vehicle implements Cloneable {

    public static double VEHICLE_LENGTH = 5;

    public Vehicle(int routingType) {
        this.routingType = routingType;
    }

    public Vehicle () {}


    private int routingType;

    //state variables
    private Node startNode, endNode;
    private Stack<Road> path;
    private Road currentRoad;
    private int totalDistance, tripDistance, roadDistance, currentSpeed, unfairnessScore, tripsFinished = 0,
            optimalTripTime, actualTripTime = 0;
    private boolean isFinished;

    @Override
    /**
     * Clones the instance of this Vehicle - note it is not a deep clone and {@code currentRoad} and {@code path} are not properly cloned.
     * @return the clone
     */
    public Object clone() {
        Vehicle clone = null;
        try {
            clone = (Vehicle) super.clone();
        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
        }
        return clone;
    }

    //-------------- movement methods --------------

    public void move() {
        actualTripTime++;
        int oldRoadLength = currentRoad.getLength();

        //check if move onto new road
        int difference = oldRoadLength - (roadDistance + currentSpeed);
        if (difference < 0) { //move to new road
            if (path.isEmpty()) {
                isFinished = true;
                tripsFinished += 1;
            }
            if (path.isEmpty() || path.peek().getDensity() < Road.MAX_DENSITY) { //check if new road isn't full
                difference = -difference; //turn number positive
                double proportionOfTimeLeft = (double) difference / (double) currentSpeed;
                incrementDistances(oldRoadLength - roadDistance); //distance travelled on old road
                currentRoad.removeVehicle(this);

                if (!isFinished) {
                    //remove from old road, add to new road
                    Road newRoad = path.pop();
                    newRoad.addVehicle(this);

                    //update vars and finish trip for time increment
                    currentSpeed = newRoad.calculateCurrentSpeed();
                    int distanceOnNewRoad = (int) (currentSpeed * proportionOfTimeLeft);
                    resetRoadDistance();
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
        return  currentRoad;
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

    public int getActualTripTime() {
        return actualTripTime;
    }
}