import java.util.ArrayList;
import java.util.Stack;
public class Vehicle implements Cloneable {

    public static double VEHICLE_LENGTH = 5;

    public Vehicle(int routingType) {
        this.routingType = routingType;
    }

    public Vehicle () {}


    //DELETE LATER!!!!
    public Stack<Road> actualPath;
    public Stack<Road> dijkstraPath;
    public Stack<Road> optimalPath;
    public double estimatedTime;

    private int routingType;

    //state variables
    private Node startNode, endNode;
    private Stack<Road> path;
    private Road currentRoad;
    private int totalDistance = 0, tripDistance = 0, roadDistance = 0, currentSpeed, unfairnessScore, tripsFinished = 0,
            optimalTripTime, dijkstraTripTime = 0,  actualTripTime = 0;
    private boolean isFinished = false, isFutureSim = false;

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


    //---------------- FUTURE SIMULATION ----------------
    //current state vars
    private Stack<Road> simPath= new Stack<>();
    private Road simCurrentRoad;
    private int simRoadDistance, simTripDistance, simTotalDistance, simCurrentSpeed, simTripsFinished, simActualTripTime;
    private boolean simIsFinished;

    //-------------- movement methods --------------

    public void move() {

        Road tempCurrentRoad;
        Stack<Road> tempPath;
        int tempCurrentSpeed, tempTotalDistance, tempTripDistance, tempRoadDistance, tempTripsFinished, tempActualTripTime;
        boolean tempIsFinished;

        if (isFutureSim) {
            tempCurrentRoad = simCurrentRoad;
            tempPath = simPath;
            tempCurrentSpeed = simCurrentSpeed;
            tempRoadDistance = simRoadDistance;
            tempTripDistance = simTripDistance;
            tempTotalDistance = simTotalDistance;
            tempIsFinished = simIsFinished;
            tempTripsFinished = simTripsFinished;
            tempActualTripTime = simActualTripTime;
        }
        else {
            tempCurrentRoad = currentRoad;
            tempPath = path;
            tempCurrentSpeed = currentSpeed;
            tempRoadDistance = roadDistance;
            tempTripDistance = tripDistance;
            tempTotalDistance = totalDistance;
            tempIsFinished = isFinished;
            tempTripsFinished = tripsFinished;
            tempActualTripTime = actualTripTime;
        }


        tempActualTripTime++;
        int oldRoadLength = tempCurrentRoad.getLength();

        //check if move onto new road
        int difference = oldRoadLength - (tempRoadDistance + tempCurrentSpeed);
        if (difference < 0) { //move to new road
            if (tempPath.isEmpty()) {
                tempIsFinished = true;
                tempTripsFinished += 1;
            }
            if (tempPath.isEmpty() || tempPath.peek().getDensity() < Road.MAX_DENSITY) { //check if new road isn't full
                difference = -difference; //turn number positive
                double proportionOfTimeLeft = (double) difference / (double) tempCurrentSpeed;

                //add distance on old road
                int distanceOnOldRoad = (oldRoadLength - tempRoadDistance); //distance travelled on old road
                tempRoadDistance += distanceOnOldRoad;
                tempTripDistance += distanceOnOldRoad;
                tempTotalDistance += distanceOnOldRoad;
                tempCurrentRoad.removeVehicle(this);

                if (!tempIsFinished) {

                    //remove from old road, add to new road
                    Road newRoad = tempPath.pop();
                    newRoad.addVehicle(this);

                    //update vars and finish trip for time increment
                    tempCurrentSpeed = newRoad.calculateCurrentSpeed();
                    int distanceOnNewRoad = (int) (tempCurrentSpeed * proportionOfTimeLeft);
                    tempRoadDistance = 0;
                    tempRoadDistance += distanceOnNewRoad;
                    tempTripDistance += distanceOnNewRoad;
                    tempTotalDistance += distanceOnNewRoad;
                    tempCurrentRoad = newRoad;
                }
            }
        }
        else {
            tempRoadDistance += tempCurrentSpeed;
            tempTripDistance += tempCurrentSpeed;
            tempTotalDistance += tempCurrentSpeed;
        }

        //set old vars back
        if (isFutureSim) {
            simCurrentRoad = tempCurrentRoad;
            simPath = tempPath;
            simCurrentSpeed = tempCurrentSpeed;
            simRoadDistance = tempRoadDistance;
            simTripDistance = tempTripDistance;
            simTotalDistance = tempTotalDistance;
            simIsFinished = tempIsFinished;
            simTripsFinished = tempTripsFinished;
            simActualTripTime = tempActualTripTime;
        }
        else {
            currentRoad = tempCurrentRoad;
            path = tempPath;
            currentSpeed = tempCurrentSpeed;
            roadDistance = tempRoadDistance;
            tripDistance = tempTripDistance;
            totalDistance = tempTotalDistance;
            isFinished = tempIsFinished;
            tripsFinished = tempTripsFinished;
            actualTripTime = tempActualTripTime;
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

    public int calculateSimTimeIncrementsToFinishRoad() {
        return (int) Math.ceil((double) (simCurrentRoad.getLength()) / (double) simCurrentSpeed);
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
        return  (isFutureSim) ? simCurrentRoad : currentRoad;
    }

    public void setCurrentRoad(Road road) {
        currentRoad = road;
    }

    public void setSimCurrentRoad(Road road) {
        simCurrentRoad = road;
    }

    public void setCurrentSpeed(int speed) {
        currentSpeed = speed;
    }

    public void setSimCurrentSpeed(int speed) {
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

    public void setSimPath(Stack<Road> path) {
        simPath = path;
    }

    public Stack<Road> getPath() {
        return (isFutureSim) ? simPath : path;
    }

    public Stack<Road> getSimPath() {
        return simPath;
    }

    public boolean isFinished() {
        return (isFutureSim) ? simIsFinished : isFinished;
    }

    public boolean isSimFinished() {
        return simIsFinished;
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

    public int getSimTripDistance() {
        return simTripDistance;
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

    public int getSimActualTripTime() {
        return simActualTripTime;
    }
}