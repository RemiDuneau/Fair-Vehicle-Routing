import java.util.HashMap;
import java.util.Map;
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


    //DELETE LATER!!!! :)
    public Stack<Road> actualPath = new Stack<>();
    public Stack<Road> dijkstraPath;
    public Stack<Road> optimalPath;
    public double estimatedFutureTime;

    private int routingType;

    //state variables
    private Node startNode, endNode;
    private Stack<Road> path;
    private Road currentRoad;
    private double unfairness = 0, worstTrip = 0;
    private int totalDistance = 0, tripDistance = 0, roadDistance = 0, currentSpeed, numTrips = 0,
            optimalTripTime, dijkstraTripTime = 0,  actualTripTime = 0, totalTripTime = 0, estimatedDijkstraTime = 0, estimatedTripTime,
            failedAddattempts;
    private boolean isFinished = false, isStarted = false, isDynamicRouting, isDijkstraOnly = false;
    private Map<Road, Integer> roadsTakenMap = new HashMap<>();

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

    /**
     * NOT FUTURE SIM SAFE BECAUSE NUMTRIPS INCREASES WHEN VEHICLE FINISHES, AND MAP IS UPDATED.
     */
    public void move() {
        actualTripTime++;
        totalTripTime++;
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
                numTrips += 1;
                updateRoadsTakenMap();
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

    private void updateRoadsTakenMap() {
        for (Road road : actualPath) {
            if (roadsTakenMap.containsKey(road))
                roadsTakenMap.put(road, roadsTakenMap.get(road)+1);
            else roadsTakenMap.put(road, 1);
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

    public double getUnfairness() {
        return unfairness;
    }

    public void setUnfairness(double unfairness) {
        this.unfairness = unfairness;
    }

    public double getWorstTrip() {
        return worstTrip;
    }

    public void setWorstTrip(double worstTrip) {
        this.worstTrip = worstTrip;
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

    public int getNumTrips() {
        return numTrips;
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

    public void setActualTripTime(int actualTripTime) {
        this.actualTripTime = actualTripTime;
    }

    public int getTotalTripTime() {
        return totalTripTime;
    }

    public double getAverageTripTime() {
        return totalTripTime / (double) numTrips;
    }

    public void setTotalTripTime(int totalTripTime) {
        this.totalTripTime = totalTripTime;
    }

    public int getEstimatedDijkstraTime() {
        return estimatedDijkstraTime;
    }

    public void setEstimatedDijkstraTime(int estimatedDijkstraTime) {
        this.estimatedDijkstraTime = estimatedDijkstraTime;
    }

    public int getEstimatedTripTime() {
        return estimatedTripTime;
    }

    public void setEstimatedTripTime(int estimatedTripTime) {
        this.estimatedTripTime = estimatedTripTime;
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

    public boolean isDijkstraOnly() {
        return isDijkstraOnly;
    }

    public void setDijkstraOnly(boolean dijkstraOnly) {
        isDijkstraOnly = dijkstraOnly;
    }


    public int getFailedAddattempts() {
        return failedAddattempts;
    }

    public void setFailedAddattempts(int failedAddattempts) {
        this.failedAddattempts = failedAddattempts;
    }

    public Map<Road, Integer> getRoadsTakenMap() {
        return roadsTakenMap;
    }
}