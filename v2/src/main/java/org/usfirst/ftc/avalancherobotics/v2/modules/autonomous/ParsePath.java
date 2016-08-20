package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;


import java.util.LinkedList;

/**
 * Takes in a path and uses positioning and motor movements to trace that path
 */
public class ParsePath {
    private Store store;

    private LinkedList<Location> simplifiedPath;
    private LinkedList<Direction> directionPath;

    public ParsePath(Store store) {
        this.store = store;
    }

    public boolean driveToTarget(Location target) {
        LinkedList<Location> pathCoordList = store.pathfinder.findPath(store.lastPosition, target);

        if (pathCoordList == null) {
            //Keep scanning environment until path opens up
            pivotToAngle(getCorrectedHeading() + 12);
        }

        simplifyPath(pathCoordList);


        if (simplifiedPath.size() == 0) {
            return true;
        }


        if (store.lastPosition.getX() > simplifiedPath.get(0).getX() - 1 //If within 1 cm of target
                && store.lastPosition.getX() < simplifiedPath.get(0).getX() + 1
                && store.lastPosition.getY() > simplifiedPath.get(0).getY() - 1
                && store.lastPosition.getY() < simplifiedPath.get(0).getY() + 1
                ) {
            simplifiedPath.remove(0); //Remove the node
            directionPath.remove(0);
            return false;
        }

        //Pivot to direction of target
        if (directionPath.get(0).equals(Direction.RIGHT)) {
            pivotToAngle(0);
        } else if (directionPath.get(0).equals(Direction.UP_RIGHT)) {
            pivotToAngle(45);
        } else if (directionPath.get(0).equals(Direction.UP)) {
            pivotToAngle(90);
        } else if (directionPath.get(0).equals(Direction.UP_LEFT)) {
            pivotToAngle(135);
        } else if (directionPath.get(0).equals(Direction.LEFT)) {
            pivotToAngle(180);
        } else if (directionPath.get(0).equals(Direction.DOWN_LEFT)) {
            pivotToAngle(225);
        } else if (directionPath.get(0).equals(Direction.DOWN)) {
            pivotToAngle(270);
        } else if (directionPath.get(0).equals(Direction.DOWN_RIGHT)) {
            pivotToAngle(315);
        }

        moveForwardOnHeading(simplifiedPath.get(0));

        return false;
    }


    // Takes in a path (list of locations) and simplifies it so that only points
    // where directions change are listed.
    public void simplifyPath(LinkedList<Location> oldPath) {
        simplifiedPath = new LinkedList<>();
        directionPath = new LinkedList<>();

        if (oldPath.size() == 0) {
            return;
        }


        Location lastLocation = oldPath.get(0);
        Direction lastDirection = null;

        for (int i = 1; i < oldPath.size(); i++) {
            int lastX = lastLocation.getX();
            int lastY = lastLocation.getY();
            int newX = oldPath.get(i).getX();
            int newY = oldPath.get(i).getY();

            Direction currentDirection = null;
            //x moves right
            if (newX > lastX) {
                if (newY > lastY) {
                    currentDirection = Direction.UP_RIGHT;
                } else if (newY == lastY) {
                    currentDirection = Direction.RIGHT;
                } else if (newY < lastY) {
                    currentDirection = Direction.DOWN_RIGHT;
                }
            } else if (newX == lastX) { //x stays put
                if (newY > lastY) {
                    currentDirection = Direction.UP;
                } else if (newY < lastY) {
                    currentDirection = Direction.DOWN;
                }
            } else if (newX < lastX) { //x moves left
                if (newY > lastY) {
                    currentDirection = Direction.UP_LEFT;
                } else if (newY == lastY) {
                    currentDirection = Direction.LEFT;
                } else if (newY < lastY) {
                    currentDirection = Direction.DOWN_LEFT;
                }
            }

            if (lastDirection == null) {
                directionPath.add(currentDirection);
            }
            else {
                if (currentDirection != null && !currentDirection.equals(lastDirection) || i == oldPath.size() - 1) {
                    simplifiedPath.add(oldPath.get(i));
                    directionPath.add(currentDirection);
                }
            }

            lastDirection = currentDirection;
        }

    }

    private int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - store.startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * store.drift);
        int targetHeading = store.gyro.getHeading() - store.offset - totalDrift;
        while (targetHeading > 359)               //
            targetHeading = targetHeading - 360; // Allows value to "wrap around"
        while (targetHeading < 0)                 // since values can only be 0-359
            targetHeading = targetHeading + 360; //
        return targetHeading;
    }

    public void heading(int deg) {
        double power;
        double proportionalConst = 0.004;

        double topCeiling = 1;
        double bottomCeiling = -1;
        double topFloor = .2;
        double bottomFloor = -.2;

        int target = getCorrectedHeading() + deg;
        while (target > 359)
            target = target - 360;
        while (target < 0)
            target = target + 360;

        while (getCorrectedHeading() != target) {
            power = (target - getCorrectedHeading()) * proportionalConst;

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;

            store.driveTrain.setRightDrivePower(power);
            store.driveTrain.setRightDrivePower(-power);
        }

        store.driveTrain.setRightDrivePower(0);
        store.driveTrain.setLeftDrivePower(0);
    }

    public void pivotToAngle(int angle) {
        int heading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = 1;
        double bottomCeiling = -1;
        double topFloor = .2;
        double bottomFloor = -.2;

        int target = angle;
        while (target > 359)
            target = target - 360;
        while (target < 0)
            target = target + 360;

        while (heading != target) {
            power = (target - heading) * proportionalConst;

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;


            boolean tarGreater = target - heading > 0;

            if ((tarGreater && target - heading > 180) || (!tarGreater && target - heading < 180)) {
                store.driveTrain.setRightDrivePower(power);
                store.driveTrain.setLeftDrivePower(-power);
            } else {
                store.driveTrain.setRightDrivePower(-power);
                store.driveTrain.setLeftDrivePower(power);
            }

        }

        store.driveTrain.setRightDrivePower(0);
        store.driveTrain.setLeftDrivePower(0);
    }

    public void moveForwardOnHeading(Location targetLocation) {
        Location currentLocation = store.lastPosition;

        double distance = Math.round(Math.sqrt(Math.pow(targetLocation.getX() - currentLocation.getX(), 2) + Math.pow(targetLocation.getY() - currentLocation.getY(), 2)));

        int ticks = (int) (Store.TICKS_PER_CM * distance);

        double power;
        double proportionalConst = 0.004;

        double topCeiling = 1;
        double bottomCeiling = -1;
        double topFloor = .12;
        double bottomFloor = -.12;

        if (distance > 0) {
            power = ticks * proportionalConst;

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;

            store.driveTrain.setLeftDrivePower(power);
            store.driveTrain.setRightDrivePower(power);

        }
    }
}
