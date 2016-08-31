package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;


import android.os.Debug;
import android.util.Log;

import java.util.LinkedList;

/**
 * Takes in a path and uses positioning and motor movements to trace that path
 */
public class ParsePath {
    private Store store;


    public ParsePath(Store store) {
        this.store = store;
    }

    public boolean driveToTarget(Location target) {
        //Update Field with new obstacles and update current position
        store.scanner.update(store.driveTrain.distanceTraveledBeforeReset(), getCorrectedHeading(), calibratedUltrasonic(store.rangeReader.getReadBuffer()[0]), store.driveTrain);

        //Update AStar Path
        LinkedList<Location> pathCoordList = store.pathfinder.findPath(store.lastPosition, target);

        //If there isn't currently a path keep looking for one
        if (pathCoordList == null) {
            //Keep scanning environment until path opens up
            store.telemetry.addData("No path", "scanning");
            store.telemetry.update();
            pivotToAngle(getCorrectedHeading() + 12);
            store.scanner.update(store.driveTrain.distanceTraveledBeforeReset(), getCorrectedHeading(), calibratedUltrasonic(store.rangeReader.getReadBuffer()[0]), store.driveTrain);
            pathCoordList = store.pathfinder.findPath(store.lastPosition, target);
        }

        LinkedList<Location> simplifiedPath = new LinkedList<>();
        LinkedList<Direction> directionPath = new LinkedList<>();

        //Simplify the path
        simplifyPath(pathCoordList, simplifiedPath, directionPath);

        //Target has been reached
        if (simplifiedPath.size() == 0) {
            return true;
        }

        //If within 1 cm of next turn/node remove the node
        //The positions are close enough so we can be considered on the node
        if (store.lastPosition.getX() > simplifiedPath.get(0).getX() - 1
                && store.lastPosition.getX() < simplifiedPath.get(0).getX() + 1
                && store.lastPosition.getY() > simplifiedPath.get(0).getY() - 1
                && store.lastPosition.getY() < simplifiedPath.get(0).getY() + 1
                ) {
            simplifiedPath.remove(0); //Remove the node
            directionPath.remove(0);
            return false;
        }

        store.telemetry.addData("path", directionPath);
        store.telemetry.update();

        //Pivot to direction of target
        if (directionPath.get(0).equals(Direction.RIGHT)) {
            pivotToAngle(0);
            //store.telemetry.addData("pivoted: ", 0);
        } else if (directionPath.get(0).equals(Direction.UP_RIGHT)) {
            pivotToAngle(45);
            //store.telemetry.addData("pivoted: ", 45);
        } else if (directionPath.get(0).equals(Direction.UP)) {
            pivotToAngle(90);
            //store.telemetry.addData("pivoted: ", 90);
        } else if (directionPath.get(0).equals(Direction.UP_LEFT)) {
            pivotToAngle(135);
            //store.telemetry.addData("pivoted: ", 135);
        } else if (directionPath.get(0).equals(Direction.LEFT)) {
            pivotToAngle(180);
            //store.telemetry.addData("pivoted: ", 180);
        } else if (directionPath.get(0).equals(Direction.DOWN_LEFT)) {
            pivotToAngle(225);
            //store.telemetry.addData("pivoted: ", 225);
        } else if (directionPath.get(0).equals(Direction.DOWN)) {
            pivotToAngle(270);
            //store.telemetry.addData("pivoted: ", 270);
        } else if (directionPath.get(0).equals(Direction.DOWN_RIGHT)) {
            pivotToAngle(315);
            //store.telemetry.addData("pivoted: ", 315);
        }
        //store.telemetry.update();

        store.telemetry.addData("location", store.lastPosition);
        store.telemetry.update();
        //Move towards the target
        moveForwardOnHeading(simplifiedPath.get(0));

        return false;
    }


    // Takes in a path (list of locations) and simplifies it so that only points
    // where directions change are listed.
    public void simplifyPath(LinkedList<Location> oldPath, LinkedList<Location> simplifiedPath, LinkedList<Direction> directionPath) {

        if (oldPath == null) {
            return;
        }
        if (oldPath.size() == 0) {
            return;
        }


        Location lastLocation = oldPath.get(0);
        Direction lastDirection = null;

        while (oldPath.size() != 0) {

            int lastX = lastLocation.getX();
            int lastY = lastLocation.getY();
            int newX = oldPath.get(0).getX();
            int newY = oldPath.get(0).getY();



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
            } else {
                if ((currentDirection != null && !currentDirection.equals(lastDirection)) || oldPath.size() == 1) {
                    simplifiedPath.add(oldPath.get(0));
                    directionPath.add(currentDirection);
                }
            }
            lastLocation = oldPath.remove(0);
            lastDirection = currentDirection;
        }

        directionPath.remove(0);

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

            //store.telemetry.addData("angle", getCorrectedHeading());
            //store.telemetry.update();

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

            power = Math.abs((target - heading) * proportionalConst);

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


            heading = getCorrectedHeading();
        }

        store.driveTrain.setRightDrivePower(0);
        store.driveTrain.setLeftDrivePower(0);
    }

    public void moveForwardOnHeading(Location targetLocation) {
        Location currentLocation = store.lastPosition;

        double distance = Math.round(Math.sqrt(Math.pow((targetLocation.getX() - currentLocation.getX()) * 3, 2) + Math.pow((targetLocation.getY() - currentLocation.getY()) * 3, 2)));

        int ticks = (int) (Store.TICKS_PER_CM * distance);

        double power;
        double proportionalConst = 0.0003;

        double topCeiling = 1;
        double bottomCeiling = -1;
        double topFloor = .08;
        double bottomFloor = -.08;

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


    public int calibratedUltrasonic(byte reading) {
        //convert byte to int
        int uncalibratedDistance = reading;
        if (uncalibratedDistance == -1) {
            return -1;
        }

        int calibratedDistance;

        if (uncalibratedDistance >= 0) {
            return (int) (((double) uncalibratedDistance / 9) * 10);
        } else {
            calibratedDistance = uncalibratedDistance * -1;
            calibratedDistance = 255 - calibratedDistance;
            return (int) (((double) calibratedDistance / 9) * 10);
        }
    }
}
