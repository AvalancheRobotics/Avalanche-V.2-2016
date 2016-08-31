package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;


import org.usfirst.ftc.avalancherobotics.v2.modules.DriveTrainController;

/**
 * This class is used to build a 2d array based off of pre-known information about a surrounding as well as scanning the environment with ultrasonic sensors
 */
public class Scanner {
    private Store store;

    private Location lastPosition;

    private int lastAngle;
    private int cmDrivenBeforeTurn;

    public Scanner(Store store) {
        this.store = store;
        lastPosition = Store.STARTING_POSITION;
        lastAngle = getCorrectedHeading();
        cmDrivenBeforeTurn = store.driveTrain.distanceTraveledBeforeReset();
    }


    public void update(int distanceTraveled, int angle, int distanceToObstacle, DriveTrainController driveTrain) {
        //Update position of robot
        double x = Math.cos(angle) * (distanceTraveled - cmDrivenBeforeTurn) / 3;
        double y = Math.sin(angle) * (distanceTraveled - cmDrivenBeforeTurn) / 3;

        //If the robot has turned since the last update
        //(A turn will currently throw off the distance (no odometer wheel))
        if (lastAngle > angle - 1 && lastAngle < angle + 1) {
            driveTrain.resetEncoders(); /** EVENTUALLY REPLACE WITH ODOMETER WHEEL STUFF **/
            cmDrivenBeforeTurn = 0;
            lastAngle = angle;
            return;
        }

        int newX = (int) Math.round(lastPosition.getX() + x);
        int newY = (int) Math.round(lastPosition.getY() + y);

        if (!(newX == lastPosition.getX() && newY == lastPosition.getY())) {
            lastPosition = new Location(newX, newY);
            cmDrivenBeforeTurn = distanceTraveled;
        }

        updateObstacles(angle, distanceToObstacle);
    }

    private void updateObstacles(int angle, int distanceToObstacle) {
        if (distanceToObstacle != -1) {
            double x = Math.cos(angle) * distanceToObstacle / 3;
            double y = Math.sin(angle) * distanceToObstacle / 3;

            int obstacleX = (int) Math.round(lastPosition.getX() + x);
            int obstacleY = (int) Math.round(lastPosition.getY() + y);

            if (obstacleX > 0 && obstacleX < 124 && obstacleY > 0 && obstacleY < 124) {
                store.field[obstacleX][obstacleY].traversable = false;
            }

            /** ADD STUFF FOR PREDICTING DEPTH OF OBSTACLES AFTER GAME IS ANNOUNCED.
             * AND WE'RE ABLE TO GUESS DEPTH OF OBSTACLES BASED ON GAME OBJECTS  */
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
}