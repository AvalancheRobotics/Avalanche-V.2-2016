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
        double x = Math.cos(angle) * (distanceTraveled - cmDrivenBeforeTurn);
        double y = Math.sin(angle) * (distanceTraveled - cmDrivenBeforeTurn);

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
        double x = Math.cos(angle) * distanceToObstacle;
        double y = Math.sin(angle) * distanceToObstacle;

        int obstacleX = (int) Math.round(lastPosition.getX() + x);
        int obstacleY = (int) Math.round(lastPosition.getY() + y);

        store.field[obstacleX][obstacleY].traversable = false;

        /** ADD STUFF FOR PREDICTING DEPTH OF OBSTACLES AFTER GAME IS ANNOUNCED.
         * AND WE'RE ABLE TO GUESS DEPTH OF OBSTACLES BASED ON GAME OBJECTS  */
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