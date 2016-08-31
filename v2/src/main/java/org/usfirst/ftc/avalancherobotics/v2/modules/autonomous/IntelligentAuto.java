package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.usfirst.ftc.avalancherobotics.v2.modules.DriveTrainController;

/**
 * Intelligent auto which uses pathfinding algorithms and sensors
 * to direct robot to given locations on fields.
 */

@Autonomous(name = "Intelligent Auto")
public class IntelligentAuto extends SynchronousOpMode {
    private Store store;


    @Override
    protected void main() throws InterruptedException {
        store = new Store();

        store.field = InitialField.generateField();
        store.pathfinder = new AStarPathfinder(store);
        store.pathrover = new ParsePath(store);

        store.telemetry = telemetry;

        // Initialize sensors
        store.gyro = hardwareMap.gyroSensor.get("gyro");

        // Initalize Drivetrain
        store.driveTrain = new DriveTrainController(hardwareMap.dcMotor.get("LeftBack"), hardwareMap.dcMotor.get("RightBack"), hardwareMap.dcMotor.get("LeftFront"), hardwareMap.dcMotor.get("RightFront"));


        /////////////////////////////////////////
        store.gyro.calibrate();                //
                                               //
        while (store.gyro.isCalibrating())     // Calibrating Gyro
            Thread.sleep(50);                  //
                                               //
        Thread.sleep(5000);                    //
        store.drift = store.gyro.getHeading(); //
        /////////////////////////////////////////

        store.range = hardwareMap.i2cDevice.get("range");
        store.rangeReader = new I2cDeviceReader(store.range, 0x28, 0x04, 2);
        store.rangeReadings = store.rangeReader.getReadBuffer();

        store.scanner = new Scanner(store);

        telemetry.addData("Done calibrating", getCorrectedHeading());
        telemetry.update();

        waitForStart();
        store.startTime = System.nanoTime();

        store.offset = store.gyro.getHeading();

        //STARTING POSITION
        store.lastPosition = new Location(1,1);


        while (opModeIsActive()) {
            /** TESTING */


            while (!store.pathrover.driveToTarget(new Location(10, 10)) && opModeIsActive()) {
                idle();
            }

            store.driveTrain.setLeftDrivePower(0);
            store.driveTrain.setRightDrivePower(0);
            telemetry.addData("finished", "done");
            telemetry.update();
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

    public int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - store.startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * store.drift);
        int targetHeading = store.gyro.getHeading() - store.offset - totalDrift;
        while (targetHeading > 359)               //
            targetHeading = targetHeading - 360; // Allows value to "wrap around"
        while (targetHeading < 0)                 // since values can only be 0-359
            targetHeading = targetHeading + 360; //
        return targetHeading;
    }

    public void moveToLocation(Location destination) {
        store.scanner.update(store.driveTrain.distanceTraveledBeforeReset(), getCorrectedHeading(), calibratedUltrasonic(store.rangeReader.getReadBuffer()[0]), store.driveTrain);
        store.pathfinder.findPath(store.lastPosition, destination);
    }
}
