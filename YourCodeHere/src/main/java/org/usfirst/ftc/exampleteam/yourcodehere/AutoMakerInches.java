package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

/**                                          DESCRIPTION OF CLASS
 * the purpose of this class is to find the correct values that we need to program autonomous
 * this class runs as a teleop method, you can only drive straight and turn.
 * you can check distance you travel by hitting a button, it sends the distances back to the phone through telemetry.
 */

/**                           CONTROLS - ONLY USES DRIVER (GAMEPAD1) CONTROLLER
 Right Trigger - Move Forwards
 Left Trigger - Move Backwards
 Right Bumper - Turn Right
 Left Bumper - Turn Right
 Y - Reset Encoders to Current Position (Wipes Distances)
 X - Degrees Traveled (WILL NOT BE ACCURATE IF YOU MOVE FORWARDS AFTER RESETTING THE ENCODERS) - sends data to phone
 B - Distance Traveled in Inches - sends data to phone
 dpad_down - get correctedHeading

MAKE SURE YOU WIPE THE DISTANCE (HIT Y) AFTER YOU GET DATA FROM EACH STEP OR READINGS WILL NOT BE ACCURATE
 */


@TeleOp(name = "AutoMakeInches")
public class AutoMakerInches extends SynchronousOpMode {
    // Declare drive motors
    DcMotor motorLeftFore;
    DcMotor motorLeftAft;
    DcMotor motorRightFore;
    DcMotor motorRightAft;

    GyroSensor gyro;


    //Declare initial positions of those drive encoders
    private int initLeftFore;
    private int initLeftAft;
    private int initRightFore;
    private int initRightAft;

    //Declare constants
    public static final double TICKS_PER_INCH = 133.7;
    public static final double TICKS_PER_DEGREE_DRIVE = (51.84*TICKS_PER_INCH)/360;

    public static int offset;
    public static int drift;
    public static double startTime;


    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        // Initialize drive motors
        motorLeftFore = hardwareMap.dcMotor.get("lf");
        motorLeftAft = hardwareMap.dcMotor.get("lb");
        motorRightFore = hardwareMap.dcMotor.get("rf");
        motorRightAft = hardwareMap.dcMotor.get("rb");
        gyro = hardwareMap.gyroSensor.get("gyro");

        //Left and right motors are on opposite sides and must spin opposite directions to go forward
        motorLeftAft.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFore.setDirection(DcMotor.Direction.REVERSE);


        // Reset encoders
        this.motorLeftAft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorLeftFore.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorRightAft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorRightFore.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        //Set Runmode for all motors to run using encoders
        motorLeftFore.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRightFore.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeftAft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRightAft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        resetEncodersToCurrentPosition();
    }

    @Override
    public void main() throws InterruptedException {
        hardwareMapping();

        /////////////////////////////////////
        gyro.calibrate();                  //
        //
        while (gyro.isCalibrating())       // Calibrating Gyro
            Thread.sleep(50);              //
        //
        Thread.sleep(5000);                //
        drift = gyro.getHeading();         //
        /////////////////////////////////////

        waitForStart();

        ////////////////////////////Op mode started/////////////////////////////////////////////////

        offset = gyro.getHeading();
        startTime = System.nanoTime();

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (updateGamepads()) {
                if (gamepad1.left_trigger > gamepad1.right_trigger) {
                    setLeftDrivePower(-scaleInput(gamepad1.left_trigger));
                    setRightDrivePower(-scaleInput(gamepad1.left_trigger));
                } else {
                    setLeftDrivePower(scaleInput(gamepad1.right_trigger));
                    setRightDrivePower(scaleInput(gamepad1.right_trigger));
                }

            }

            //resets init motor values
            if (gamepad1.y) {
                resetEncodersToCurrentPosition();
            }

            //prints out difference between current position and init position
            //(how far motors have traveled in ticks)
            if (gamepad1.b) {
                printEncoderTravelFromInitPos();
            }

            //turn right
            if (gamepad1.right_bumper) {
                setLeftDrivePower(.2);
                setRightDrivePower(-.2);
            }

            if (gamepad1.left_bumper) {
                setLeftDrivePower(-.2);
                setRightDrivePower(.2);
            }

            if (gamepad1.x) {
                printDegreesTravelFromInitPos();
            }

            if (gamepad1.dpad_down) {
                getCorrectedHeading();
            }

            idle();
        }
    }

    private void setLeftDrivePower(double power) {
        motorLeftFore.setPower(power);
        motorLeftAft.setPower(power);
    }

    private void setRightDrivePower(double power) {
        motorRightFore.setPower(power);
        motorRightAft.setPower(power);
    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0)
            index = -index;
        if (index > 16)
            index = 16;


        double dScale;
        if (dVal < 0)
            dScale = -scaleArray[index];
        else
            dScale = scaleArray[index];

        return dScale;
    }


    //changes encoder init values to current position
    private void resetEncodersToCurrentPosition() {
        initLeftFore = motorLeftFore.getCurrentPosition();
        initLeftAft = motorLeftAft.getCurrentPosition();
        initRightFore = motorRightFore.getCurrentPosition();
        initRightAft = motorRightAft.getCurrentPosition();
    }

    //prints how much the encoder values have changed from the set init position
    private void printEncoderTravelFromInitPos() {
        telemetry.addData("left side inches", (motorLeftAft.getCurrentPosition() - initLeftAft) / TICKS_PER_INCH);
        telemetry.addData("right side inches", (motorRightAft.getCurrentPosition() - initRightAft) / TICKS_PER_INCH);
        telemetry.update();
    }

    private void printDegreesTravelFromInitPos() {
        double degrees = (Math.abs((motorLeftAft.getCurrentPosition() - initLeftAft) / TICKS_PER_DEGREE_DRIVE) + Math.abs((motorRightAft.getCurrentPosition() - initRightAft) / TICKS_PER_DEGREE_DRIVE)) / 2;
        telemetry.addData("degrees rotated (always positive)", degrees);
        telemetry.update();
    }


    public int getCorrectedHeading(){
        double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
        int totalDrift = (int)(elapsedSeconds / 5 * drift);
        int targetHeading = gyro.getHeading() - offset - totalDrift;
        while(targetHeading > 359)               //
            targetHeading = targetHeading - 360; // Allows value to "wrap around"
        while(targetHeading < 0)                 // since values can only be 0-359
            targetHeading = targetHeading + 360; //
        return targetHeading;
    }

}
