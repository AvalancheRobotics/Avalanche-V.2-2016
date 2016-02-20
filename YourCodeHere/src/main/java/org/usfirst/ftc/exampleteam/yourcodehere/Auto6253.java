package org.usfirst.ftc.exampleteam.yourcodehere;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.*;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

/**
 * Autonomous modes:
 * <p/>
 * test()
 * set()
 * park(boolean blue)
 * climb(boolean blue)
 * <p/>
 * Autonomous support methods:
 * <p/>
 * raiseArm() < --- NOT TESTED
 * lowerArm() < --- NOT TESTED
 * scoreClimbers(boolean left) < --- NOT TESTED
 * moveForwardOnHeading(int distance)
 * moveForward(double power, int distance)
 * pivot(double power, int deg)
 * turnWithLeftSide(double power, int deg)
 * turnWithRightSide(double power, int deg)
 * heading(int deg)  < --- DOES NOT ALWAYS PIVOT THE RIGHT WAY
 * <p/>
 * Low level support methods:
 * <p/>
 * getCorrectedHeading(int offset, int drift, double startTime)
 * setArmHeight(double power, int deg) < --- NOT TESTED
 * extendSlides()
 * retractSlides()
 * releaseClimbers(boolean left)
 * setAllDrivePower(double power)
 * pivot(double power)
 * setLeftDrivePower(double power)
 * setRightDrivePower(double power)
 */

@Autonomous(name = "AutoBlue")
public class Auto6253 extends SynchronousOpMode {
    public static final double TICKS_PER_INCH = 133.7;
    public static final double TICKS_PER_DEGREE_DRIVE = (51.84 * TICKS_PER_INCH) / 360;
    public static final double TICKS_PER_DEGREE_ARM = ((24 / 16) * 1120) / 360;
    public static int offset;
    public static int drift;
    public static double startTime;


    //Servo Values
    private static final double RIGHT_ZIP_UP = 0.753;
    private static final double RIGHT_ZIP_DOWN = 0;
    private static final double LEFT_ZIP_UP = 0.293;
    private static final double LEFT_ZIP_DOWN = 0.216667;
    private static final double LOCK_ENGAGED = 1.0;
    private static final double LOCK_DISENGAGED = .178333;
    private static final double SHELF_STOW_LEFT = .5;
    private static final double SHELF_STOW_RIGHT = .75;
    private static final double SHELF_DISPENSE_LEFT = .40;
    private static final double SHELF_DISPENSE_RIGHT = .60;
    private static final double DISPENSER_NEUTRAL = 0.5;
    private static final double DISPENSER_LEFT = .593666;
    private static final double DISPENSER_RIGHT = 0.3577;

    // Declare drive motors
    DcMotor motorLeftFore;
    DcMotor motorLeftAft;
    DcMotor motorRightFore;
    DcMotor motorRightAft;

    // Declare drawer slide motor and servos
    // motor extends/retracts slides
    // servoSlide(continuous) slides the deposit box laterally
    // servoBlockRelease(s) open flaps on the bottom of the bucket, releasing blocks/climbers
    DcMotor motorSlide;
    Servo servoTilt;

    // Declare tape measure motor and servo
    // motor extends/retracts tape
    // servo(continuous) angles tape
    DcMotor motorTape;
    Servo servoTape;

    // Declare motor that spins the harvester
    DcMotor motorHarvest;

    // Declare motor that raises and lowers the collection arm
    DcMotor motorArm;

    // Declare zipline flipping servos
    Servo servoLeftZip;
    Servo servoRightZip;

    // Declare ramp servos
    Servo servoLeftRamp;
    Servo servoRightRamp;

    Servo servoLock;

    // Declare sensors
    //  GyroSensor gyro;

    @Override
    public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */

        // Initialize drive motors
        motorLeftFore = hardwareMap.dcMotor.get("LeftFore");
        motorLeftAft = hardwareMap.dcMotor.get("LeftAft");
        motorRightFore = hardwareMap.dcMotor.get("RightFore");
        motorRightAft = hardwareMap.dcMotor.get("RightAft");

        motorRightFore.setDirection(DcMotor.Direction.REVERSE);
        motorRightAft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize drawer slide motor and servos
        motorSlide = hardwareMap.dcMotor.get("Slide");
        servoTilt = hardwareMap.servo.get("Tilt");
        servoTilt.setPosition(DISPENSER_NEUTRAL);

        // Initialize tape measure motor and servo
        motorTape = hardwareMap.dcMotor.get("Tape");
        servoTape = hardwareMap.servo.get("TapeAngle");

        // Initialize motor that spins the harvester
        motorHarvest = hardwareMap.dcMotor.get("Harvest");

        // Initialize motor that raises and lowers the collection arm
        motorArm = hardwareMap.dcMotor.get("Arm");
        motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorArm.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorArm.setTargetPosition(-2335);
        motorArm.setPower(.3);

        // Initialize zipline flipping servos
        servoLeftZip = hardwareMap.servo.get("LeftZip");
        servoRightZip = hardwareMap.servo.get("RightZip");
        servoLeftZip.setPosition(LEFT_ZIP_UP);
        servoRightZip.setPosition(RIGHT_ZIP_UP);

        // Initialize ramp servos
        servoLeftRamp = hardwareMap.servo.get("LeftRamp");
        servoRightRamp = hardwareMap.servo.get("RightRamp");
        servoLeftRamp.setPosition(SHELF_DISPENSE_LEFT);
        servoRightRamp.setPosition(SHELF_DISPENSE_RIGHT);

        servoLock = hardwareMap.servo.get("Lock");
        servoLock.setPosition(LOCK_DISENGAGED);




        // Initialize sensors
            /*gyro = hardwareMap.gyroSensor.get("gyro");


            /////////////////////////////////////
            gyro.calibrate();                  //
                                               //
            while (gyro.isCalibrating())       // Calibrating Gyro
                Thread.sleep(50);              //
                                               //
            Thread.sleep(5000);                //
            drift = gyro.getHeading();         //
            /////////////////////////////////////
            */


        waitForStart();

        ////////////////////////////Op mode started/////////////////////////////////////////////////

        //offset = gyro.getHeading();
        startTime = System.nanoTime();


        boolean blue = true;
        v2AutoSquare(blue);

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                              Autonomous modes                                              //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public void park(boolean blue) throws InterruptedException {
        lowerArm();
        motorHarvest.setPower(-1);
        moveForwardOnHeading(60);
        if (blue)
            heading(90);
        else
            heading(-90);
        moveForwardOnHeading(48);
        motorHarvest.setPower(0);
        raiseArm();
    }

    public void climb(boolean blue) throws InterruptedException {
        lowerArm();
        motorHarvest.setPower(-1);
        moveForwardOnHeading(42);
        if (blue)
            heading(-135);
        else
            heading(135);
        moveForwardOnHeading(-22);
        if (blue)
            heading(90);
        else
            heading(-90);
        motorHarvest.setPower(0);
        moveForwardOnHeading(-108);
    }

    public void v2Auto(boolean blue) throws InterruptedException {
        // lowerArm();
        motorHarvest.setPower(-1);
        Thread.sleep(2000);
        moveForward(0.5, 84);
        if (blue)
            pivot(0.3, 40);
        else
            pivot(0.3, -40);
        moveForward(0.5, 6);
        if (blue)
            pivot(0.5, -180);
        else
            pivot(0.5, 181);
        moveForward(0.5, -8);
        scoreClimbers(true);
    }

    public void v2AutoSquare(boolean blue) throws InterruptedException {
        lowerArm();
        motorHarvest.setPower(-1);
        Thread.sleep(1500);
        moveForward(0.6, 13);
        if(blue)
            pivot(0.4, 46);
        else
            pivot(0.4, -46);
        moveForward(0.8, 97);
        if(blue)
            pivot(0.4, 209);
        else
            pivot(0.4, -209);
        moveForward(0.6, -3);
        extendSlides();
        if(blue)
            pivot(0.4, 8);
        else
            pivot(0.4, -8);
        releaseClimbers(blue);
        if(blue)
            pivot(0.4, -8);
        else
            pivot(0.4, 8);
        retractSlides();
        if(blue)
            pivot(0.4, -63);
        else
            pivot(0.4, 63);
        moveForward(0.6, 21);
        if(blue)
            pivot(0.4, 18);
        else
            pivot(0.4, -18);
        moveForward(0.6, 18);
        if(blue)
            pivot(0.4, 110);
        else
            pivot(0.4, -110);
        moveForward(0.6, -16);
        motorHarvest.setPower(0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                              Autonomous support methods                                    //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public void raiseArm() throws InterruptedException {
        motorArm.setTargetPosition(-1965);
    }

    public void lowerArm() throws InterruptedException {
        motorArm.setTargetPosition(0);
    }

    public void scoreClimbers(boolean left) throws InterruptedException {
        extendSlides();
        releaseClimbers(left);
        retractSlides();
    }

    public void moveForwardOnHeading(int distance) throws InterruptedException {
        telemetry.addData("in method", "moveForwardOnHeading");
        telemetry.update();
        int ticks = (int) (TICKS_PER_INCH * distance);
        int totalTicksLeft = motorLeftAft.getCurrentPosition() + ticks;
        int totalTicksRight = motorRightAft.getCurrentPosition() + ticks;
        int initialHeading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = 1;
        double bottomCeiling = -1;
        double topFloor = .12;
        double bottomFloor = -.12;

        if (distance > 0)
            while (motorLeftAft.getCurrentPosition() < totalTicksLeft && motorRightAft.getCurrentPosition() < totalTicksRight) {
                power = (totalTicksLeft - motorLeftAft.getCurrentPosition()) * proportionalConst;

                if (power > topCeiling)
                    power = topCeiling;
                else if (power < bottomCeiling)
                    power = bottomCeiling;
                else if (power < topFloor && power > 0)
                    power = topFloor;
                else if (power > bottomFloor && power < 0)
                    power = bottomFloor;

                setAllDrivePower(power);

                if (getCorrectedHeading() < initialHeading || getCorrectedHeading() > initialHeading)
                    heading(initialHeading);

                telemetry.addData("power", power);
                telemetry.addData("heading", getCorrectedHeading());
                telemetry.addData("initialHeading", initialHeading);
                telemetry.update();


            }
        else
            while (motorLeftAft.getCurrentPosition() > totalTicksLeft && motorRightAft.getCurrentPosition() > totalTicksRight) {
                power = (totalTicksLeft - motorLeftAft.getCurrentPosition()) * proportionalConst;

                if (power > topCeiling)
                    power = topCeiling;
                else if (power < bottomCeiling)
                    power = bottomCeiling;
                else if (power < topFloor && power > 0)
                    power = topFloor;
                else if (power > bottomFloor && power < 0)
                    power = bottomFloor;

                setAllDrivePower(power);

                if (getCorrectedHeading() < initialHeading || getCorrectedHeading() > initialHeading)
                    heading(initialHeading);

                telemetry.addData("power", power);
                telemetry.addData("heading", getCorrectedHeading());
                telemetry.addData("initialHeading", initialHeading);
                telemetry.update();
            }
        setAllDrivePower(0);
        heading(initialHeading);
    }

    public void moveForward(double power, int distance) throws InterruptedException {
        int ticks = (int) (TICKS_PER_INCH * distance);

        motorLeftFore.setTargetPosition(motorLeftFore.getCurrentPosition() + ticks);
        motorLeftAft.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);
        motorRightFore.setTargetPosition(motorRightFore.getCurrentPosition() + ticks);
        motorRightAft.setTargetPosition(motorRightAft.getCurrentPosition() + ticks);

        setAllDrivePower(power);

        motorLeftFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeftAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRightFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRightAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorLeftAft.isBusy() || motorRightAft.isBusy())
            idle();

        setAllDrivePower(0);

    }

    public void pivot(double power, int deg) throws InterruptedException {
        int ticks = (int) (TICKS_PER_DEGREE_DRIVE * deg);

        motorLeftFore.setTargetPosition(motorLeftFore.getCurrentPosition() + ticks);
        motorLeftAft.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);
        motorRightFore.setTargetPosition(motorRightFore.getCurrentPosition() - ticks);
        motorRightAft.setTargetPosition(motorRightAft.getCurrentPosition() - ticks);

        setLeftDrivePower(power);
        setRightDrivePower(power);

        motorLeftFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeftAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRightFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRightAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorLeftAft.isBusy() || motorRightAft.isBusy())
            idle();

        setAllDrivePower(0);

    }

    public void turnWithLeftSide(double power, int deg) throws InterruptedException {
        int ticks = (int) (TICKS_PER_DEGREE_DRIVE * deg * 2);

        motorLeftFore.setTargetPosition(motorLeftFore.getCurrentPosition() + ticks);
        motorLeftAft.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);

        setLeftDrivePower(power);

        motorLeftFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeftAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorLeftAft.isBusy())
            idle();

        setAllDrivePower(0);

    }

    public void turnWithRightSide(double power, int deg) throws InterruptedException {
        int ticks = (int) (TICKS_PER_DEGREE_DRIVE * deg * 2);

        motorRightFore.setTargetPosition(motorRightFore.getCurrentPosition() + ticks);
        motorRightAft.setTargetPosition(motorRightAft.getCurrentPosition() + ticks);

        setRightDrivePower(power);

        motorRightFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRightAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorRightAft.isBusy())
            idle();

        setAllDrivePower(0);

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

            pivot(power);

            telemetry.addData("power", power);
            telemetry.addData("heading", getCorrectedHeading());
            telemetry.addData("target", target);
            telemetry.update();
        }

        setAllDrivePower(0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                              Low level support methods                                     //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * drift);
        int targetHeading = 0; //ARBITRARY VALUE //gyro.getHeading() - offset - totalDrift;
        while (targetHeading > 359)               //
            targetHeading = targetHeading - 360; // Allows value to "wrap around"
        while (targetHeading < 0)                 // since values can only be 0-359
            targetHeading = targetHeading + 360; //
        return targetHeading;
    }

    public void setArmHeight(double power, int deg) throws InterruptedException {
        int ticks = (int) (TICKS_PER_DEGREE_ARM * deg);

        motorArm.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);

        motorArm.setPower(power);

        motorArm.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorArm.isBusy())
            idle();

        motorArm.setPower(0);
    }

    public void extendSlides() throws InterruptedException {
        int distance = -4450;
        double power = 1;

        motorSlide.setTargetPosition(motorSlide.getCurrentPosition() + distance);

        motorSlide.setPower(power);
        motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorSlide.isBusy())
            idle();

        motorSlide.setPower(0);
    }

    public void retractSlides() throws InterruptedException {
        int distance = 4450;
        double power = 1;

        servoRightRamp.setPosition(SHELF_STOW_RIGHT);
        servoLeftRamp.setPosition(SHELF_STOW_LEFT);

        motorSlide.setTargetPosition(motorSlide.getCurrentPosition() + distance);

        motorSlide.setPower(power);
        motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorSlide.isBusy())
            idle();

        motorSlide.setPower(0);
    }

    public void releaseClimbers(boolean left) throws InterruptedException {
        if (left) {
            servoTilt.setPosition(0.18);
            Thread.sleep(2000);
            servoTilt.setPosition(0.5);
            Thread.sleep(1000);
        } else {
            servoTilt.setPosition(0.82);
            Thread.sleep(2000);
            servoTilt.setPosition(0.5);
            Thread.sleep(1000);
        }
    }

    public void setAllDrivePower(double power) {
        motorLeftFore.setPower(power);
        motorLeftAft.setPower(power);
        motorRightFore.setPower(power);
        motorRightAft.setPower(power);
    }

    public void pivot(double power) {
        motorLeftFore.setPower(power);
        motorLeftAft.setPower(power);
        motorRightFore.setPower(-power);
        motorRightAft.setPower(-power);
    }

    public void setLeftDrivePower(double power) {
        motorLeftFore.setPower(power);
        motorLeftAft.setPower(power);
    }

    public void setRightDrivePower(double power) {
        motorRightFore.setPower(power);
        motorRightAft.setPower(power);
    }
}
