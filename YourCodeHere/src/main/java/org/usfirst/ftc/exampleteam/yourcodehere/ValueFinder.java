/**
 * HOW TO USE THIS METHOD
 *
 * Only uses gamepad1
 *
 * x - resets all motor encoders to zero (Do this if position somehow gets thrown off
 * encoders are automatically reset to zero at initialization
 *
 * left stick y value - used to change servo positions and for running motors forwards and back
 *
 * Current value as well as the thing you're controlling
 * should auto print on the driver phone in the telemetry section
 *
 * keep track of values in the value section (right after class declaration in beginning of code)
 */





package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

@TeleOp(name = "Value Finder")
public class ValueFinder extends SynchronousOpMode {

    /** Keep Track Of Assigned Values Here**/

    private static final int ARBITRARYVALUE = 0;

    //Servo Values
    private static final double RIGHT_ZIP_UP = ARBITRARYVALUE;
    private static final double RIGHT_ZIP_DOWN = ARBITRARYVALUE;
    private static final double LEFT_ZIP_UP = ARBITRARYVALUE;
    private static final double LEFT_ZIP_DOWN = ARBITRARYVALUE;
    private static final double LOCK_ENGAGED = ARBITRARYVALUE;
    private static final double LOCK_DISENGAGED = ARBITRARYVALUE;
    private static final double SHELF_STOW_LEFT = ARBITRARYVALUE;
    private static final double SHELF_STOW_RIGHT = ARBITRARYVALUE;
    private static final double SHELF_DISPENSE_LEFT = ARBITRARYVALUE;
    private static final double SHELF_DISPENSE_RIGHT = ARBITRARYVALUE;
    private static final double DISPENSER_NEUTRAL = ARBITRARYVALUE;
    private static final double DISPENSER_LEFT = ARBITRARYVALUE;
    private static final double DISPENSER_RIGHT = ARBITRARYVALUE;

    //Motor Positions
    private double slideBotPosition = ARBITRARYVALUE;
    private double slideMidPosition = ARBITRARYVALUE;
    private double slideTopPosition = ARBITRARYVALUE;
    private int armInitPosition = ARBITRARYVALUE;
    private int armHarvestPosition = ARBITRARYVALUE;
    private int armDispensePosition = ARBITRARYVALUE;
    private int armMountainPosition = ARBITRARYVALUE;
    //farthest slide can extend without damage (soft stop)
    private int maxSlideLength = ARBITRARYVALUE;
    //length which tape extends to to hang
    private int hangLength = ARBITRARYVALUE;



    //Declare Servos
    Servo servoLock;
    Servo servoLeftZip;
    Servo servoRightZip;
    Servo servoLeftRamp;
    Servo servoRightRamp;
    Servo servoTilt;
    DcMotor motorSlide;
    DcMotor motorArm;
    DcMotor motorTape;

    int toggle = 0;
    Servo currentServo = servoLock;

    String servoName = "servoLock";

    DcMotor currentMotor = motorArm;

    String motorName = "motorArm";


    private void hardwareMapping() throws InterruptedException {
        // SERVOS
        servoLock = hardwareMap.servo.get("Lock");
        servoLeftZip = hardwareMap.servo.get("LeftZip");
        servoRightZip = hardwareMap.servo.get("RightZip");
        servoLeftRamp = hardwareMap.servo.get("LeftRamp");
        servoRightRamp = hardwareMap.servo.get("RightRamp");
        servoTilt = hardwareMap.servo.get("Tilt");

        servoLock.setPosition(servoLock.getPosition());
        servoLeftZip.setPosition(servoLeftZip.getPosition());
        servoRightZip.setPosition(servoRightZip.getPosition());
        servoLeftRamp.setPosition(servoLeftRamp.getPosition());
        servoRightRamp.setPosition(servoRightRamp.getPosition());
        servoTilt.setPosition(servoTilt.getPosition());

        // MOTORS
        motorArm = hardwareMap.dcMotor.get("Arm");
        motorSlide = hardwareMap.dcMotor.get("Slide");

        motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorSlide.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTape.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorArm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorSlide.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorTape.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    @Override
    public void main() throws InterruptedException {
        hardwareMapping();
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {


            if (updateGamepads()) {

                if (gamepad1.right_bumper) {
                    toggle++;
                }

                if (gamepad1.left_bumper) {
                    toggle--;
                }

                if (toggle > 6) {
                    toggle = 0;
                }

                if (toggle < 0) {
                    toggle = 6;
                }

                if (gamepad1.x) {
                    motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorSlide.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorTape.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                }

            }
            if (toggle == 0) {
                servoLock.setPosition(servoLock.getPosition() + (scaleInput(gamepad1.left_stick_y) / 30));
                currentServo = servoLock;
                servoName = "servoLock";
            }

            if (toggle == 1) {
                servoRightRamp.setPosition(servoRightRamp.getPosition() + (scaleInput(gamepad1.left_stick_y) / 30));
                servoLeftRamp.setPosition(servoLeftRamp.getPosition() - (scaleInput(gamepad1.left_stick_y) / 30));
                currentServo = servoRightRamp;
                servoName = "servoRightRamp";
            }

            if (toggle == 2) {
                servoRightZip.setPosition(servoRightZip.getPosition() + (scaleInput(gamepad1.left_stick_y) / 30));
                currentServo = servoRightZip;
                servoName = "servoRightZip";
            }

            if (toggle == 3) {
                servoLeftZip.setPosition(servoLeftZip.getPosition() + (scaleInput(gamepad1.left_stick_y) / 30));
                currentServo = servoLeftZip;
                servoName = "servoLeftZip";
            }

            if (toggle == 4) {
                motorArm.setPower(scaleInput(gamepad1.left_stick_y));
                currentServo = null;
                servoName = null;
                motorName = "motorArm";
                currentMotor = motorArm;
            }
            else {
                motorArm.setPower(0);
            }

            if (toggle == 5) {
                motorSlide.setPower(scaleInput(gamepad1.left_stick_y));
                currentServo = null;
                servoName = null;
                motorName = "motorSlide";
                currentMotor = motorSlide;
            }
            else {
                motorSlide.setPower(0);
            }

            if (toggle == 6) {
                motorTape.setPower(scaleInput(gamepad1.left_stick_y));
                currentServo = null;
                servoName = null;
                motorName = "motorTape";
                currentMotor = motorTape;
            }

            if (toggle == 7) {
                servoTilt.setPosition(servoTilt.getPosition() + (scaleInput(gamepad1.left_stick_y) / 30));
                currentServo = servoTilt;
                servoName = "servoTilt";
            }

        }

        if (currentServo != null) {
            telemetry.addData(servoName, currentServo.getPosition());
            if (servoName.equals("servoRightRamp"))
                telemetry.addData("servoLeftRamp", servoLeftRamp.getPosition());
        }
        else {
            telemetry.addData(motorName, currentMotor.getCurrentPosition());
        }

        telemetry.update();
        idle();
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
}
