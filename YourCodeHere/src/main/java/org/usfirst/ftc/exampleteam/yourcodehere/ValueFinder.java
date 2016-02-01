package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

@TeleOp(name = "Value Finder")
public class ValueFinder extends SynchronousOpMode {
    //Declare Servos
    Servo servoLock;
    Servo servoLeftZip;
    Servo servoRightZip;
    Servo servoShelfLeft;
    Servo servoShelfRight;
    DcMotor motorSlide;
    DcMotor motorArm;
    DcMotor motorTape;

    int toggle = 0;
    Servo currentServo = servoLock;

    String servoName = "servoLock";

    DcMotor currentMotor = motorArm;

    String motorName = "motorArm";

    //constant values (servo positions)

    private static final double ARBITRARYVALUE = 0.0;

    private static final double ZIP_UP_LEFT = ARBITRARYVALUE;
    private static final double ZIP_DOWN_LEFT = ARBITRARYVALUE;
    private static final double ZIP_UP_RIGHT = ARBITRARYVALUE;
    private static final double ZIP_DOWN_RIGHT = ARBITRARYVALUE;
    private static final double LOCK_ENGAGED = ARBITRARYVALUE;
    private static final double LOCK_DISENGAGED = ARBITRARYVALUE;
    private static final double SHELF_STOW_LEFT = ARBITRARYVALUE;
    private static final double SHELF_STOW_RIGHT = ARBITRARYVALUE;
    private static final double SHELF_DISPENSE_LEFT = ARBITRARYVALUE;
    private static final double SHELF_DISPENSE_RIGHT = ARBITRARYVALUE;


    private void hardwareMapping() throws InterruptedException {
        // SERVOS
        servoLock = hardwareMap.servo.get("servoLock");
        servoLeftZip = hardwareMap.servo.get("servoLeftZip");
        servoRightZip = hardwareMap.servo.get("servoRightZip");
        servoShelfLeft = hardwareMap.servo.get("servoShelfLeft");
        servoShelfRight = hardwareMap.servo.get("servoShelfRight");

        servoLock.setPosition(servoLock.getPosition());
        servoLeftZip.setPosition(servoLeftZip.getPosition());
        servoRightZip.setPosition(servoRightZip.getPosition());
        servoShelfLeft.setPosition(servoShelfLeft.getPosition());
        servoShelfRight.setPosition(servoShelfRight.getPosition());

        // MOTORS
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorSlide = hardwareMap.dcMotor.get("motorSlide");

        motorArm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorSlide.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorTape.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorSlide.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTape.setMode(DcMotorController.RunMode.RESET_ENCODERS);
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
                servoShelfRight.setPosition(servoShelfRight.getPosition() + (scaleInput(gamepad1.left_stick_y) / 30));
                servoShelfLeft.setPosition(servoShelfLeft.getPosition() - (scaleInput(gamepad1.left_stick_y) / 30));
                currentServo = servoShelfRight;
                servoName = "servoShelfRight";
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

        }

        if (currentServo != null) {
            telemetry.addData(servoName, currentServo.getPosition());
            if (servoName.equals("servoShelfRight"))
                telemetry.addData("servoShelfLeft", servoShelfLeft.getPosition());
        }
        else {
            telemetry.addData(motorName, currentMotor.getCurrentPosition());
        }

            telemetry.update();
            idle();

            if (toggle > 6) {
                toggle = 0;
            }

            if (toggle < 0) {
                toggle = 6;
            }
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
