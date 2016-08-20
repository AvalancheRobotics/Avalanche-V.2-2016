package org.usfirst.ftc.avalancherobotics.v2.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.ArrayList;

/**
 * Code for DriveTrain
 * Initialize in main TeleOp class
 * call manualDrive in a recursive loop updating the left and right inputs
 * Use setDriveMode to change controls between tank and arcade (defaults to tank)
 */

public class DriveTrainController extends MotorController {

    private boolean usingTankDrive;
    private MotorController motors;

    private static final double WHEEL_DIAMETER = 10.16; //IN CM
    private static final int TICKS_PER_ROTATION = 1120;

    //Constructors

    public DriveTrainController(DcMotor leftBack, DcMotor rightBack, DcMotor leftFront, DcMotor rightFront) {
        motors = new MotorController();
        motors.add(leftBack);
        motors.add(rightBack);
        motors.add(leftFront);
        motors.add(rightFront);

        //Reverse right motors because gearing is flipped
        motors.reverseMotors(1);
        motors.reverseMotors(3);

        usingTankDrive = true;
    }

    public DriveTrainController(DcMotor left, DcMotor right) {
        motors = new MotorController();
        motors.add(left);
        motors.add(right);

        //Reverse right motor because gearing is flipped
        motors.reverseMotors(1);

        usingTankDrive = true;
    }

    public void setControlMode(boolean tankDrive) {
        usingTankDrive = tankDrive;
    }


    //Functions by taking inputs (most likely joystick) and squaring them, providing more precise controls when moving slowly
    //When joysticks are not in use, the wheels lock in place using PID
    public void manualDrive(float leftInput, float rightInput) {
        if (usingTankDrive) { //tank drive
            if (ScaleInput.scale(leftInput) == 0 && ScaleInput.scale(rightInput) == 0) {
                boolean runningAuto = false;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.runningAuto(i, 5)) {
                        runningAuto = true;
                    }
                }
                if (runningAuto) {
                    for (int i = 0; i < motors.size(); i++) {
                        motors.runToPosition(i, 1, motors.getEncoderValue(i));
                    }
                }
            } else {
                setLeftDrivePower(ScaleInput.scale(leftInput));
                setRightDrivePower(ScaleInput.scale(rightInput));
            }


            //arcade drive
        } else {
            setLeftDrivePower(ScaleInput.scale(leftInput) + ScaleInput.scale(rightInput));
            setRightDrivePower(ScaleInput.scale(leftInput) - ScaleInput.scale(rightInput));
        }
    }

    //Functions by taking inputs (most likely joystick) and squaring them, providing more precise controls when moving slowly
    //When joysticks are not in use, the wheels lock in place using PID
    private void manualDrive2(float leftInput, float rightInput) {
        if (usingTankDrive) { //tank drive
            if (ScaleInput.scale(leftInput) == 0 && ScaleInput.scale(rightInput) == 0) {
                for (int i = 0; i < motors.size(); i++) {
                    motors.runToPosition(i, 1, motors.getEncoderValue(i));
                }
            } else {
                setLeftDrivePower(ScaleInput.scale(leftInput));
                setRightDrivePower(ScaleInput.scale(rightInput));
            }


            //arcade drive
        } else {
            setLeftDrivePower(ScaleInput.scale(leftInput) + ScaleInput.scale(rightInput));
            setRightDrivePower(ScaleInput.scale(leftInput) - ScaleInput.scale(rightInput));
        }
    }


    public void setLeftDrivePower(double power) {
        if (motors.size() > 2) {
            motors.setPower(0, power);
            motors.setPower(2, power);
        } else {
            motors.setPower(0, power);
        }
    }

    public void setRightDrivePower(double power) {
        if (motors.size() > 2) {
            motors.setPower(1, power);
            motors.setPower(3, power);
        } else {
            motors.setPower(1, power);
        }
    }

    public int distanceTraveledBeforeReset() {
        /** NEED ODOMETER WHEEL CURRENTLY ONLY USES WHEEL MEASUREMENT */

        int totalOdometerReading = 0;
        for (int i = 0; i < motors.size(); i++) {
            totalOdometerReading = totalOdometerReading + motors.getEncoderValue(i);
        }

        totalOdometerReading = totalOdometerReading / motors.size();

        return (int) Math.round(totalOdometerReading / TICKS_PER_ROTATION * (WHEEL_DIAMETER * Math.PI));
    }

}
