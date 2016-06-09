package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.usfirst.ftc.exampleteam.yourcodehere.Modules.ScaleInput;

import java.util.ArrayList;

/**
 * Code for DriveTrain
 * Initialize in main TeleOp class
 * call manualDrive in a recursive loop updating the left and right inputs
 * Use setDriveMode to change controls between tank and arcade (defaults to tank)
 */

public class DriveTrainController {

    private ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    private boolean usingTankDrive;
    private ArrayList<Integer> encoderStartValues = new ArrayList<Integer>();

    //Constructors

    public DriveTrainController(DcMotor leftBack, DcMotor rightBack, DcMotor leftFront, DcMotor rightFront) {
        motors.add(leftBack);
        motors.add(rightBack);
        motors.add(leftFront);
        motors.add(rightFront);

        //Reverse right motors because gearing is flipped
        motors.get(1).setDirection(DcMotor.Direction.REVERSE);
        motors.get(3).setDirection(DcMotor.Direction.REVERSE);

        usingTankDrive = true;
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motors.get(i).setPower(0);
            encoderStartValues.add(motors.get(i).getCurrentPosition());
        }
    }

    public DriveTrainController(DcMotor left, DcMotor right) {
        motors.add(left);
        motors.add(right);

        //Reverse right motor because gearing is flipped
        motors.get(1).setDirection(DcMotor.Direction.REVERSE);

        usingTankDrive = true;
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motors.get(i).setPower(0);
            encoderStartValues.add(motors.get(i).getCurrentPosition());
        }
    }

    public void setControlMode(boolean tankDrive) {
        usingTankDrive = tankDrive;
    }


    //Functions by taking inputs (most likely joystick) and squaring them, providing more precise controls when moving slowly
    //When joysticks are not in use, the wheels lock in place using PID
    public void manualDrive(float leftInput, float rightInput) {
        if (usingTankDrive) { //tank drive
            if (ScaleInput.scale(leftInput) == 0 && ScaleInput.scale(rightInput) == 0) {
                if (motors.get(0).getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                    setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    for (int i = 0; i < motors.size(); i++) {
                        motors.get(i).setTargetPosition(motors.get(i).getCurrentPosition());
                    }
                    setLeftDrivePower(1);
                    setRightDrivePower(1);
                }
            } else {
                if (!motors.get(0).getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                    setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                }
                setLeftDrivePower(ScaleInput.scale(leftInput));
                setRightDrivePower(ScaleInput.scale(rightInput));
            }


            //arcade drive
        } else {
            setLeftDrivePower(ScaleInput.scale(leftInput) + ScaleInput.scale(rightInput));
            setRightDrivePower(ScaleInput.scale(leftInput) - ScaleInput.scale(rightInput));
        }
    }


    public void setDriveMode(DcMotorController.RunMode driveMode) {
        if (motors.get(0).getMode().equals(driveMode))
            return;

        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(driveMode);
        }
    }

    public void setLeftDrivePower(double power) {
        if (motors.size() > 2) {
            motors.get(0).setPower(power);
            motors.get(2).setPower(power);
        } else {
            motors.get(0).setPower(power);
        }
    }

    public void setRightDrivePower(double power) {
        if (motors.size() > 2) {
            motors.get(1).setPower(power);
            motors.get(3).setPower(power);
        } else {
            motors.get(1).setPower(power);
        }
    }

    public void resetEncoders() {
        for (int i = 0; i < motors.size(); i++) {
            encoderStartValues.set(i, motors.get(i).getCurrentPosition());
        }
    }

    public void hardResetEncoders() {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RESET_ENCODERS);
            encoderStartValues.set(i, 0);
        }

    }

    public int getEncoderValue(int index) {
        return motors.get(index).getCurrentPosition() - encoderStartValues.get(index);
    }

    public int getAverageEncoderValue() { //returns average of back wheels
        return (motors.get(0).getCurrentPosition() + motors.get(1).getCurrentPosition() - encoderStartValues.get(0) - encoderStartValues.get(1)) / 2;
    }

    public void setTargetPosition (int index, int target) {
        motors.get(index).setTargetPosition(target + encoderStartValues.get(index));
    }

    public ArrayList<DcMotor> getMotors() {
        return motors;
    }

    public boolean isBusy() {
        boolean busy = false;
        for (int i = 0; i < motors.size(); i++) {
            if (motors.get(i).isBusy()) {
                busy = true;
            }
        }

        return busy;
    }

    public ArrayList<DcMotor> getRightMotors() {
        ArrayList<DcMotor> rightMotors = new ArrayList<DcMotor>();

        rightMotors.add(motors.get(1));

        if (motors.size() > 2) {
            rightMotors.add(motors.get(3));
        }
        return rightMotors;
    }

    public ArrayList<DcMotor> getLeftMotors() {
        ArrayList<DcMotor> leftMotors = new ArrayList<DcMotor>();

        leftMotors.add(motors.get(0));

        if (motors.size() > 2) {
            leftMotors.add(motors.get(2));
        }
        return leftMotors;
    }

    public boolean reachedTarget() {
        boolean reached = true;

        for (int i = 0; i < motors.size(); i++) {
            if (!(motors.get(i).getCurrentPosition() < motors.get(i).getTargetPosition() + 18 && motors.get(i).getCurrentPosition() > motors.get(i).getTargetPosition() - 18)) {
                reached = false;
            }
        }
        return reached;
    }

}
