package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.usfirst.ftc.exampleteam.yourcodehere.ArmPosition;

import java.util.ArrayList;

/**
 * Code for DriveTrain
 * Initialize in main TeleOp class
 * call manualDrive in a recursive loop updating the left and right inputs
 * Use setDriveMode to change controls between tank and arcade (defaults to tank)
 */

public class DriveTrainController {

    private ArrayList<DcMotor> motors = new ArrayList<>();
    private boolean usingTankDrive;


    //Constructors

    public DriveTrainController(DcMotor leftBack, DcMotor rightBack, DcMotor leftFront, DcMotor rightFront) {
        DcMotor motor;
        motors.add(leftBack);
        motors.add(rightBack);
        motors.add(leftFront);
        motors.add(rightFront);

        //Reverse left motors because gearing is flipped
        motors.get(0).setDirection(DcMotor.Direction.REVERSE);
        motors.get(2).setDirection(DcMotor.Direction.REVERSE);

        usingTankDrive = true;
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    public DriveTrainController(DcMotor left, DcMotor right) {
        motors.add(left);
        motors.add(right);

        //Reverse left motor because gearing is flipped
        motors.get(0).setDirection(DcMotor.Direction.REVERSE);

        usingTankDrive = true;
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
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
            motors.get(i).setMode(DcMotorController.RunMode.RESET_ENCODERS);
            motors.get(i).setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    public int getEncoderValue(int index) {
        return motors.get(index).getCurrentPosition();
    }

    public int getAverageEncoderValue() { //returns average of back wheels
        int total = 0;
        for (int i = 0; i < 2; i++) {
            total = total + motors.get(i).getCurrentPosition();
        }
        return total / 2;
    }

    public int getAverageEncoderValueLeft() {
        return (motors.get(0).getCurrentPosition());
    }

    public int getAverageEncoderValueRight() {
        return (motors.get(1).getCurrentPosition());
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

}
