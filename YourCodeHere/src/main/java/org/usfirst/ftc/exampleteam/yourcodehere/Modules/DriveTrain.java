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
 * */

public class DriveTrain {

    ArrayList<DcMotor> motors = new ArrayList<>();
    boolean usingTankDrive;


    //Constructors

    public DriveTrain(DcMotor leftBack, DcMotor rightBack, DcMotor leftFront, DcMotor rightFront) {
        DcMotor motor;
        motors.add(leftBack);
        motors.add(rightBack);
        motors.add(leftFront);
        motors.add(rightFront);
        usingTankDrive = true;
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    public DriveTrain(DcMotor left, DcMotor right) {
        motors.add(left);
        motors.add(right);
        usingTankDrive = true;
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    public void setDriveMode(boolean tankDrive) {
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
                if (motors.get(0).getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
                    setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                }
            }


            //arcade drive
        } else {
                setLeftDrivePower(ScaleInput.scale(leftInput) + ScaleInput.scale(rightInput));
                setRightDrivePower(ScaleInput.scale(leftInput) - ScaleInput.scale(rightInput));
        }
    }



    public void setDriveMode(DcMotorController.RunMode driveMode) {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(driveMode);
        }
    }

    public void setLeftDrivePower(double power) {
        motors.get(0).setPower(power);
        motors.get(2).setPower(power);
    }

    public void setRightDrivePower(double power) {
        motors.get(1).setPower(power);
        motors.get(3).setPower(power);
    }

}
