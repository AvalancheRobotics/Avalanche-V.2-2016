package org.usfirst.ftc.avalancherobotics.v2.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.ArrayList;

/**
 * Created by austinzhang on 6/22/16.
 */
public class MotorController {
    private ArrayList<DcMotor> motors;
    private ArrayList<Integer> encoderStartValues;

    public MotorController() {
        motors = new ArrayList<>();
        encoderStartValues = new ArrayList<Integer>();
    }

    public void add(DcMotor motor) {
        motors.add(motor);
        encoderStartValues.add(motor.getCurrentPosition());
    }

    public int size() {
        return motors.size();
    }

    //Makes the direction of the motor opposite to it's current direction.
    public void reverseMotors(int motorIndex) {
        if (motors.get(motorIndex).getDirection().equals(DcMotor.Direction.FORWARD)) {
            motors.get(motorIndex).setDirection(DcMotor.Direction.REVERSE);
        } else {
            motors.get(motorIndex).setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public DcMotorController.RunMode getRunMode(int index) {
        return motors.get(index).getMode();
    }

    //Sets the runmode for all motors
    public void setRunMode(DcMotorController.RunMode runMode) {
        for (int i = 0; i < motors.size(); i++) {
            if (!motors.get(i).getMode().equals(runMode)) {
                motors.get(i).setMode(runMode);
            }
        }
    }

    //Sets runmode of motor at specified index
    public void setRunMode(DcMotorController.RunMode runMode, int index) {
        if (!motors.get(index).getMode().equals(runMode)) {
            motors.get(index).setMode(runMode);
        }
    }

    public void setPower(int index, double power) {
        motors.get(index).setPower(power);
    }

    public void setPowers(double power) {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(power);
        }
    }

    public void getPower(int index) {
        motors.get(index).getPower();
    }

    public void resetEncoder(int index) {
        encoderStartValues.set(index, motors.get(index).getCurrentPosition());
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

    public void hardResetEncoder(int index) {
        encoderStartValues.set(index, motors.get(index).getCurrentPosition());
    }

    public int getEncoderValue(int index) {
        return motors.get(index).getCurrentPosition() - encoderStartValues.get(index);
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

    public boolean reachedTargets() {
        boolean reached = true;

        for (int i = 0; i < motors.size(); i++) {
            if (!(motors.get(i).getCurrentPosition() < motors.get(i).getTargetPosition() + 5 && motors.get(i).getCurrentPosition() > motors.get(i).getTargetPosition() - 5)) {
                reached = false;
            }
        }
        return reached;
    }

    public boolean reachedTarget(int index) {
        return (motors.get(index).getCurrentPosition() < motors.get(index).getTargetPosition() + 5 && motors.get(index).getCurrentPosition() > motors.get(index).getTargetPosition() - 5);
    }

    public void setTargetPosition(int index, int target) {
        motors.get(index).setTargetPosition(target + encoderStartValues.get(index));
    }

    public int getTargetPosition(int index) {
        return motors.get(index).getCurrentPosition();
    }

}
