package org.usfirst.ftc.avalancherobotics.v2.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.ArrayList;

/**
 * Class for use in modules to control motors. Do all motor controls though this class.
 */

public class MotorController {
    private ArrayList<DcMotor> motors;
    private ArrayList<Integer> encoderStartValues;
    private boolean autoOverrideEnabled;

    public MotorController() {
        motors = new ArrayList<>();
        encoderStartValues = new ArrayList<Integer>();
        autoOverrideEnabled = true;
    }

    //Adds motor to controller and saves it's current value as a start value.
    //Initalizes motor to RUN_USING_ENCODERS, sets the power to 0, and the target position to it's current position.
    public void add(DcMotor motor) {
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motor.setPower(0);
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

    //Returns run mode of motor at given position
    public DcMotorController.RunMode getRunMode(int index) {
        return motors.get(index).getMode();
    }

    //Sets the run mode for all motors
    public void setRunMode(DcMotorController.RunMode runMode) {
        for (int i = 0; i < motors.size(); i++) {
            if (!motors.get(i).getMode().equals(runMode)) {
                motors.get(i).setMode(runMode);
            }
        }
    }

    //Sets run mode of motor at specified index
    public void setRunMode(DcMotorController.RunMode runMode, int index) {
        if (!motors.get(index).getMode().equals(runMode)) {
            motors.get(index).setMode(runMode);
        }
    }

    //sets power at given index
    //Also changes to manuel mode if manualOverride is enabled or the motor has already reached it's target
    public void setPower(int index, double power) {
        if (getRunMode(index).equals(DcMotorController.RunMode.RUN_USING_ENCODERS) || reachedTarget(index, 5) || autoOverrideEnabled) {

            if (!getRunMode(index).equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                setRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, index);
            }
            motors.get(index).setPower(power);

        }
    }

    //returns the power of the motor at the index
    public void getPower(int index) {
        motors.get(index).getPower();
    }

    //Resets the encoderStartValue at the given index to current value (faster and more reliable than hard resetting encoders)
    public void resetEncoder(int index) {
        encoderStartValues.set(index, motors.get(index).getCurrentPosition());
    }

    //Resets all encoderStartValues to current value (faster and more reliable than hard resetting encoders)
    public void resetEncoders() {
        for (int i = 0; i < motors.size(); i++) {
            encoderStartValues.set(i, motors.get(i).getCurrentPosition());
        }
    }

    //Resets the actual values of all encoders (Use sparingly)
    public void hardResetEncoders() {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setMode(DcMotorController.RunMode.RESET_ENCODERS);
            encoderStartValues.set(i, 0);
        }

    }

    //Resets the actual values of encoder at given value (Use sparingly)
    public void hardResetEncoder(int index) {
        encoderStartValues.set(index, motors.get(index).getCurrentPosition());
    }

    //Gets the soft encoder value of the motor at the given position
    public int getEncoderValue(int index) {
        return motors.get(index).getCurrentPosition() - encoderStartValues.get(index);
    }

    //If all the motors are busy returns true
    public boolean isBusy() {
        for (int i = 0; i < motors.size(); i++) {
            if (motors.get(i).isBusy()) {
                return true;
            }
        }
        return false;
    }

    //If the motor is within the specified threshold of every motors' target positions. returns true
    public boolean reachedTargets(int threshold) {
        boolean reached = true;

        for (int i = 0; i < motors.size(); i++) {
            if (!(motors.get(i).getCurrentPosition() < motors.get(i).getTargetPosition() + threshold && motors.get(i).getCurrentPosition() > motors.get(i).getTargetPosition() - threshold)) {
                reached = false;
            }
        }
        return reached;
    }

    //If the motor is within the specified threshold of the motor's target position. returns true
    public boolean reachedTarget(int index, int threshold) {
        return motors.get(index).getCurrentPosition() < motors.get(index).getTargetPosition() + threshold && motors.get(index).getCurrentPosition() > motors.get(index).getTargetPosition() - threshold;
    }

    //Sets the soft target position of the motor at the given index
    public void setTargetPosition(int index, int target) {
        motors.get(index).setTargetPosition(target + encoderStartValues.get(index));
    }

    //Sets the soft target position of the motor at the given index
    public int getTargetPosition(int index) {
        return motors.get(index).getCurrentPosition() - encoderStartValues.get(index);
    }

    //Sets the power and target position of the motor at the given index.
    //Automatically sets the run mode to auto mode if not already
    public void runToPosition(int index, double power, int targetPosition) {
        if (!getRunMode(index).equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
            setRunMode(DcMotorController.RunMode.RUN_TO_POSITION, index);
        }
        if (motors.get(index).getPower() != power) {
            motors.get(index).setPower(power);
        }
            setTargetPosition(index, targetPosition);
        }

    //If the run mode is runToPosition return true
    public boolean runningAuto(int index) {
        if (!DcMotorController.RunMode.RUN_TO_POSITION.equals(motors.get(index).getMode())) {
            return false;
        }

        return !reachedTarget(index, 10);

    }

    //Toggles auto override.
    //If auto override is enabled, setting the power for a motor will interrupt the
    //current auto action regardless if it's reached its target
    public void toggleAutoOverride(boolean enableOverride) {
        autoOverrideEnabled = enableOverride;
    }

    //Returns auto override
    public boolean isAutoOverrideEnabled() {
        return autoOverrideEnabled;
    }
}
