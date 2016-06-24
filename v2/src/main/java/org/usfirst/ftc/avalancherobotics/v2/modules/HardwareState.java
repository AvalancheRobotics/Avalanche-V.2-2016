package org.usfirst.ftc.avalancherobotics.v2.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by austinzhang on 6/6/16.
 */

/** DO NOT USE DOESN'T WORK TILL RESOLVE WHAT TO DO WHEN NO MOTORS/SERVOS */

public class HardwareState {
    DcMotor motorStates[];
    Servo servoStates[];

    public HardwareState(DcMotor motorStates[], Servo servoStates[]) {
        this.motorStates = motorStates;
        this.servoStates = servoStates;
    }

    public double[] getMotorPower() {
        double motorPower[] = new double[motorStates.length];
        for (int i = 0; i < motorStates.length; i++) {
            motorPower[i] = motorStates[i].getPower();
        }
        return motorPower;
    }

    public double[] getServoPosition() {
        double servoPosition[] = new double[servoStates.length];
        for (int i = 0; i < servoStates.length; i++) {
            servoPosition[i] = servoStates[i].getPosition();
        }
        return servoPosition;
    }

    public DcMotorController.RunMode[] getMotorModes() {
        DcMotorController.RunMode motorModes[] = new DcMotorController.RunMode[motorStates.length];
        for (int i = 0; i < motorStates.length; i++) {
            motorModes[i] = motorStates[i].getMode();
        }
        return motorModes;
    }

    public int[] getMotorTargetPositions() {
        int motorTargetPositions[] = new int[motorStates.length];
        for (int i = 0; i < motorStates.length; i++) {
            motorTargetPositions[i] = motorStates[i].getCurrentPosition();
        }
        return motorTargetPositions;
    }

}
