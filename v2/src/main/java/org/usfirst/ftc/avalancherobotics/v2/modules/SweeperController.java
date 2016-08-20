package org.usfirst.ftc.avalancherobotics.v2.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by austinzhang on 6/24/16.
 */
public class SweeperController {
    MotorController motor;

    public SweeperController(DcMotor motor) {
        this.motor = new MotorController();
        this.motor.add(motor);
        this.motor.setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void reverseSweeper() {
        motor.setPower(0, - motor.getPower(0));
    }

    public void startSweeper(double power) {
        motor.setPower(0, power);
    }

    public void stopSweeper() {
        motor.setPower(0, 0);
    }

    public void toggleSweeper(double power) {
        if (Math.abs(motor.getPower(0)) != 0) {
            motor.setPower(0, 0);
        }
        else {
            motor.setPower(0, power);
        }
    }
}
