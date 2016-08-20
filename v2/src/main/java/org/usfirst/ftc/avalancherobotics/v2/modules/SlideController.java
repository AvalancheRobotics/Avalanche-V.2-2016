package org.usfirst.ftc.avalancherobotics.v2.modules;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *
 */
public class SlideController extends MotorController {
    MotorController motor;

    public SlideController(DcMotor motor) {
        this.motor = new MotorController();
        this.motor.add(motor);
    }

    public void manualControl(double input) {
        if (ScaleInput.scale(input) != 0) {
            motor.setPower(0, input);
        }
    }
}
