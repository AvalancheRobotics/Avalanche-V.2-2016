package org.usfirst.ftc.avalancherobotics.v2.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by austinzhang on 6/24/16.
 */
public class TapeController {
    private MotorController motor;
    private Servo servo;

    public TapeController (DcMotor tapeMotor, Servo ratchetServo) {
        motor = new MotorController();
        motor.add(tapeMotor);
        motor.toggleAutoOverride(false);
        servo = ratchetServo;
    }

    public void tapeIn(boolean pressed, int power) {
        if (pressed) {
            motor.setPower(0, power);
        }
    }

    //Has to disengage ratchet
    public void tapeOut(boolean pressed, int backRollValue, int backPower, int power) {
        if (pressed) {
            motor.runToPosition(0, backPower, motor.getEncoderValue(0) + backRollValue);}
            if (!motor.runningAuto(0, 5)) {
                motor.setPower(0, power);
        }
    }

    public void manuelTape(double value) {

    }



}
