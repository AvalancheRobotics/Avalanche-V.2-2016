package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.usfirst.ftc.exampleteam.yourcodehere.Height;

/**
 * Created by Austin on 6/3/2016.
 */
public class LinearSlides {
    DcMotor motor;
    private boolean runningAutoRetract = false;

    public LinearSlides(DcMotor slideMotor) {
        motor = slideMotor;
        motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    public void retractSlides(boolean button, Position startPosition) {
        if (button) {
            if (!motor.getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
                motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }
            motor.setPower(1);
            motor.setTargetPosition(startPosition.getValue());
            runningAutoRetract = true;
        }
    }

    //Stops any auto methods using slides and manually controls power with joysticks
    public void manualSlide(float leftInput, float rightInput) {
        if (ScaleInput.scale(leftInput) != 0) { //Threshold so you don't accidentally start running the slides manually
            if (!motor.getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                runningAutoRetract = false;
            }
            motor.setPower(ScaleInput.scale(leftInput));
        } else {
            if (runningAutoRetract) {
                if (motor.getCurrentPosition() >= motor.getTargetPosition() - 20 && motor.getCurrentPosition() <= motor.getTargetPosition() + 20) {
                    runningAutoRetract = false;
                }
            } else {
                if (!motor.getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
                    motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    motor.setPower(.5);
                    motor.setTargetPosition(motor.getCurrentPosition());
                }
            }
        }
    }

    private void extendSlide(Position position) {
        motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        motor.setTargetPosition(position.getValue());
    }

}
