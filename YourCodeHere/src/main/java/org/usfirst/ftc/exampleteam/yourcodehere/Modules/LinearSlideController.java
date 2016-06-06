package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.usfirst.ftc.exampleteam.yourcodehere.Height;

/**
 * Class for controlling linear slides
 * Call manualSlide in a recursive loop, updating the input
 * Call retractSlides to retract slides to the starting position
 * Call extendSlides to extend slides to a specified length
 */
public class LinearSlideController {
    DcMotor motor;
    private boolean runningAutoRetract = false;

    public LinearSlideController(DcMotor slideMotor) {
        motor = slideMotor;
        motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    public void retractSlides(int startPosition) {
        if (!motor.getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
            motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(1);
        motor.setTargetPosition(startPosition);
        runningAutoRetract = true;
    }

    //Stops any auto methods using slides and manually controls power with joysticks
    public void manualSlide(float input) {
        if (ScaleInput.scale(input) != 0) { //Threshold so you don't accidentally start running the slides manually
            if (!motor.getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                runningAutoRetract = false;
            }
            motor.setPower(ScaleInput.scale(input));
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

    public void extendSlide(int position) {
        motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        motor.setTargetPosition(position);
    }

    private void runToPosition(int position, double power) {
        if (!motor.getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
            motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
        motor.setTargetPosition(position);
        motor.setPower(power);
    }

}
