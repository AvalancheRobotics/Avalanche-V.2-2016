package org.usfirst.ftc.avalancherobotics.v2.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Class for controlling linear slides
 * Call manualSlide in a recursive loop, updating the input
 * Call retractSlides to retract slides to the starting position
 * Call extendSlides to extend slides to a specified length
 */
public class LinearSlideController {
    DcMotor motor;
    private boolean runningAutoRetract;

    public LinearSlideController(DcMotor slideMotor) {
        motor = slideMotor;
        motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(0);
        runningAutoRetract = false;
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

    private void runToPosition(int position, double power) {
        if (!motor.getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
            motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
        motor.setTargetPosition(position);
        motor.setPower(power);
        runningAutoRetract = true;
    }

}
