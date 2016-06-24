package org.usfirst.ftc.avalancherobotics.v2;

import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

@TeleOp(name = "ContServo")
public class ContServoTest extends SynchronousOpMode {
    //Declare Servos
    Servo servoShuttle;

    @Override
    public void main() throws InterruptedException {
        servoShuttle = hardwareMap.servo.get("servoShuttle");
        servoShuttle.setPosition(.5);
        waitForStart();
        while (opModeIsActive()) {
            if (updateGamepads()) {
                if (gamepad1.a)
                    servoShuttle.setPosition(0);
                if (gamepad1.b)
                    servoShuttle.setPosition(.5);
                if (gamepad1.x)
                    servoShuttle.setPosition(1);
                if (gamepad1.right_stick_x > .2 || gamepad1.right_stick_x < -.2)
                    servoShuttle.setPosition(gamepad1.right_stick_x/2 + .5);
            }
        }
    }
}
