package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by austinzhang on 2/5/16.
 */
public class RunForTime {
    private long endTime;
    private DcMotor motor;
    private Servo servo;
    private boolean usingMotor;
    private boolean running;

    public RunForTime(DcMotor mot) {
        motor = mot;
        usingMotor = true;
        running = false;
    }

    public RunForTime(Servo serv) {
        servo = serv;
        usingMotor = false;
        running = false;
    }

    public void startRun(long targetTime, double power) {
        endTime = targetTime;
        if (usingMotor) {
            motor.setPower(power);
        }
        else {
            servo.setPosition(power);
        }
        running = true;
    }

    public void update(long currentTime) {
        if (running) {
            if (currentTime > endTime) {
                if (usingMotor) {
                    motor.setPower(0);
                } else {
                    servo.setPosition(.5);
                }
                running = false;
            }
        }
    }
}
