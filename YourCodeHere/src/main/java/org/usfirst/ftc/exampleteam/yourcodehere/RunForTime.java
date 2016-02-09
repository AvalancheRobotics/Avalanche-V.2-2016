package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class is for running a motor or servo at a specified speed for a specified amount of time.
 *
 * update needs to be run at the end of each main loop as it updates the internal class time
 *
 * startRun takes in a target time, NOT A RUN TIME
 * you would specify a targetTime as for example, System.currentTimeMillis + 300 if you wanted it
 * to run for 300 milliseconds, not just 300
 *
 * This class allows you to take manual control of the motor, and in doing so, will abort auto stop
 * as well as the run power it's set to
 * - (you'll be able to change the power and it won't automatically stop when the time is reached)
 */

public class RunForTime {
    private long endTime;
    private DcMotor motor;
    private Servo servo;
    private boolean usingMotor;
    private boolean running;
    private double power;

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
        this.power = power;
        if (usingMotor) {
            motor.setPower(this.power);
        } else {
            servo.setPosition(this.power);
        }
        running = true;
    }

    public void update(long currentTime) {
        if (running) {
            if (currentTime > endTime) {
                if (usingMotor) {
                    if (motor.getPower() == power)
                        motor.setPower(0);
                } else {
                    if (servo.getPosition() == power)
                        servo.setPosition(.5);
                }
                running = false;
            }
        }
    }
}
