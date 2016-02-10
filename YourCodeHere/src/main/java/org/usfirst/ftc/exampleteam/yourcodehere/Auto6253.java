package org.usfirst.ftc.exampleteam.yourcodehere;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.*;
import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

/**
 * Autonomous modes:
 *
 *      test()
 *      set()
 *      park(boolean blue)
 *      climb(boolean blue)
 *
 * Autonomous support methods:
 *
 *      raiseArm() < --- NOT TESTED
 *      lowerArm() < --- NOT TESTED
 *      scoreClimbers(boolean left) < --- NOT TESTED
 *      moveForwardOnHeading(int distance)
 *      moveForward(double power, int distance)
 *      pivot(double power, int deg)
 *      turnWithLeftSide(double power, int deg)
 *      turnWithRightSide(double power, int deg)
 *      heading(int deg)  < --- DOES NOT ALWAYS PIVOT THE RIGHT WAY
 *
 * Low level support methods:
 *
 *      getCorrectedHeading(int offset, int drift, double startTime)
 *      setArmHeight(double power, int deg) < --- NOT TESTED
 *      extendSlides()
 *      retractSlides()
 *      releaseClimbers(boolean left)
 *      setAllDrivePower(double power)
 *      pivot(double power)
 *      setLeftDrivePower(double power)
 *      setRightDrivePower(double power)
 */

@Autonomous(name="Auto6253")
public class Auto6253 extends SynchronousOpMode {
    public static final double TICKS_PER_INCH = 133.7;
    public static final double TICKS_PER_DEGREE_DRIVE = (51.84*TICKS_PER_INCH)/360;
    public static final double TICKS_PER_DEGREE_ARM = ((24/16)*1120)/360;
    public static int offset;
    public static int drift;
    public static double startTime;

    // Declare drive motors
        DcMotor motorLeftFore;
        DcMotor motorLeftAft;
        DcMotor motorRightFore;
        DcMotor motorRightAft;

    // Declare drawer slide motor and servos
    // motor extends/retracts slides
    // servoSlide(continuous) slides the deposit box laterally
    // servoBlockRelease(s) open flaps on the bottom of the bucket, releasing blocks/climbers
        DcMotor motorSlide;
        Servo servoTilt;

    // Declare tape measure motor and servo
    // motor extends/retracts tape
    // servo(continuous) angles tape
        DcMotor motorTape;
        Servo servoTape;

    // Declare motor that spins the harvester
        DcMotor motorHarvest;

    // Declare motor that raises and lowers the collection arm
        DcMotor motorArm;

    // Declare zipline flipping servos
        Servo servoLeftZip;
        Servo servoRightZip;

    // Declare sensors
        GyroSensor gyro;

    @Override public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */

            // Initialize drive motors
            motorLeftFore = hardwareMap.dcMotor.get("motorLeftFore");
            motorLeftAft = hardwareMap.dcMotor.get("motorLeftAft");
            motorRightFore = hardwareMap.dcMotor.get("motorRightFore");
            motorRightAft= hardwareMap.dcMotor.get("motorRightAft");

            motorRightFore.setDirection(DcMotor.Direction.REVERSE);
            motorRightAft.setDirection(DcMotor.Direction.REVERSE);

            // Initialize drawer slide motor and servos
            motorSlide = hardwareMap.dcMotor.get("motorSlide");
            servoTilt = hardwareMap.servo.get("servoTilt");

            // Initialize tape measure motor and servo
            motorTape = hardwareMap.dcMotor.get("motorTape");
            servoTape = hardwareMap.servo.get("servoTape");

            // Initialize motor that spins the harvester
            motorHarvest = hardwareMap.dcMotor.get("motorHarvest");

            // Initialize motor that raises and lowers the collection arm
            motorArm = hardwareMap.dcMotor.get("motorArm");

            // Initialize zipline flipping servos
            servoLeftZip = hardwareMap.servo.get("servoLeftZip");
            servoRightZip = hardwareMap.servo.get("servoRightZip");

            // Initialize sensors
            gyro = hardwareMap.gyroSensor.get("gyro");


            /////////////////////////////////////
            gyro.calibrate();                  //
                                               //
            while (gyro.isCalibrating())       // Calibrating Gyro
                Thread.sleep(50);              //
                                               //
            Thread.sleep(5000);                //
            drift = gyro.getHeading();         //
            /////////////////////////////////////


            waitForStart();

        ////////////////////////////Op mode started/////////////////////////////////////////////////

            offset = gyro.getHeading();
            startTime = System.nanoTime();

            motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);

            boolean blue = true;
            v2Auto(blue);

        }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                              Autonomous modes                                              //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

        public void park(boolean blue) throws InterruptedException{
            lowerArm();
            motorHarvest.setPower(-1);
            moveForwardOnHeading(60);
            if (blue)
                heading(90);
            else
                heading(-90);
            moveForwardOnHeading(48);
            motorHarvest.setPower(0);
            raiseArm();
        }
        public void climb(boolean blue) throws InterruptedException{
            lowerArm();
            motorHarvest.setPower(-1);
            moveForwardOnHeading(42);
            if (blue)
                heading(-135);
            else
                heading(135);
            moveForwardOnHeading(-22);
            if (blue)
                heading(90);
            else
                heading(-90);
            motorHarvest.setPower(0);
            moveForwardOnHeading(-108);
        }

        public void  v2Auto(boolean blue) throws InterruptedException{
            // Tudo, get TICKS_PER_DEGREE_ARM 16/24 for torque
            //       extendSlides() and retractSlides()
            //       releaseClimbers(left)
            lowerArm();
            motorHarvest.setPower(-1);
            Thread.sleep(2000);
            moveForwardOnHeading(12);
            if (blue)
                heading(45);
            else
                heading(-45);
            moveForwardOnHeading(48);
            if(blue)
                heading(-135);
            else
                heading(135);
            moveForwardOnHeading(-24);
            scoreClimbers(true);
            moveForwardOnHeading(6);
            if(blue)
                heading(-90);
            else
                heading(90);
            moveForwardOnHeading(24);
            if(blue)
                heading(45);
            else
                heading(-45);
            moveForwardOnHeading(24);
            moveForwardOnHeading(-6);
            if (blue)
                heading(90);
            else
                heading(-90);
            motorHarvest.setPower(0);
            raiseArm();
            moveForwardOnHeading(-24);
        }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                              Autonomous support methods                                    //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

        public void raiseArm() throws InterruptedException{
            setArmHeight(.5, 45);
        }

        public void lowerArm() throws InterruptedException{
            setArmHeight(.5, -90);
        }

        public void scoreClimbers(boolean left) throws InterruptedException{
            extendSlides();
            releaseClimbers(left);
            retractSlides();
        }

        public void moveForwardOnHeading(int distance) throws InterruptedException{
            telemetry.addData("in method", "moveForwardOnHeading");
            telemetry.update();
            int ticks = (int)(TICKS_PER_INCH * distance);
            int totalTicksLeft = motorLeftAft.getCurrentPosition() + ticks;
            int totalTicksRight = motorRightAft.getCurrentPosition() + ticks;
            int initialHeading = getCorrectedHeading();

            double power;
            double proportionalConst = 0.004;

            double topCeiling = 1;
            double bottomCeiling = -1;
            double topFloor = .12;
            double bottomFloor = -.12;

            if (distance > 0)
                while (motorLeftAft.getCurrentPosition() < totalTicksLeft && motorRightAft.getCurrentPosition() < totalTicksRight){
                    power = (totalTicksLeft - motorLeftAft.getCurrentPosition()) * proportionalConst;

                    if(power > topCeiling)
                        power = topCeiling;
                    else if(power < bottomCeiling )
                        power = bottomCeiling;
                    else if(power < topFloor && power > 0)
                        power = topFloor;
                    else if(power > bottomFloor && power < 0)
                        power = bottomFloor;

                    setAllDrivePower(power);

                    if(getCorrectedHeading() < initialHeading || getCorrectedHeading() > initialHeading)
                        heading(initialHeading);

                    telemetry.addData("power", power);
                    telemetry.addData("heading", getCorrectedHeading());
                    telemetry.addData("initialHeading", initialHeading);
                    telemetry.update();


            }
            else
                while (motorLeftAft.getCurrentPosition() > totalTicksLeft && motorRightAft.getCurrentPosition() > totalTicksRight){
                    power = (totalTicksLeft - motorLeftAft.getCurrentPosition()) * proportionalConst;

                    if(power > topCeiling)
                        power = topCeiling;
                    else if(power < bottomCeiling )
                        power = bottomCeiling;
                    else if(power < topFloor && power > 0)
                        power = topFloor;
                    else if(power > bottomFloor && power < 0)
                        power = bottomFloor;

                    setAllDrivePower(power);

                    if(getCorrectedHeading() < initialHeading || getCorrectedHeading() > initialHeading)
                        heading(initialHeading);

                    telemetry.addData("power", power);
                    telemetry.addData("heading", getCorrectedHeading());
                    telemetry.addData("initialHeading", initialHeading);
                    telemetry.update();
            }
            setAllDrivePower(0);
            heading(initialHeading);
        }

        public void moveForward(double power, int distance) throws InterruptedException{
            int ticks = (int)(TICKS_PER_INCH * distance);

            motorLeftFore.setTargetPosition(motorLeftFore.getCurrentPosition() + ticks);
            motorLeftAft.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);
            motorRightFore.setTargetPosition(motorRightFore.getCurrentPosition() + ticks);
            motorRightAft.setTargetPosition(motorRightAft.getCurrentPosition() + ticks);

            setAllDrivePower(power);

            motorLeftFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorLeftAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorRightFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorRightAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while (motorLeftAft.isBusy() || motorRightAft.isBusy())
                idle();

            setAllDrivePower(0);

        }

        public void pivot(double power, int deg) throws InterruptedException{
            int ticks = (int)(TICKS_PER_DEGREE_DRIVE * deg);

            motorLeftFore.setTargetPosition(motorLeftFore.getCurrentPosition() + ticks);
            motorLeftAft.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);
            motorRightFore.setTargetPosition(motorRightFore.getCurrentPosition() - ticks);
            motorRightAft.setTargetPosition(motorRightAft.getCurrentPosition() - ticks);

            setLeftDrivePower(power);
            setRightDrivePower(power);

            motorLeftFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorLeftAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorRightFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorRightAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while (motorLeftAft.isBusy() || motorRightAft.isBusy())
                idle();

            setAllDrivePower(0);

        }

        public void turnWithLeftSide(double power, int deg) throws InterruptedException{
            int ticks = (int)(TICKS_PER_DEGREE_DRIVE * deg * 2);

            motorLeftFore.setTargetPosition(motorLeftFore.getCurrentPosition()+ ticks);
            motorLeftAft.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);

            setLeftDrivePower(power);

            motorLeftFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorLeftAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while (motorLeftAft.isBusy())
                idle();

            setAllDrivePower(0);

        }

        public void turnWithRightSide(double power, int deg) throws InterruptedException{
            int ticks = (int)(TICKS_PER_DEGREE_DRIVE * deg * 2);

            motorRightFore.setTargetPosition(motorRightFore.getCurrentPosition() + ticks);
            motorRightAft.setTargetPosition(motorRightAft.getCurrentPosition() + ticks);

            setRightDrivePower(power);

            motorRightFore.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorRightAft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while (motorRightAft.isBusy())
                idle();

            setAllDrivePower(0);

        }

        public void heading(int deg){
            double power;
            double proportionalConst = 0.004;

            double topCeiling = 1;
            double bottomCeiling = -1;
            double topFloor = .12;
            double bottomFloor = -.12;

            int target = getCorrectedHeading() + deg;
            while(target > 359)
                target = target - 360;
            while(target < 0)
                target = target + 360;

            while(getCorrectedHeading() != target){
                power = (target - getCorrectedHeading()) * proportionalConst;

                if(power > topCeiling)
                    power = topCeiling;
                else if(power < bottomCeiling )
                    power = bottomCeiling;
                else if(power < topFloor && power > 0)
                    power = topFloor;
                else if(power > bottomFloor && power < 0)
                    power = bottomFloor;

                pivot(power);

                telemetry.addData("power", power);
                telemetry.addData("heading", getCorrectedHeading());
                telemetry.addData("target", target);
                telemetry.update();
            }

            setAllDrivePower(0);
        }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                              Low level support methods                                     //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

        public int getCorrectedHeading(){
            double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
            int totalDrift = (int)(elapsedSeconds / 5 * drift);
            int targetHeading = gyro.getHeading() - offset - totalDrift;
            while(targetHeading > 359)               //
                targetHeading = targetHeading - 360; // Allows value to "wrap around"
            while(targetHeading < 0)                 // since values can only be 0-359
                targetHeading = targetHeading + 360; //
            return targetHeading;
        }

        public void setArmHeight(double power, int deg) throws InterruptedException{
            int ticks = (int)(TICKS_PER_DEGREE_ARM * deg);

            motorArm.setTargetPosition(motorLeftAft.getCurrentPosition() + ticks);

            motorArm.setPower(power);

            motorArm.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while (motorArm.isBusy())
                idle();

            motorArm.setPower(0);
        }

        public void extendSlides() throws InterruptedException{
            int distance = 120;
            double power = .5;

            motorSlide.setTargetPosition(motorSlide.getCurrentPosition() + distance);

            motorSlide.setPower(power);
            motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while (motorSlide.isBusy())
                idle();

            motorSlide.setPower(0);
        }
        public void retractSlides() throws InterruptedException{
            int distance = -120;
            double power = .5;

            motorSlide.setTargetPosition(motorSlide.getCurrentPosition() + distance);

            motorSlide.setPower(power);
            motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while (motorSlide.isBusy())
                idle();

            motorSlide.setPower(0);
        }
        public void releaseClimbers(boolean left) throws InterruptedException{
            if(left){
                servoTilt.setPosition(.25);
                Thread.sleep(2000);
                servoTilt.setPosition(0);
            } else{
                servoTilt.setPosition(-.25);
                Thread.sleep(2000);
                servoTilt.setPosition(0);
            }
        }
        public void setAllDrivePower(double power){
            motorLeftFore.setPower(power);
            motorLeftAft.setPower(power);
            motorRightFore.setPower(power);
            motorRightAft.setPower(power);
        }
        public void pivot(double power){
            motorLeftFore.setPower(power);
            motorLeftAft.setPower(power);
            motorRightFore.setPower(-power);
            motorRightAft.setPower(-power);
        }
        public void setLeftDrivePower(double power){
            motorLeftFore.setPower(power);
            motorLeftAft.setPower(power);
        }
        public void setRightDrivePower(double power){
            motorRightFore.setPower(power);
            motorRightAft.setPower(power);
        }
}
