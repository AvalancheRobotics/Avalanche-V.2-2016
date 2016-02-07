package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;
/**
 * TO DO LIST:
 * add in shelf servos
 * make it so that shelf doesn't let servo shelf go to dispense position when arms are extended
 * make servoShelf automatically go to stow position when arm is extended
 */


/**
 * Version 1.0 of Team Avalanche 6253's TeleOp program for Robot version 2.0.
 * Currently most distance and position values are arbitrary due to not having a complete robot we can test values on.
 * Nearly finished, all methods save loadDispenser are written, needs some fixing, values, and a lot of testing.
 * UNLIKE NORMAL TELEOP, USES FTC'S BUILT IN MOVE_TO_POSITION which causes the motor to move more slowly but is also more accurate and easier to code than custom PID
 */
@TeleOp(name = "TeleOpNoPID")
public class TeleOpNoPID extends SynchronousOpMode {
    //RunForTime Class declaration
    RunForTime timerHarvest;


    // Variables

    //Enums

    //Used to tell where the servoSlide is (whether it's in a scoring position or neutral position
    SlidePosition slidePosition;

    //Running loadDispenser Method
    private boolean runningLoadDispenser = false;

    //shows what step score method is on
    private int scoreToggle = 0;

    //How long it has been since the Tape has been called
    private long tapeStartTime = 0;

    // defaults to blue alliance
    private boolean isBlue = true;

    //tells whether triggers are in resting position or are down/active, starts at rest
    private boolean atRestTriggers = true;

    //tells whether conveyor is running automatically in score method
    private boolean autoConveyorRunning = false;

    // methods with these variables need values
    private final double ARBITRARYDOUBLE = 0;
    private final int ARBITRARYINT = 0;

    //Declare starting positions for tape, arm, and slide motors
    private int startPosTape;
    private int startPosArm;
    private int startPosSlide;
    private int maxTapeLength;

    //Measured in inches
    private double slideBotPosition;
    private double slideMidPosition;
    private double slideTopPosition;

    //Measured in ticks
    private int armInitPosition;
    private int armHarvestPosition;
    private int armDispensePosition;
    private int armMountainPosition;

    //farthest slide can extend without damage
    private int maxSlideLength;

    //length which tape extends to to hang
    private int hangLength;

    //sees if this is the first time pressing the button
    private boolean firstPressedDPad = true;
    private boolean firstPressedLeftTrigger = true;

    private final double RIGHT_ZIP_UP = 1; //ARBITRARY
    private final double RIGHT_ZIP_DOWN = 0; //ARBITRARY
    private final double LEFT_ZIP_UP = 1; //ARBITRARY
    private final double LEFT_ZIP_DOWN = 0; //ARBITRARY
    private final int TICKS_IN_DEGREE_ARM = ARBITRARYINT;
    private final int TICKS_IN_INCH_SLIDE = 1200; //ARBITRARY
    private final double LOCK_ENGAGED = 1; //ARBITRARY
    private final double LOCK_DISENGAGED = 0; //ARBITRARY
    private final double CONSTANT_DRIVE_SPEED = 1; //TESTING PURPOSES
    private static final double SHELF_STOW_LEFT = 0; //ARBITRARY
    private static final double SHELF_STOW_RIGHT = 0; //ARBITRARY
    private static final double SHELF_DISPENSE_LEFT = 0; //ARBITRARY
    private static final double SHELF_DISPENSE_RIGHT = 0; //ARBITRARY
    private static final double DISPENSER_NEUTRAL = 0; //ARBITRARY
    private static final double DISPENSER_LEFT = 0; //ARBITRARY
    private static final double DISPENSER_RIGHT = 0; //ARBITRARY
    private static final double FLAP_UP_LEFT = 0; //ARBITRARY
    private static final double FLAP_UP_RIGHT = 0; //ARBITRARY
    private static final double FLAP_DOWN_LEFT = 0; //ARBITRARY
    private static final double FLAP_DOWN_RIGHT = 0; //ARBITRARY


    // Declare drive motors
    DcMotor motorLeftFore;
    DcMotor motorLeftAft;
    DcMotor motorRightFore;
    DcMotor motorRightAft;

    // Declare drawer slide motor and servos
    // motor extends/retracts slides
    // servoSlide(continuous) slides the deposit box laterally
    // servoConveyor runs the conveyor belt, dispensing blocks over the side
    // servoLock is the servo for locking the tape measure in place once hanging.
    DcMotor motorSlide;
    Servo servoShuttle;
    Servo servoConveyor;
    Servo servoLock;

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

    //Declare color sensors and enable,x led
    ColorSensor colorLeft;
    ColorSensor colorRight;

    //Shelf servos
    Servo servoShelfLeft;
    Servo servoShelfRight;

    //Servo for angling dispenser
    Servo servoDispenserAngle;

    //Servo for dispenser flaps
    Servo servoLeftFlap;
    Servo servoRightFlap;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        // Initialize drive motors
        motorLeftFore = hardwareMap.dcMotor.get("motorLeftFore");
        motorLeftAft = hardwareMap.dcMotor.get("motorLeftAft");
        motorRightFore = hardwareMap.dcMotor.get("motorRightFore");
        motorRightAft = hardwareMap.dcMotor.get("motorRightAft");

        //Left and right motors are on opposite sides and must spin opposite directions to go forward
        motorLeftAft.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFore.setDirection(DcMotor.Direction.REVERSE);

        // Initialize drawer slide motor and servos
        motorSlide = hardwareMap.dcMotor.get("motorSlide");
        servoShuttle = hardwareMap.servo.get("servoShuttle");
        servoConveyor = hardwareMap.servo.get("servoConveyor");

        // Initialize tape measure motor, servo tape, and servo lock.
        motorTape = hardwareMap.dcMotor.get("motorTape");
        servoTape = hardwareMap.servo.get("servoTape");
        servoLock = hardwareMap.servo.get("servoLock");

        // Initialize motor that spins the harvester
        motorHarvest = hardwareMap.dcMotor.get("motorHarvest");

        // Initialize motor that raises and lowers the collection arm
        motorArm = hardwareMap.dcMotor.get("motorArm");

        // Initialize zipline flipping servos
        servoLeftZip = hardwareMap.servo.get("servoLeftZip");
        servoRightZip = hardwareMap.servo.get("servoRightZip");

        // Initialize shelf servos
        servoShelfLeft = hardwareMap.servo.get("servoShelfLeft");
        servoShelfRight = hardwareMap.servo.get("servoShelfRight");

        // Initialize dispenser angle servo
        servoDispenserAngle = hardwareMap.servo.get("servoDispenserAngle");

        // Initialize dispenser flap control servos
        servoLeftFlap = hardwareMap.servo.get("servoLeftFlap");
        servoRightFlap = hardwareMap.servo.get("servoRightFlap");

        // Initialize sensors

        //Initialize color sensor and turn on led
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");
        colorLeft.enableLed(true);
        colorRight.enableLed(true);

        //Initialize gyro
        gyro = hardwareMap.gyroSensor.get("gyro");
        //Initialize touch sensors
        //touchLeftPos = hardwareMap.touchSensor.get("touchLeftPos");
        //touchRightPos = hardwareMap.touchSensor.get("touchRightPos");
        //touchNeutralPos = hardwareMap.touchSensor.get("touchNeutralPos");

        gyro.calibrate();

        // Reset encoders
        this.motorLeftAft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorLeftFore.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorTape.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorHarvest.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorRightAft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorRightFore.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorSlide.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        //Set Runmode for all motors to run using encoders
        motorLeftFore.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRightFore.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeftAft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRightAft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorSlide.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorTape.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorArm.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorHarvest.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        //keep track of the starting positions of arm, slide, and tape motors
        startPosArm = motorArm.getCurrentPosition();
        startPosSlide = motorSlide.getCurrentPosition();
        startPosTape = motorSlide.getCurrentPosition();
        motorArm.setTargetPosition(startPosArm);

        //Score heights for slides
        slideBotPosition = startPosTape + 3000; //ARBITRARY
        slideMidPosition = startPosTape + 6000; //ARBITRARY
        slideTopPosition = startPosTape + 9000; //ARBITRARY
        maxSlideLength = startPosTape + 12000; //ARBITRARY

        //Arm Positions
        armInitPosition = startPosArm;
        armDispensePosition = startPosArm + 3000; //ARBITRARY
        armHarvestPosition = startPosArm + 3000; //ARBITRARY
        armMountainPosition = startPosArm + 30; //ARBITRARY

        //keep track of max length tape should extend
        maxTapeLength = motorTape.getCurrentPosition() + 6000; //ARBITRARY

        hangLength = motorTape.getCurrentPosition() + 5000; //ARBITRARY

        //initializes to neutral because that's where the starting position is
        slidePosition = SlidePosition.NEUTRAL;

        //initializes servos to their starting positions
        servoDispenserAngle.setPosition(DISPENSER_NEUTRAL);
        servoRightFlap.setPosition(FLAP_UP_RIGHT);
        servoLeftFlap.setPosition(FLAP_UP_LEFT);
        servoLeftZip.setPosition(LEFT_ZIP_UP);
        servoRightZip.setPosition(RIGHT_ZIP_UP);
        servoShelfLeft.setPosition(SHELF_STOW_LEFT);
        servoShelfRight.setPosition(SHELF_STOW_RIGHT);
        servoLock.setPosition(LOCK_ENGAGED);

        timerHarvest = new RunForTime(motorHarvest);
    }

    @Override
    public void main() throws InterruptedException {
        hardwareMapping();

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            //Checks to see if something changed on the controller ie. a button is pressed or a trigger is bumped
            if (updateGamepads()) {

                //AUTOMATIC CONTROLS//
                setAllAutoMethods();

                //MANUAL CONTROLS
                manualMethods();
            }

            //Runs autonomous methods
            runAllAutoMethods();

            //read sensors and adjust accordingly
            sensorChanges();

            //Run For Time Methods (Checks and sees if time is up and if so stops motor
            timerHarvest.update(System.currentTimeMillis());

            idle();
        }
    }

    /**
     * in the sensorChanges method we put the methods that depend on
     * sensor input do decide when to start, stop, etc.
     * It is the last mehtod to run in the main loop
     */

    private void sensorChanges() {

        //stops the slide and updates position when reaches desired position
        // shuttleAutoStop();

        /** THIS IS A TEST METHOD FOR THE COLOR SENSOR
         if (colorLeft.blue() + colorLeft.red() + colorLeft.green() > 50)
         motorHarvest.setPower(0);
         else
         motorHarvest.setPower(1);
         */
    }

    private void manualMethods() {
        //Toggle Team (if we need to score on an opponent's ramp)
        if ((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b)) {
            isBlue = false;
        }
        if ((gamepad1.back && gamepad1.x) || (gamepad2.back && gamepad2.x)) {
            isBlue = true;
        }


        //starts and stops harvester
        if (gamepad1.left_trigger > .8 && firstPressedLeftTrigger) {
            toggleHarvester(1);
            firstPressedLeftTrigger = false;
        } else
            firstPressedLeftTrigger = true;

        //toggles harvester spin direction
        if (gamepad1.left_bumper)
            motorHarvest.setPower(-motorHarvest.getPower());

        //Read Joystick Data and Update Speed of Left and Right Motors
        //If joystick buttons are pressed, sets drive power to preset value
        manualDriveControls(true);


        //toggles zip line position
        if (gamepad2.x && !gamepad2.back)
            triggerZipline();


        //stops all motors and cancels all motor operations, basically a panic button that stops all functions
        if ((gamepad1.back && gamepad1.start) || (gamepad2.back && gamepad2.start))
            cancelAll();


        //Stops any auto methods using slides and manually controls power with joysticks
        if (gamepad2.left_stick_y < -.2 || gamepad2.left_stick_y > .2) { //Threshold so you don't accidentally start running the slides manually
            motorSlide.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            if (motorSlide.getCurrentPosition() > startPosSlide + 20 && motorSlide.getCurrentPosition() < maxSlideLength) {
                motorSlide.setPower(scaleInput(gamepad2.left_stick_y));
            } else {
                motorSlide.setPower(0);
            }
        } else if (motorSlide.getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS))
            motorSlide.setPower(0);


        //adjust the shuttle servo manually
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            if (gamepad2.left_bumper) {
                servoConveyor.setPosition(0); //COULD BE 1 ARBITRARY
            }
            if (gamepad2.right_bumper) {
                servoConveyor.setPosition(1); //COULD BE 0 ARBITRARY
            }
            autoConveyorRunning = false;
        } else {
            if (!autoConveyorRunning)
                servoConveyor.setPosition(.5);
        }


        //manually adjust the conveyors
        if (gamepad2.right_trigger > .2 || gamepad2.left_trigger > .2) {
            if (gamepad2.right_trigger > .2) //.2 is a threshold so you don't accidentally press it
                servoShuttle.setPosition(gamepad2.right_trigger / 2 + .5);
            else if (gamepad2.left_trigger > .2)
                servoShuttle.setPosition(-gamepad2.left_trigger / 2 + .5);
        } else {
            servoShuttle.setPosition(.5);
        }

        //adjust angle of tape
        servoTape.setPosition(scaleInput(gamepad2.right_stick_y / 2 + .5));


    }

    /**
     * put all autonomous setters in here, when the code updates the gamepad
     * it should look at this section to see if we're starting any auto methods
     * DO NOT PUT MANUAL METHODS- Separate Wrapper Method
     */
    private void setAllAutoMethods() {

        /**
         * Scoring methods
         * First button press extends slides and shuttle
         * Second button press starts conveyor
         * Third button press stops conveyor, and restores slides and shuttle to default positions
         */
        scoreSet(Height.BOT, gamepad2.a);
        scoreSet(Height.MID, gamepad2.b && !gamepad2.back);
        scoreSet(Height.TOP, gamepad2.y);

    }

    /**
     * put all autonomous functions in here, to be run after code finishes reading gamepad
     * this method runs approx every 35 ms
     * this section is for setting the motor power for all of the automatic
     * methods that we call in the gamepad section
     */
    private void runAllAutoMethods() {

        //Controls for motorTape - Also automatically sets the tapeLock after a specified amount of time
        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            // if it's the first time you loop after holding down the button.
            if ((motorTape.getPower() == 0) && !firstPressedDPad) {
                tapeStartTime = System.currentTimeMillis();
                firstPressedDPad = true;
                motorTape.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }
            if ((System.currentTimeMillis() - tapeStartTime) > 350) {    /*Test for THRESH, the time which it takes for the lock to unlock  ARBITRARY*/
                if (gamepad1.dpad_up) {
                    motorTape.setPower(1);
                } else {
                    if (motorTape.getCurrentPosition() > startPosTape + 40 || (!gamepad1.back && gamepad1.b)) {
                        motorTape.setPower(-1);
                        if (gamepad1.b) {
                            startPosTape = motorTape.getCurrentPosition();
                        }
                    } else
                        motorTape.setPower(0);
                }
            } else {
                motorTape.setPower(.8); //REVERSE THE MOTOR BEFORE DISENGAGING THE LOCK TO PREVENT GETTING STUCK
                servoLock.setPosition(LOCK_DISENGAGED);
            }
        } else {
            motorTape.setPower(0.0);
            if (motorTape.getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                servoLock.setPosition(LOCK_ENGAGED);
            }
            firstPressedDPad = false;
        }

    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                     main methods                                           //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private void triggerZipline() {
        if (isBlue) {
            if (atRestTriggers)
                servoRightZip.setPosition(RIGHT_ZIP_UP); //Needs resting and active servo positions
            else
                servoRightZip.setPosition(RIGHT_ZIP_DOWN);
        } else {
            if (atRestTriggers)
                servoLeftZip.setPosition(LEFT_ZIP_UP);
            else
                servoLeftZip.setPosition(LEFT_ZIP_DOWN);
        }
        atRestTriggers = !atRestTriggers;
    }

    private void toggleHarvester(double power) {
        if (motorHarvest.getPower() == 0.0)
            motorHarvest.setPower(power);
        else
            motorHarvest.setPower(0.0);
    }

    //stop all motors/servos and ends all processes
    private void cancelAll() {
        motorArm.setPower(0);
        motorHarvest.setPower(0);
        motorTape.setPower(0);
        motorSlide.setPower(0);
        setLeftDrivePower(0);
        setRightDrivePower(0);
        servoShuttle.setPosition(.5);
        servoConveyor.setPosition(.5);
        autoConveyorRunning = false;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                     support methods                                        //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////


    private void setLeftDrivePower(double power) {
        motorLeftFore.setPower(power);
        motorLeftAft.setPower(power);
    }

    private void setRightDrivePower(double power) {
        motorRightFore.setPower(power);
        motorRightAft.setPower(power);
    }


    //Default Scale Input Method created by FTC- We will use this one until someone creates a better one.
    //Used for scaling joysticks, basically is a floor function that is squared
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0)
            index = -index;
        if (index > 16)
            index = 16;


        double dScale;
        if (dVal < 0)
            dScale = -scaleArray[index];
        else
            dScale = scaleArray[index];

        return dScale;
    }

    //This method is a combination of autonomous and manual controls
    //Certain actions such as the conveyor run on button prompts while
    //Other automated actions such as shuttling run on their own
    private void scoreSet(Height height, boolean button) {  // can be either top mid or bot
        if (button) {
            if (scoreToggle == 0) {
                //extend slides to specified height
                extendSlide(height);
                //shuttle(isBlue);
                scoreToggle++;
                return;
            }

            if (scoreToggle == 1) {
                //start the conveyor
                if (isBlue)
                    servoConveyor.setPosition(1); //MAY BE 0
                else
                    servoConveyor.setPosition(0);
                autoConveyorRunning = true;
                scoreToggle++;
                return;
            }
            if (scoreToggle == 2) {
                //stop conveyor and return to retracted position
                servoConveyor.setPosition(.5);
                autoConveyorRunning = false;
                //shuttle(isBlue); //Runs using touch sensor
                motorSlide.setTargetPosition(startPosSlide);
                scoreToggle = 0;
                //shuttle(isBlue);
            }
            /**
             * NEED TO WRITE METHOD THAT CAN SHUTTLE THE DISPENSER TO A SPECIFIC DISTANCE - most likely
             * will be time based and refined through testing since continuous servos don't have
             * encoders or positions
             */
        }
    }

    private void loadDispenserSet() {
        runningLoadDispenser = true;
        motorHarvest.setPower(0);
        toggleShelf(false);
        setArmPosition(ArmPosition.DISPENSE);
    }

    private void loadDispenserRun() {
        timerHarvest.startRun(System.currentTimeMillis() + 300, 1); //ARBITRARY
        if (motorHarvest.getPower() == 0)
        setArmPosition(ArmPosition.MOUNTAIN);
    }

    //returns and updates are for testing purposes
    private void extendSlide(Height height) { //Can be either top mid or bot
        motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        if (height == Height.TOP) {
            motorSlide.setTargetPosition((int) (slideTopPosition * TICKS_IN_INCH_SLIDE));
            return;
        }

        if (height == Height.MID) {
            motorSlide.setTargetPosition((int) (slideMidPosition * TICKS_IN_INCH_SLIDE));
            return;
        }

        if (height == Height.BOT) {
            motorSlide.setTargetPosition((int) (slideBotPosition * TICKS_IN_INCH_SLIDE));
            return;
        }

        telemetry.addData("NOT A VALID HEIGHT FOR SLIDES", "FIX CODE");
        telemetry.update();
    }

    /*
 Runs tank drive and arcade drive
 If boolean is true, it runs tank if false, it runs arcade
 with tank, each joystick's y value controls each side
  ex. left joystick controls left wheels right joystick controls right wheels
  With Arcade drive, the left trigger y controls drive and right trigger x controls turning
  kind of like a video game
  */
    private void manualDriveControls(boolean usingTankDrive) {
        if (usingTankDrive) {
            if (gamepad1.right_trigger > .7 && (gamepad1.left_stick_y > .2 || gamepad1.left_stick_y < -.2)) {
                if (gamepad1.left_stick_y > 0)
                    setLeftDrivePower(CONSTANT_DRIVE_SPEED);
                else
                    setLeftDrivePower(-CONSTANT_DRIVE_SPEED);
            } else
                setLeftDrivePower(scaleInput(gamepad1.left_stick_y));

            if (gamepad1.right_trigger > .7 && (gamepad1.right_stick_y > .2 || gamepad1.right_stick_y < -.2)) {
                if (gamepad1.right_stick_y > 0)
                    setRightDrivePower(CONSTANT_DRIVE_SPEED);
                else
                    setRightDrivePower(-CONSTANT_DRIVE_SPEED);
            } else
                setRightDrivePower(scaleInput(gamepad1.right_stick_y));
        } else {
            if (gamepad1.right_trigger > .7 && (gamepad1.right_stick_y > .2 || gamepad1.right_stick_y < -.2)) {
                if (gamepad1.left_stick_y > 0) {
                    setLeftDrivePower(CONSTANT_DRIVE_SPEED);
                    setRightDrivePower(CONSTANT_DRIVE_SPEED);
                } else {
                    setLeftDrivePower(-CONSTANT_DRIVE_SPEED);
                    setRightDrivePower(-CONSTANT_DRIVE_SPEED);
                }
            } else {
                setLeftDrivePower(scaleInput(gamepad1.left_stick_y) + scaleInput(gamepad1.right_stick_x));
                setRightDrivePower(scaleInput(gamepad1.left_stick_y) - scaleInput(gamepad1.right_stick_x));
            }
        }
    }


    //Method for controlling dispenser movement to and from scoring buckets
    private void shuttle(boolean directionLeft) {
        if (!(slidePosition == SlidePosition.NEUTRAL)) { //if shuttle is currently in an out position it returns to mid
            if (Math.abs(servoShuttle.getPosition() - .5) == .5) {
                servoShuttle.setPosition(.5);
            } else {
                if (directionLeft) //not yet sure if this value is 0 or 1 need to test
                    servoShuttle.setPosition(1);
                else
                    servoShuttle.setPosition(0);
            }
        } else {
            if (Math.abs(servoShuttle.getPosition() - .5) == .5) { //if it's running to the left or right, (the power is 1 or 0) stops the servo
                servoShuttle.setPosition(.5); //shuts off power
            } else {
                if (directionLeft)
                    servoShuttle.setPosition(0); //not yet sure if this value is 0 or 1 need to test
                else
                    servoShuttle.setPosition(1);
            }
        }
    }

    private void shelfPositionToggle(boolean stow) {
        if (stow) {
            servoShelfLeft.setPosition(SHELF_STOW_LEFT);
            servoShelfRight.setPosition(SHELF_STOW_RIGHT);
        } else {
            servoShelfLeft.setPosition(SHELF_DISPENSE_LEFT);
            servoShelfRight.setPosition(SHELF_DISPENSE_RIGHT);
        }
    }

    private void setArmPosition(ArmPosition position) {
        if (ArmPosition.INITIALIZE == position) {
            motorArm.setTargetPosition(armInitPosition);
        }

        if (ArmPosition.DISPENSE == position) {
            motorArm.setTargetPosition(armDispensePosition);
        }

        if (ArmPosition.HARVEST == position) {
            motorArm.setTargetPosition(armHarvestPosition);
        }

        if (ArmPosition.MOUNTAIN == position) {
            motorArm.setTargetPosition(armMountainPosition);
        }
    }

    /**
     * updates slidePosition (the enum for keeping track of where the slide is)
     * and also stops the motor once it reaches the desired position by turning
     * off the power. This code runs every main loop in the sensorUpdates method
     */


    private void toggleFlaps(boolean up) {
        if (isBlue) {
            if (true) {
                servoLeftFlap.setPosition(FLAP_UP_LEFT);
            }
            else {
                servoLeftFlap.setPosition(FLAP_DOWN_LEFT);
            }
        }
        else {
            if (true) {
                servoRightFlap.setPosition(FLAP_UP_RIGHT);
            }
            else {
                servoRightFlap.setPosition(FLAP_DOWN_RIGHT);
            }
        }
    }

    private void toggleShelf(boolean stow) {
        if (stow) {
            servoShelfRight.setPosition(SHELF_STOW_RIGHT);
            servoShelfLeft.setPosition(SHELF_STOW_LEFT);
        }
        else {
            servoShelfRight.setPosition(SHELF_DISPENSE_RIGHT);
            servoShelfLeft.setPosition(SHELF_DISPENSE_RIGHT);
        }
    }

    /**
     * MAY NOT NEED
     */
    private void shuttleAutoStop() {
        if (slidePosition == SlidePosition.NEUTRAL) {
            if ((colorLeft.red() + colorLeft.green() + colorLeft.blue()) > ARBITRARYDOUBLE) {
                servoShuttle.setPosition(.5);
                if (isBlue) {
                    slidePosition = SlidePosition.RIGHT;
                } else {
                    slidePosition = SlidePosition.LEFT;
                }
            }
        } else {
            if ((colorLeft.red() + colorLeft.green() + colorLeft.blue()) > ARBITRARYDOUBLE) {
                servoShuttle.setPosition(.5);
            }
        }
    }
}
