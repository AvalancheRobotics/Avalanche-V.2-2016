package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;
/**
 * TO DO LIST:
 * Debug and Test.
 * <p>
 * <p>
 * /**
 * Version 1.0 of Team Avalanche 6253's TeleOp program for Robot version 2.0.
 * Currently most distance and position values are arbitrary due to not having a complete robot we can test values on.
 * Nearly finished, all methods save loadDispenser are written, needs some fixing, values, and a lot of testing.
 * UNLIKE NORMAL TELEOP, USES FTC'S BUILT IN MOVE_TO_POSITION which causes the motor to move more slowly but is also more accurate and easier to code than custom PID
 */

//MAKE TRIGGERS LESS SENSITIVE


/**
 * DO
 * NOT
 * RUN
 * THIS
 * CODE
 * UNTIL
 * VALUES
 * HAVE
 * BEEN
 * ASSIGNED
 * AND
 * POSITIONS
 * ARE
 * FIGURED
 * OUT
 * OR
 * BAD
 * THINGS
 * WILL
 * HAPPEN
 */

@TeleOp(name = "TeleOpNoPID")
public class TeleOpNoPID extends SynchronousOpMode {

    //Enums

    // Variables

    //shows what step the loadDispenser method is on
    private boolean loadStep1Complete = false;
    private long loadStep2StartTime = 0;

    //sees if this is the first time pressing button
    private boolean firstPressedDPad = true;
    private boolean firstPressedAutoSlide = true;

    //Running Methods
    private boolean runningLoadDispenser = false;
    private boolean runningAutoSlide = false;
    private boolean runningExtend = false;

    //shows what step score method is on
    private int scoreToggle = 0;

    //How long it has been since the Tape has been called
    private long tapeStartTime = 0;

    // defaults to blue alliance
    private boolean isBlue = false;

    //tells whether triggers are in resting position or are down/active, starts at rest
    private boolean atRestTriggers = true;

    //used when climbing/descending the mountain to spin wheels backwards so we don't get stuck
    private final double CONSTANT_UP_SPEED = .7;
    private final double HARVEST_SPEED = .7;

    //motor values (measured in ticks)

    //Declare starting positions for tape, arm, and slide motors
    private int startPosTape;
    private int startPosArm;
    private int startPosSlide;
    private int maxTapeLength;

    //Motor Positions
    private double slideBotPosition;
    private double slideMidPosition;
    private double slideTopPosition;
    private int armInitPosition = -2335;
    private int armHarvestPosition = 0;
    private int armDispensePosition = -1650;
    private int armMountainPosition = -300; //-1965- old position, new one prevents tipping
    private int armDescendPosition = -1000;
    //farthest slide can extend without damage
    private int maxSlideLength;
    //length which tape extends to to hang

    //Servo Values
    private static final double RIGHT_ZIP_UP = 0.753;
    private static final double RIGHT_ZIP_DOWN = 0;
    private static final double LEFT_ZIP_UP = 0.293;
    private static final double LEFT_ZIP_DOWN = 0.216667;
    private static final double LOCK_ENGAGED = 1.0;
    private static final double LOCK_DISENGAGED = .178333;
    private static final double SHELF_STOW_LEFT = .287;
    private static final double SHELF_STOW_RIGHT = .713;
    private static final double SHELF_DISPENSE_LEFT = .476333;
    private static final double SHELF_DISPENSE_RIGHT = .523666;
    private static final double DISPENSER_NEUTRAL = 0.5;
    private static final double DISPENSER_LEFT = 0.6693;
    private static final double DISPENSER_RIGHT = 0.3577;

    private double pad1LeftStickX = 0;
    private double pad1LeftStickY = 0;
    private double pad1RightStickX = 0;
    private double pad1RightStickY = 0;
    private double pad2LeftStickX = 0;
    private double pad2LeftStickY = 0;
    private double pad2RightStickX = 0;
    private double pad2RightStickY = 0;
    private double pad1LeftTrigger = 0;
    private double pad1RightTrigger = 0;
    private double pad2LeftTrigger = 0;
    private double pad2RightTrigger = 0;
    private float disconnectTimer = 0;

    // Declare drive motors
    DcMotor motorLeftFore;
    DcMotor motorLeftAft;
    DcMotor motorRightFore;
    DcMotor motorRightAft;

    // Declare drawer slide motor and servos
    // motor extends/retracts slides
    // servoLock is the servo for locking the tape measure in place once hanging.
    DcMotor motorSlide;
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

    //Shelf servos
    Servo servoLeftRamp;
    Servo servoRightRamp;

    //Servo for angling dispenser
    Servo servoTilt;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        // Initialize drive motors
        motorLeftFore = hardwareMap.dcMotor.get("LeftFore");
        motorLeftAft = hardwareMap.dcMotor.get("LeftAft");
        motorRightFore = hardwareMap.dcMotor.get("RightFore");
        motorRightAft = hardwareMap.dcMotor.get("RightAft");

        //Left and right motors are on opposite sides and must spin opposite directions to go forward
        motorLeftAft.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFore.setDirection(DcMotor.Direction.REVERSE);

        // Initialize drawer slide motor and servos
        motorSlide = hardwareMap.dcMotor.get("Slide");

        // Initialize tape measure motor, servo tape, and servo lock.
        motorTape = hardwareMap.dcMotor.get("Tape");
        servoTape = hardwareMap.servo.get("TapeAngle");
        servoLock = hardwareMap.servo.get("Lock");

        // Initialize motor that spins the harvester
        motorHarvest = hardwareMap.dcMotor.get("Harvest");

        // Initialize motor that raises and lowers the collection arm
        motorArm = hardwareMap.dcMotor.get("Arm");

        // Initialize zipline flipping servos
        servoLeftZip = hardwareMap.servo.get("LeftZip");
        servoRightZip = hardwareMap.servo.get("RightZip");

        // Initialize shelf servos
        servoLeftRamp = hardwareMap.servo.get("LeftRamp");
        servoRightRamp = hardwareMap.servo.get("RightRamp");

        // Initialize dispenser angle servo
        servoTilt = hardwareMap.servo.get("Tilt");

        // Reset encoders
        this.motorLeftAft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorLeftFore.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorTape.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorRightAft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorRightFore.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        this.motorSlide.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        //Set Runmode for all motors to run using encoders
        motorLeftFore.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRightFore.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeftAft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRightAft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorTape.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorArm.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorHarvest.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        motorSlide.setPower(1);
        motorArm.setPower(.3);

        //keep track of the starting positions of arm, slide, and tape motors
        startPosArm = motorArm.getCurrentPosition();
        startPosSlide = motorSlide.getCurrentPosition();
        startPosTape = motorSlide.getCurrentPosition();
        motorSlide.setTargetPosition(startPosSlide);

        //Score heights for slides
        slideBotPosition = -2825;
        slideMidPosition = -2188;
        slideTopPosition = -7600; //ARBITRARY
        //maxSlideLength = startPosTape + 12000; //ARBITRARY

        motorArm.setTargetPosition(armInitPosition);

        //keep track of max length tape should extend
        //maxTapeLength = motorTape.getCurrentPosition() + 6000; //ARBITRARY
        //hangLength = motorTape.getCurrentPosition() + 5000; //ARBITRARY


        //initializes servos to their starting positions
        servoTilt.setPosition(DISPENSER_NEUTRAL);
        servoLeftZip.setPosition(LEFT_ZIP_UP);
        servoRightZip.setPosition(RIGHT_ZIP_UP);
        servoLeftRamp.setPosition(SHELF_DISPENSE_LEFT);
        servoRightRamp.setPosition(SHELF_DISPENSE_RIGHT);
        servoLock.setPosition(LOCK_ENGAGED);
        servoTape.setPosition(.5);

        //motorTape is on opposite side on v2 so we need to reverse the motor
        motorTape.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void main() throws InterruptedException {
        hardwareMapping();

        waitForStart();

        // Start TeleOp

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

            idle();
        }
    }

    /**
     * in the sensorChanges method we put the methods that depend on
     * sensor input do decide when to start, stop, etc.
     * It is the last method to run in the main loop
     */
    private void sensorChanges() {

    }

    private void manualMethods() {
        //Toggle Team (if we need to score on an opponent's ramp)
        if ((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b)) {
            telemetry.addData("team: ", "red");
            telemetry.update();
            isBlue = false;
        }
        if ((gamepad1.back && gamepad1.x) || (gamepad2.back && gamepad2.x)) {
            telemetry.addData("team: ", "blue");
            telemetry.update();
            isBlue = true;
        }


        //starts and stops harvester
        if (gamepad1.x && !gamepad1.back) { //so it doesn't run when we try to change teams (uses x)
            toggleHarvester(HARVEST_SPEED);
        }

        //toggles harvester spin direction
        if (gamepad1.left_bumper)
            motorHarvest.setPower(-HARVEST_SPEED);


        //toggles zip line position
        if (gamepad2.y)
            triggerZipline();


        //stops all motors and cancels all motor operations, basically a panic button that stops all functions
        if ((gamepad1.back && gamepad1.start) || (gamepad2.back && gamepad2.start))
            cancelAll();


        //Stops any auto methods using slides and manually controls power with joysticks
        if (scaleInput(gamepad2.left_stick_y) != 0) { //Threshold so you don't accidentally start running the slides manually
            if (!motorSlide.getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                motorSlide.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }
            motorSlide.setPower(scaleInput(gamepad2.left_stick_y));
        } else {
            if (!motorSlide.getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
                motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorSlide.setPower(.5);
            }
            motorSlide.setTargetPosition(motorSlide.getCurrentPosition());
        }

        //adjust angle of tape
        servoTape.setPosition(-scaleInput(gamepad2.right_stick_y / 2) + .5);

        //auto positions for tilt

        if (gamepad2.x && !gamepad2.back) {
            servoTilt.setPosition(DISPENSER_LEFT);
        }

        if (gamepad2.b && !gamepad2.back) {
            servoTilt.setPosition(DISPENSER_RIGHT);
        }

        if (gamepad2.a) {
            servoTilt.setPosition(DISPENSER_NEUTRAL);
        }

        if (gamepad2.dpad_up) {
            setArmPosition(ArmPosition.INITIALIZE);
            servoLeftRamp.setPosition(SHELF_DISPENSE_LEFT);
            servoRightRamp.setPosition(SHELF_DISPENSE_RIGHT);
        }

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
        if (gamepad1.right_bumper) {
            loadDispenserSet();
        }

        //score(Height.BOT, gamepad2.a);
        //score(Height.MID, gamepad2.b && !gamepad2.back);
        //score(Height.TOP, gamepad2.y);

        startHarvest(gamepad2.right_bumper);

        // auto tape extend
        if (gamepad1.a || gamepad1.y) {

            if (!runningAutoSlide) {
                if (gamepad1.a) {
                    runningExtend = false;
                }
                if (gamepad1.y) {
                    runningExtend = true;
                }
            }

            runningAutoSlide = !runningAutoSlide;
        }

    }


    /**
     * put all autonomous functions in here, to be run after code finishes reading gamepad
     * this method runs approx every 35 ms
     * this section is for setting the motor power for all of the automatic
     * methods that we call in the gamepad section
     */
    private void runAllAutoMethods() {


        //Read Joystick Data and Update Speed of Left and Right Motors
        //If joystick buttons are pressed, sets drive power to preset value
        //Reads joystick values and needs to update fast so not in manual methods

        /** ADDED PART THAT SHUTS OFF DRIVE IF DISCONNECTED*/
        //manualDriveControls(true); - NO AUTO STOP
        disconnectStopDrive();


        if (!runningAutoSlide) {
            firstPressedAutoSlide = true;
            tapeLengthManualControls();
        } else {
            tapeLengthAutoControls(runningExtend);
        }

        loadDispenserRun();


        //manual controls but needs to be run every time
        //manually adjust the tilt of dispenser
        if (gamepad2.right_trigger > .08 || gamepad2.left_trigger > .08) {
            if (gamepad2.right_trigger > gamepad2.left_trigger) {
                if (gamepad2.right_trigger > .15) {
                    servoTilt.setPosition(.5);
                }
                servoTilt.setPosition(Math.min((-gamepad2.right_trigger / 3) + .5, HARVEST_SPEED));
            } else {
                if (gamepad2.left_trigger > .15) {
                    servoTilt.setPosition(.5);
                }
                servoTilt.setPosition(Math.max(gamepad2.left_trigger / 3 + .5, .3));
            }
        }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                     main methods                                           //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private void tapeLengthAutoControls(boolean up) {
        // if it's the first time you loop after holding down the button.
        if (firstPressedAutoSlide) {
            tapeStartTime = System.currentTimeMillis();
            firstPressedAutoSlide = false;
        }
        if ((System.currentTimeMillis() - tapeStartTime) > 200) {
            if (up) {
                motorTape.setPower(1);
            } else {
                if (motorTape.getCurrentPosition() > startPosTape + 40) {
                    motorTape.setPower(-1);
                } else
                    motorTape.setPower(0);
            }
        } else {
            motorTape.setPower(-.5); //REVERSE THE MOTOR BEFORE DISENGAGING THE LOCK TO PREVENT GETTING STUCK
            servoLock.setPosition(LOCK_DISENGAGED);
        }
    }

    //runs in auto methods due to need for scanning time
    private void tapeLengthManualControls() {
        //Controls for motorTape - Also automatically sets the tapeLock after a specified amount of time
        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            // if it's the first time you loop after holding down the button.
            if ((motorTape.getPower() == 0) && !firstPressedDPad) {
                tapeStartTime = System.currentTimeMillis();
                firstPressedDPad = true;
            }
            if ((System.currentTimeMillis() - tapeStartTime) > 200) {
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
                motorTape.setPower(-.5); //REVERSE THE MOTOR BEFORE DISENGAGING THE LOCK TO PREVENT GETTING STUCK
                servoLock.setPosition(LOCK_DISENGAGED);
            }
        } else {
            motorTape.setPower(0.0);
            servoLock.setPosition(LOCK_ENGAGED);
            firstPressedDPad = false;
        }
    }

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

    //stop all motors/servos and ends all processes ONLY RUN IN CASE OF EMERGENCY- BASICALLY ENDS PROGRAM IN IT'S CURRENT STATE
    private void cancelAll() {
        motorArm.setPower(0);
        motorHarvest.setPower(0);
        motorTape.setPower(0);
        motorSlide.setPower(0);
        setLeftDrivePower(0);
        setRightDrivePower(0);
        servoTape.setPosition(.5);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                     support methods                                        //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private void setDriveMode(DcMotorController.RunMode driveMode) {
        motorLeftAft.setMode(driveMode);
        motorLeftFore.setMode(driveMode);
        motorRightAft.setMode(driveMode);
        motorRightFore.setMode(driveMode);
    }

    private void setLeftDrivePower(double power) {
        motorLeftFore.setPower(power);
        motorLeftAft.setPower(power);
    }

    private void setRightDrivePower(double power) {
        motorRightFore.setPower(power);
        motorRightAft.setPower(power);
    }

    private void setForePower(double power) {
        motorLeftFore.setPower(power);
        motorRightFore.setPower(power);
    }

    private void setAftPower(double power) {
        motorRightAft.setPower(power);
        motorLeftAft.setPower(power);
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
    private void score(Height height, boolean button) {  // can be either top mid or bot
        if (button) {
            if (scoreToggle == 0) {
                //extend slides to specified height
                extendSlide(height);
                toggleShelf(true);
                scoreToggle++;
                return;
            }

            if (scoreToggle == 1) {
                //tilt dispenser
                if (isBlue) {
                    servoTilt.setPosition(DISPENSER_RIGHT);
                } else {
                    servoTilt.setPosition(DISPENSER_LEFT);
                }
                scoreToggle++;
                return;
            }
            if (scoreToggle == 2) {
                //tilt dispenser back to neutral and return slides to starting position
                servoTilt.setPosition(DISPENSER_NEUTRAL);
                motorSlide.setTargetPosition(startPosSlide);
                scoreToggle = 0;
            }
        }
    }

    private void loadDispenserSet() {
        runningLoadDispenser = true;
        toggleShelf(false);
        setArmPosition(ArmPosition.DISPENSE);
    }

    private void loadDispenserRun() { //Run this constantly in a loop
        if (runningLoadDispenser) {
            if ((motorArm.getCurrentPosition() >= motorArm.getTargetPosition() - 20 && motorArm.getCurrentPosition() <= motorArm.getTargetPosition() + 20) && !loadStep1Complete) {
                loadStep1Complete = true;
                loadStep2StartTime = System.currentTimeMillis();
            }
            if (loadStep1Complete) {
                if (System.currentTimeMillis() - loadStep2StartTime < 700) //500 value can be adjusted ARBITRARY
                    motorHarvest.setPower(.25); //ARBITRARY - need to test for best speed for spinning out blocks
                else {
                    motorHarvest.setPower(0.0);
                    setArmPosition(ArmPosition.MOUNTAIN);
                    toggleShelf(true);
                    loadStep1Complete = false;
                    runningLoadDispenser = false;
                }
            } else {
                if (motorArm.getCurrentPosition() <= -270) {
                    motorHarvest.setPower(0);
                }
            }
        }
    }


    //returns and updates are for testing purposes
    private void extendSlide(Height height) { //Can be either top mid or bot
        motorSlide.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorSlide.setPower(1);
        if (height == Height.TOP) {
            motorSlide.setTargetPosition((int) (slideTopPosition));
            return;
        }

        if (height == Height.MID) {
            motorSlide.setTargetPosition((int) (slideMidPosition));
            return;
        }

        if (height == Height.BOT) {
            motorSlide.setTargetPosition((int) (slideBotPosition));
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
        if (usingTankDrive) { //tank drive
            if (scaleInput(gamepad1.left_stick_y) == 0 && scaleInput(gamepad1.right_stick_y) == 0 && gamepad1.right_trigger < .7 && gamepad1.left_trigger < .9) {
                if (motorLeftAft.getMode().equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                    setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    motorRightAft.setTargetPosition(motorRightAft.getCurrentPosition());
                    motorLeftAft.setTargetPosition(motorLeftAft.getCurrentPosition());
                    motorLeftFore.setTargetPosition(motorLeftFore.getCurrentPosition());
                    motorRightFore.setTargetPosition(motorRightFore.getCurrentPosition());
                    setLeftDrivePower(1);
                    setRightDrivePower(1);
                }
            } else {
                if (motorLeftAft.getMode().equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
                    setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                }

                if (gamepad1.right_trigger > .7 || gamepad1.left_trigger > .7) {
                    if (gamepad1.right_trigger > gamepad1.left_trigger) {
                        setForePower(CONSTANT_UP_SPEED / 2);
                        setAftPower(CONSTANT_UP_SPEED);
                    } else {
                        if (gamepad1.left_trigger > .2) {
                            setArmPosition(ArmPosition.DESCEND);
                        }

                        if (gamepad1.left_trigger > .9) {
                            setForePower(-1);
                            setAftPower(-1);
                        }
                    }

                } else {
                    setLeftDrivePower(scaleInput(gamepad1.left_stick_y));
                }

                if (gamepad1.right_trigger > .7 || gamepad1.left_trigger > .7) {
                    if (gamepad1.right_trigger > gamepad1.left_trigger) {
                        setForePower(CONSTANT_UP_SPEED / 2);
                        setAftPower(CONSTANT_UP_SPEED);
                    }
                } else {
                    setRightDrivePower(scaleInput(gamepad1.right_stick_y));
                }
            }


            //arcade drive
        } else {
            if (gamepad1.right_trigger > .7 && (gamepad1.right_stick_y > .2 || gamepad1.right_stick_y < -.2)) {
                if (gamepad1.left_stick_y > 0) {
                    setLeftDrivePower(CONSTANT_UP_SPEED);
                    setRightDrivePower(CONSTANT_UP_SPEED);
                } else {
                    setLeftDrivePower(-CONSTANT_UP_SPEED);
                    setRightDrivePower(-CONSTANT_UP_SPEED);
                }
            } else {
                setLeftDrivePower(scaleInput(gamepad1.left_stick_y) + scaleInput(gamepad1.right_stick_x));
                setRightDrivePower(scaleInput(gamepad1.left_stick_y) - scaleInput(gamepad1.right_stick_x));
            }
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

        if (ArmPosition.DESCEND == position) {
            motorArm.setTargetPosition(armDescendPosition);
        }
    }

    private void toggleShelf(boolean stow) {
        if (stow) {
            servoRightRamp.setPosition(SHELF_STOW_RIGHT);
            servoLeftRamp.setPosition(SHELF_STOW_LEFT);
        } else {
            servoRightRamp.setPosition(SHELF_DISPENSE_RIGHT);
            servoLeftRamp.setPosition(SHELF_DISPENSE_LEFT);
        }
    }

    private void startHarvest(boolean button) {
        if (button) {
            toggleShelf(true);
            setArmPosition(ArmPosition.HARVEST);
            motorHarvest.setPower(HARVEST_SPEED);
            runningLoadDispenser = false;
        }
    }

    //drive controls that stops drive motors if we disconnect
    private void disconnectStopDrive() {
        if (
                pad1LeftStickX == gamepad1.left_stick_x
                        && pad1LeftStickY == gamepad1.left_stick_y
                        && pad1LeftTrigger == gamepad1.left_trigger
                        && pad1RightStickX == gamepad1.right_stick_x
                        && pad1RightStickY == gamepad1.right_stick_y
                        && pad1RightTrigger == gamepad1.right_trigger
                        && pad2LeftStickX == gamepad2.left_stick_x
                        && pad2LeftStickY == gamepad2.left_stick_y
                        && pad2LeftTrigger == gamepad2.left_trigger
                        && pad2RightStickX == gamepad2.right_stick_x
                        && pad2RightStickY == gamepad2.right_stick_y
                        && pad2RightTrigger == gamepad2.right_trigger
                ) {
            if (System.currentTimeMillis() - disconnectTimer > 2000) {
                setRightDrivePower(0);
                setLeftDrivePower(0);
            }
        } else {
            disconnectTimer = System.currentTimeMillis();
            manualDriveControls(true);
        }

        pad1LeftStickY = gamepad1.left_stick_y;
        pad1LeftTrigger = gamepad1.left_trigger;
        pad1RightStickX = gamepad1.right_stick_x;
        pad1RightStickY = gamepad1.right_stick_y;
        pad1RightTrigger = gamepad1.right_trigger;
        pad2LeftStickX = gamepad2.left_stick_x;
        pad2LeftStickY = gamepad2.left_stick_y;
        pad2LeftTrigger = gamepad2.left_trigger;
        pad2RightStickX = gamepad2.right_stick_x;
        pad2RightStickY = gamepad2.right_stick_y;
        pad2RightTrigger = gamepad2.right_trigger;
    }
}
