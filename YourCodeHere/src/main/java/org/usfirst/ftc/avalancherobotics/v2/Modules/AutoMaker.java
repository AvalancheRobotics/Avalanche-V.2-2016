package org.usfirst.ftc.avalancherobotics.v2.Modules;

/**
 * Created by austinzhang on 6/3/16.
 */

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 * DESCRIPTION OF CLASS
 * the purpose of this class is to find the correct values that we need to program autonomous
 * this class runs as a teleop method, you can only drive straight and turn.
 * you can check distance you travel by hitting a button, it sends the distances back to the phone through telemetry.
 */

/**
 * CONTROLS - ONLY USES DRIVER (GAMEPAD1) CONTROLLER
 * Right Trigger - Move Forwards
 * Left Trigger - Move Backwards
 * Right Bumper - Turn Right
 * Left Bumper - Turn Right
 * A - Revert Last Move
 */

//Format Type of movement forward backward left right (f, b, l, r), number of ticks followed by t (2000t)
@TeleOp(name = "AutoMaker")
public class AutoMaker extends SynchronousOpMode {


    DcMotor motorLeftFore;
    DcMotor motorRightFore;
    DcMotor motorLeftAft;
    DcMotor motorRightAft;
    DriveTrainController driveTrain;
    boolean previousAction;

    boolean currentAction; //
    boolean firstAction = true;

    OutputStreamWriter outputStreamWriter;


    public void initialization() {
        try {
            outputStreamWriter = new OutputStreamWriter(MyApplication.getContext().openFileOutput("auto.txt", Context.MODE_PRIVATE));
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }


        try {
            outputStreamWriter.flush();
            telemetry.addData("flush", "success");
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("flush", "failed");
        }

        String s = readFromFile();

        hardwareMapping();

        telemetry.addData("file", s);
        telemetry.addData("Out", outputStreamWriter);
        telemetry.update();
    }

    //Initialize and Map All Hardware
    private void hardwareMapping() {
        motorLeftAft = hardwareMap.dcMotor.get("LeftAft");
        motorLeftFore = hardwareMap.dcMotor.get("LeftFore");
        motorRightAft = hardwareMap.dcMotor.get("RightAft");
        motorRightFore = hardwareMap.dcMotor.get("RightFore");

        driveTrain = new DriveTrainController(motorLeftAft, motorRightAft, motorLeftFore, motorRightFore);


        // Reset encoders
        driveTrain.resetEncoders();

    }

    @Override
    public void main() throws InterruptedException {
        initialization();

        waitForStart();

        while (opModeIsActive()) {

            if (updateGamepads()) {
                previousAction = currentAction;
                boolean firstAct = firstAction;


                if (ScaleInput.scale(gamepad1.left_trigger + gamepad1.right_trigger) > 0) {
                    if (gamepad1.left_trigger > gamepad1.right_trigger) {
                        driveTrain.manualDrive(-gamepad1.left_trigger, -gamepad1.left_trigger);
                    } else {
                        driveTrain.manualDrive(gamepad1.right_trigger, gamepad1.right_trigger);
                    }
                    currentAction = true;
                    firstAction = false;
                }


                //turn right
                if (gamepad1.right_bumper) {
                    driveTrain.setLeftDrivePower(.2);
                    driveTrain.setRightDrivePower(-.2);
                    currentAction = false;
                    firstAction = false;
                }

                //turn left
                if (gamepad1.left_bumper) {
                    driveTrain.setLeftDrivePower(-.2);
                    driveTrain.setRightDrivePower(.2);
                    currentAction = false;
                    firstAction = false;
                }

                if (!gamepad1.left_bumper && !gamepad1.right_bumper && !(ScaleInput.scale(gamepad1.left_trigger + gamepad1.right_trigger) > 0)) {
                    driveTrain.setLeftDrivePower(0);
                    driveTrain.setRightDrivePower(0);
                }


                if ((previousAction != currentAction) && !firstAct) {
                    int ticks = Math.abs(driveTrain.getEncoderValue(0)) + Math.abs(driveTrain.getEncoderValue(1)) + Math.abs(driveTrain.getEncoderValue(2)) + Math.abs(driveTrain.getEncoderValue(3));
                    ticks = ticks / 4;
                    String message;
                    if (previousAction) {
                        if (driveTrain.getEncoderValue(0) > 0) {
                            message = "f";
                        } else {
                            message = "b";
                        }
                    } else {
                        if (driveTrain.getEncoderValue(0) > 0) {
                            message = "r";
                        } else {
                            message = "l";
                        }
                    }

                    message = message + ticks + "t";
                    writeToFile(message);
                    telemetry.addData("encoder1", driveTrain.getEncoderValue(0));
                    driveTrain.resetEncoders();
                    telemetry.addData("encoder2", driveTrain.getEncoderValue(0));
                }


                if (gamepad1.b) {
                    try {
                        outputStreamWriter.flush();
                        telemetry.addData("Read", readFromFile());
                    } catch (IOException e) {
                        telemetry.addData("Read", "Failed");
                    }
                }


                if (gamepad1.a) {
                    telemetry.addData("wait", "done");
                    undoLastMove();
                }

                if (gamepad1.x) {
                    forceWrite();
                }
            }
            telemetry.update();
            idle();
        }

        idle();
    }

    private void forceWrite() {
        int ticks = Math.abs(driveTrain.getEncoderValue(0)) + Math.abs(driveTrain.getEncoderValue(1)) + Math.abs(driveTrain.getEncoderValue(2)) + Math.abs(driveTrain.getEncoderValue(3));
        ticks = ticks / 4;
        String message;
        if (previousAction) {
            if (driveTrain.getEncoderValue(0) > 0) {
                message = "f";
            } else {
                message = "b";
            }
        } else {
            if (driveTrain.getEncoderValue(0) > 0) {
                message = "r";
            } else {
                message = "l";
            }
        }

        message = message + ticks + "t";
        writeToFile(message);
        telemetry.addData("encoder1", driveTrain.getEncoderValue(0));
        driveTrain.resetEncoders();
        telemetry.addData("encoder2", driveTrain.getEncoderValue(0));

        firstAction = true;
    }

    private void writeToFile(String data) {
        try {
            outputStreamWriter.write(data);
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    private String readFromFile() {

        String ret = "";

        try {
            InputStream inputStream = MyApplication.getContext().openFileInput("auto.txt");

            if (inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString = "";
                StringBuilder stringBuilder = new StringBuilder();

                while ((receiveString = bufferedReader.readLine()) != null) {
                    stringBuilder.append(receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        } catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }

        return ret;
    }

    private void undoLastMove() throws InterruptedException {
        //Make sure there isn't anything buffered
        try {
            outputStreamWriter.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }

        driveTrain.setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);

        //Undo current move
        for (int i = 0; i < driveTrain.getMotors().size(); i++) {
            driveTrain.setTargetPosition(i, 0);
        }

        driveTrain.setRightDrivePower(.3);
        driveTrain.setLeftDrivePower(.3);



        while (!driveTrain.reachedTarget()) {
            idle();
            telemetry.addData(driveTrain.getMotors().get(0).getCurrentPosition() - driveTrain.getMotors().get(0).getTargetPosition() + "", driveTrain.getMotors().get(1).getCurrentPosition() - driveTrain.getMotors().get(1).getTargetPosition() + "");
            telemetry.addData(driveTrain.getMotors().get(2).getCurrentPosition() - driveTrain.getMotors().get(2).getTargetPosition() + "", driveTrain.getMotors().get(3).getCurrentPosition() - driveTrain.getMotors().get(3).getTargetPosition() + "");
            telemetry.update();
        }

        String s = readFromFile();
        int endIndex = s.lastIndexOf("t");

        if (endIndex == -1) {
            driveTrain.resetEncoders();
            driveTrain.setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            //Do nothing if there are no operations
            return;
        }

        driveTrain.resetEncoders();


        //Find the start index of the last operation performed
        int lastUseB = s.lastIndexOf("b");
        int lastUseF = s.lastIndexOf("f");
        int lastUseR = s.lastIndexOf("r");
        int lastUseL = s.lastIndexOf("l");
        int startIndex = Math.max(Math.max(lastUseB, lastUseF), Math.max(lastUseL, lastUseR));

        String operation = s.substring(startIndex, endIndex);

        //turn the string number into an int
        int ticks = Integer.parseInt(operation.substring(1));

        //Operations are reversed of what they normally are because you're undoing a move
        if (operation.substring(0, 1).equals("f")) {
            for (int i = 0; i < driveTrain.getMotors().size(); i++) {
                driveTrain.setTargetPosition(i, -ticks);
            }
        }

        else if (operation.substring(0, 1).equals("b")) {
            for (int i = 0; i < driveTrain.getMotors().size(); i++) {
                driveTrain.setTargetPosition(i, ticks);
            }
        }

        else if (operation.substring(0, 1).equals("l")) {
            driveTrain.setTargetPosition(0, ticks);
            driveTrain.setTargetPosition(1, -ticks);
            driveTrain.setTargetPosition(2, ticks);
            driveTrain.setTargetPosition(3, -ticks);
        }

        else if (operation.substring(0, 1).equals("r")) {
            driveTrain.setTargetPosition(0, -ticks);
            driveTrain.setTargetPosition(1, ticks);
            driveTrain.setTargetPosition(2, -ticks);
            driveTrain.setTargetPosition(3, ticks);
        }

        while (!driveTrain.reachedTarget()) {
            idle();
        }

        //Rebuild file without last operation

        String withoutLastOp = s.substring(0, startIndex);

        try {
            outputStreamWriter.close();
            outputStreamWriter = new OutputStreamWriter(MyApplication.getContext().openFileOutput("auto.txt", Context.MODE_PRIVATE));
            writeToFile(withoutLastOp);
        } catch (IOException e) {
            e.printStackTrace();
        }

        driveTrain.setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        driveTrain.setRightDrivePower(0);
        driveTrain.setLeftDrivePower(0);
        driveTrain.resetEncoders();

        firstAction = true;
    }

    @Override
    public void idle() throws InterruptedException
    {
        // Abort the world if the OpMode has been asked to stop
        if (this.isStopRequested()) {
            int ticks = Math.abs(driveTrain.getEncoderValue(0)) + Math.abs(driveTrain.getEncoderValue(1)) + Math.abs(driveTrain.getEncoderValue(2)) + Math.abs(driveTrain.getEncoderValue(3));
            ticks = ticks / 4;
            String message;
            if (previousAction) {
                if (driveTrain.getEncoderValue(0) > 0) {
                    message = "f";
                } else {
                    message = "b";
                }
            } else {
                if (driveTrain.getEncoderValue(0) > 0) {
                    message = "r";
                } else {
                    message = "l";
                }
            }

            message = message + ticks + "t";
            writeToFile(message);
            telemetry.addData("encoder1", driveTrain.getEncoderValue(0));
            driveTrain.resetEncoders();
            telemetry.addData("encoder2", driveTrain.getEncoderValue(0));

            try {
                outputStreamWriter.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
            throw new InterruptedException();

        }

        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }


}



