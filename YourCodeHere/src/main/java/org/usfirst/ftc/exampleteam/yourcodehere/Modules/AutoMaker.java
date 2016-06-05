package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

/**
 * Created by austinzhang on 6/3/16.
 */

import android.content.Context;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

import java.io.BufferedReader;
import java.io.File;
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
    DcMotor motorSlide;
    DriveTrainController driveTrain;

    boolean drivingStraight;
    boolean firstAction = true;

    OutputStreamWriter outputStreamWriter;



    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
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

        //Deletes Previous Auto Operation
        MyApplication.getContext().deleteFile("auto.txt");

        try {
            outputStreamWriter = new OutputStreamWriter(MyApplication.getContext().openFileOutput("auto.txt", Context.MODE_PRIVATE));
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }

        hardwareMapping();

        waitForStart();


        // Go go gadget robot!
        while (opModeIsActive()) {

            boolean previousAction = drivingStraight;
            boolean firstAct = firstAction;

            if (ScaleInput.scale(gamepad1.left_trigger + gamepad1.right_trigger) > 0) {
                if (gamepad1.left_trigger > gamepad1.right_trigger) {
                    driveTrain.manualDrive(-gamepad1.left_trigger, -gamepad1.left_trigger);
                } else {
                    driveTrain.manualDrive(gamepad1.right_trigger, gamepad1.right_trigger);
                }
                drivingStraight = true;
                firstAction = false;
            }


            //turn right
            if (gamepad1.right_bumper) {
                driveTrain.setLeftDrivePower(.2);
                driveTrain.setRightDrivePower(-.2);
                drivingStraight = false;
                firstAction = false;
            }

            //turn left
            if (gamepad1.left_bumper) {
                driveTrain.setLeftDrivePower(-.2);
                driveTrain.setRightDrivePower(.2);
                drivingStraight = false;
                firstAction = false;
            }

            if (gamepad1.a) {
                undoLastMove();
            }


            if (previousAction != drivingStraight && !firstAct) {
                //Log if change actions

                //If you just finished driving straight
                if (previousAction) {
                    //If drove forward
                    if (driveTrain.getAverageEncoderValue() > 0) {
                        writeToFile("f" + Math.abs(driveTrain.getAverageEncoderValue()) + "t");
                    }
                    else {
                        //If drove backward
                        writeToFile("b" + Math.abs(driveTrain.getAverageEncoderValue()) + "t");
                    }
                }
                //If you just finished turning
                else {
                    if (driveTrain.getAverageEncoderValueLeft() > 0) {
                        //turned right
                        writeToFile("r" + Math.abs(driveTrain.getAverageEncoderValueLeft()));
                    }
                    else {
                        //turned left
                        writeToFile("l" + Math.abs(driveTrain.getAverageEncoderValueRight()));
                    }
                }

                driveTrain.resetEncoders();
            }

            idle();

        }

        try {
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }

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


        //Undo current move
        for (int i = 0; i < driveTrain.getMotors().size(); i++) {
            driveTrain.getMotors().get(i).setTargetPosition(0);
        }

        driveTrain.setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);

        driveTrain.setRightDrivePower(.7);
        driveTrain.setLeftDrivePower(.7);


        while (driveTrain.isBusy()) {
            idle();
        }

        String s = readFromFile();
        int endIndex = s.lastIndexOf("t");

        if (endIndex == -1) {
            //Do nothing if there are no operations
            return;
        }


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
        if (operation.substring(0,1).equals("f")) {
            for (int i = 0; i < driveTrain.getMotors().size(); i++) {
                driveTrain.getMotors().get(i).setTargetPosition(-ticks);
            }
        }

        if (operation.substring(0,1).equals("r")) {
            for (int i = 0; i < driveTrain.getMotors().size(); i++) {
                driveTrain.getMotors().get(i).setTargetPosition(ticks);
            }
        }

        if (operation.substring(0,1).equals("l")) {
            for (int i = 0; i < driveTrain.getLeftMotors().size(); i++) {
                driveTrain.getLeftMotors().get(i).setTargetPosition(ticks);
                driveTrain.getRightMotors().get(i).setTargetPosition(-ticks);
            }
        }

        if (operation.substring(0,1).equals("r")) {
            for (int i = 0; i < driveTrain.getLeftMotors().size(); i++) {
                driveTrain.getLeftMotors().get(i).setTargetPosition(-ticks);
                driveTrain.getRightMotors().get(i).setTargetPosition(ticks);
            }
        }

        while (driveTrain.isBusy()) {
            idle();
        }

        //Rebuild file without last operation

        String withoutLastOp = s.substring(0, startIndex);

        try {
            outputStreamWriter.close();
            outputStreamWriter = new OutputStreamWriter(MyApplication.getContext().openFileOutput("auto.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(withoutLastOp);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}

