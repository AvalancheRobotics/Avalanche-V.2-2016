package org.usfirst.ftc.avalancherobotics.v2.modules;

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorController;

import java.io.IOException;
import java.io.OutputStreamWriter;

/**
 * Created by austinzhang on 6/6/16.
 */
public class StateLogger {

    OutputStreamWriter outputStreamWriter;
    long startTime;
    boolean firstLog;

    public StateLogger(String fileName) {
        try {
            outputStreamWriter = new OutputStreamWriter(MyApplication.getContext().openFileOutput(fileName, Context.MODE_PRIVATE));
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }

        firstLog = true;
    }

    private void writeToFile(String data) {
        try {
            outputStreamWriter.write(data);
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }
/*
    private void log(HardwareState state) {
        if (firstLog) {
            startTime = System.nanoTime();
        }

        String composition = composer(state);
        writeToFile(composition);
        firstLog = false;
    }

    //Stored as s + servoPosition + p + motorPower + m + motorMode (0, 1, 2) runtoposition, runusingencoders, runwithoutencoders + t + motorTargetPosition
    private String composer(HardwareState state) {
        String composition = "";
        //Marks beginning of state
        composition = "b";

        composition = composition + (System.nanoTime() - startTime);


        //Marks beginning of servoPosition
        for (int i = 0; i < state.getServoPosition().length; i++) {
            composition = composition + "s";
            composition = composition + state.getServoPosition()[i];
        }

        for (int i = 0; i < state.getMotorPower().length; i++) {
            composition = composition + "p";
            composition = composition + state.getMotorPower()[i];
        }

        for (int i = 0; i < state.getMotorModes().length; i++) {
            composition = composition + "m";
            DcMotorController.RunMode mode = state.getMotorModes()[i];
            if (mode.equals(DcMotorController.RunMode.RUN_TO_POSITION)) {
                composition = composition + 0;
            }
            if (mode.equals(DcMotorController.RunMode.RUN_USING_ENCODERS)) {
                composition = composition + 1;
            }
            if (mode.equals(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS)) {
                composition = composition + 2;
            }
        }

        for (int i = 0; i < state.getMotorTargetPositions().length; i++) {
            composition = composition + "t";
            composition = composition + state.getMotorTargetPositions()[i];
        }

        return composition;
    }

    public void close() {
        try {
            outputStreamWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    */
}
