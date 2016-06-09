package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

import android.media.MediaPlayer;
import android.util.Log;

import com.qualcomm.robotcore.hardware.*;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;
import org.usfirst.ftc.exampleteam.yourcodehere.Modules.DriveTrainController;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;

@Autonomous(name = "TeleGhost")
public class TeleGhost extends SynchronousOpMode {

    // Declare drive motors
    DcMotor motorLeftFore;
    DcMotor motorLeftAft;
    DcMotor motorRightFore;
    DcMotor motorRightAft;
    ArrayList<Double>[] servoPosition;
    ArrayList<Double>[] motorPower;
    ArrayList<Integer>[] motorMode;
    ArrayList<Integer>[] motorPosition;
    long[] time;

    @Override
    public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */

        // Initialize drive motors
        motorLeftFore = hardwareMap.dcMotor.get("LeftFore");
        motorLeftAft = hardwareMap.dcMotor.get("LeftAft");
        motorRightFore = hardwareMap.dcMotor.get("RightFore");
        motorRightAft = hardwareMap.dcMotor.get("RightAft");


        waitForStart();


    }

    private String readFromFile() {

        String ret = "";

        try {
            InputStream inputStream = MyApplication.getContext().openFileInput("ghost.txt");

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

    private String parser(String message) {
        ArrayList<String> operationsAndTime = new ArrayList<String>();

        String allOperations = readFromFile();

        if (allOperations.length() == 0) {
            //No operations in file
            stop();
        }

        //Parse into individual state+time
        while (allOperations.contains("b")) {
            operationsAndTime.add(allOperations.substring(allOperations.lastIndexOf("b") + 1, allOperations.length()));
            allOperations = allOperations.substring(0, allOperations.lastIndexOf("t"));
        }
        operationsAndTime.add(allOperations);

        servoPosition = new ArrayList[operationsAndTime.size()];
        time = new long[operationsAndTime.size()];

        String[] servoPos = new String[operationsAndTime.size()];
        String[] motorPow = new String[operationsAndTime.size()];
        String[] motorMod = new String[operationsAndTime.size()];
        String[] motorTar = new String[operationsAndTime.size()];

        //Build array of timestamps
        for (int i = 0; i < operationsAndTime.size(); i++) {
            String opTime = operationsAndTime.get(i);
            time[i] = Integer.parseInt(opTime.substring(0, opTime.indexOf("s")));
            servoPos[i] = opTime.substring(opTime.indexOf("s") + 1, opTime.indexOf("p"));
            motorPow[i] = opTime.substring(opTime.indexOf("p") + 1, opTime.indexOf("m"));
            motorMod[i] = opTime.substring(opTime.indexOf("m") + 1, opTime.indexOf("t"));
            motorTar[i] = opTime.substring(opTime.indexOf("t") + 1);
        }

        //Convert to individual motors and servos

        for (int i = 0; i < servoPos.length; i++) {
            String servos = servoPos[i];
            while (servos.contains("s")) {
                servoPosition[i].add(Double.parseDouble(servos.substring(0, servos.indexOf("s"))));
                servos = servos.substring(servos.indexOf("s") + 1);
            }
            servoPosition[i].add(Double.parseDouble(servos));
        }

        for (int i = 0; i < motorPow.length; i++) {
            String motors = motorPow[i];
            while (motors.contains("p")) {
                motorPower[i].add(Double.parseDouble(motors.substring(0, motors.indexOf("p"))));
                motors = motors.substring(motors.indexOf("p") + 1);
            }
        }

        for (int i = 0; i < motorMod.length; i++) {
            String motors = motorMod[i];
            while (motors.contains("m")) {
                motorMode[i].add(Integer.parseInt(motors.substring(0, motors.indexOf("m"))));
                motors = motors.substring(motors.indexOf("m") + 1);
            }
        }

        for (int i = 0; i < motorTar.length; i++) {
            String motors = motorTar[i];
            while (motors.contains("t")) {
                motorPosition[i].add(Integer.parseInt(motors.substring(0, motors.indexOf("t"))));
                motors = motors.substring(motors.indexOf("t") + 1);
            }
        }
        return null;

    }

}
