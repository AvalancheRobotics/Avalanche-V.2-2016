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

@Autonomous(name = "AutoRunner")
public class AutoRunner extends SynchronousOpMode {

    // Declare drive motors
    DcMotor motorLeftFore;
    DcMotor motorLeftAft;
    DcMotor motorRightFore;
    DcMotor motorRightAft;
    ArrayList<String> operations = new ArrayList<String>();

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

        DriveTrainController driveTrain = new DriveTrainController(motorLeftAft, motorRightAft, motorLeftFore, motorRightFore);

        driveTrain.resetEncoders();
        driveTrain.setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < driveTrain.getMotors().size(); i++) {
            driveTrain.getMotors().get(i).setTargetPosition(0);
        }

        driveTrain.setLeftDrivePower(.8);
        driveTrain.setRightDrivePower(.8);


        String allOperations = readFromFile();

        if (allOperations.length() == 0) {
            //No operations in file
            return;
        }

        //remove end t
        allOperations = allOperations.substring(0, allOperations.length() - 1);

        while (allOperations.contains("t")) {
            operations.add(allOperations.substring(allOperations.lastIndexOf("t") + 1, allOperations.length()));
            allOperations = allOperations.substring(0, allOperations.lastIndexOf("t"));
        }
        operations.add(allOperations);

        waitForStart();


        while (operations.size() > 0) {
            //Remove the firstOperation and store it seperately.
            String currentOperation = operations.remove(operations.size() - 1);

            //turn the string number into an int
            int ticks = Integer.parseInt(currentOperation.substring(1));

            if (currentOperation.substring(0, 1).equals("f")) {
                for (int i = 0; i < driveTrain.getMotors().size(); i++) {
                    driveTrain.getMotors().get(i).setTargetPosition(ticks);
                }
            }

            if (currentOperation.substring(0, 1).equals("r")) {
                for (int i = 0; i < driveTrain.getMotors().size(); i++) {
                    driveTrain.getMotors().get(i).setTargetPosition(-ticks);
                }
            }

            if (currentOperation.substring(0, 1).equals("l")) {
                for (int i = 0; i < driveTrain.getLeftMotors().size(); i++) {
                    driveTrain.getLeftMotors().get(i).setTargetPosition(-ticks);
                    driveTrain.getRightMotors().get(i).setTargetPosition(ticks);
                }
            }

            if (currentOperation.substring(0, 1).equals("r")) {
                for (int i = 0; i < driveTrain.getLeftMotors().size(); i++) {
                    driveTrain.getLeftMotors().get(i).setTargetPosition(ticks);
                    driveTrain.getRightMotors().get(i).setTargetPosition(-ticks);
                }
            }

            while (driveTrain.isBusy()) {
                idle();
            }

            driveTrain.resetEncoders();

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

}
