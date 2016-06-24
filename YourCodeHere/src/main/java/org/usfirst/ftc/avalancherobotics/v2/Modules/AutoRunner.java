package org.usfirst.ftc.avalancherobotics.v2.Modules;

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
import java.util.ArrayList;

@TeleOp(name = "AutoRunner")
public class AutoRunner extends SynchronousOpMode {

    // Declare drive motors
    DcMotor motorLeftFore;
    DcMotor motorLeftAft;
    DcMotor motorRightFore;
    DcMotor motorRightAft;
    ArrayList<String> operations = new ArrayList<String>();

    @Override
    public void main() throws InterruptedException {

        telemetry.addData("Started", "Init");

        // Initialize drive motors
        motorLeftFore = hardwareMap.dcMotor.get("LeftFore");
        motorLeftAft = hardwareMap.dcMotor.get("LeftAft");
        motorRightFore = hardwareMap.dcMotor.get("RightFore");
        motorRightAft = hardwareMap.dcMotor.get("RightAft");

        DriveTrainController driveTrain = new DriveTrainController(motorLeftAft, motorRightAft, motorLeftFore, motorRightFore);

        driveTrain.setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);


        for (int i = 0; i < driveTrain.getMotors().size(); i++) {
            driveTrain.setTargetPosition(i, 0);
        }


        driveTrain.setLeftDrivePower(.4);
        driveTrain.setRightDrivePower(.4);

        String allOperations = readFromFile();

        telemetry.addData("Ops", allOperations);

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

        telemetry.addData("Ops", operations);

        driveTrain.resetEncoders();

        telemetry.update();

        waitForStart();

        while ((opModeIsActive() && operations.size() > 0)) {
            telemetry.addData("reachedTarget?", driveTrain.reachedTarget());
                //Remove the firstOperation and store it separately.
                //String currentOperation = operations.remove(operations.size() - 1);
                String currentOperation = operations.remove(operations.size() - 1);
                String direction = currentOperation.substring(0, 1);

                telemetry.addData("Current Op", currentOperation);

                //turn the string number into an int
                int ticks = Integer.parseInt(currentOperation.substring(1));
                int leftPositive = 1;
                int rightPositive = 1;

                switch (direction) {
                    case "f":
                        leftPositive = 1;
                        rightPositive = 1;
                        break;
                    case "b":
                        leftPositive = -1;
                        rightPositive = -1;
                        break;
                    case "l":
                        leftPositive = -1;
                        rightPositive = 1;
                        break;
                    case "r":
                        leftPositive = 1;
                        rightPositive = -1;
                        break;
                }
                driveTrain.resetEncoders();

                driveTrain.setTargetPosition(0, leftPositive * ticks);
                driveTrain.setTargetPosition(2, leftPositive * ticks);
                driveTrain.setTargetPosition(1, rightPositive * ticks);
                driveTrain.setTargetPosition(3, rightPositive * ticks);

            while (!driveTrain.reachedTarget()) {
                idle();
            }
            telemetry.addData("Done", operations);
        }

        //stop();


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
