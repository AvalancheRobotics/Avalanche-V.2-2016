package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

import org.usfirst.ftc.avalancherobotics.v2.modules.DriveTrainController;


/**
 * Created by austinzhang on 8/13/16.
 */
public class Store {
    long startTime;
    int drift;
    int offset;
    static Location STARTING_POSITION = new Location(0,0);
    static final double WHEEL_DIAMETER = 10.16; //IN CM
    static final int TICKS_PER_ROTATION = 1120;    Cell[][] field;
    static final double TICKS_PER_CM = 35.09;

    Location lastPosition;

    DriveTrainController driveTrain;
    GyroSensor gyro;
    I2cDevice range;
    I2cDeviceReader rangeReader;
    AStarPathfinder pathfinder;
    ParsePath pathrover;
    Scanner scanner;

    byte[] rangeReadings;

}
