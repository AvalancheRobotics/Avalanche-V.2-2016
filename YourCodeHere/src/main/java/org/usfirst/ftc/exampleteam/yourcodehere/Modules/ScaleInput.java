package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

/**
 * Created by Austin on 6/3/2016.
 */
public class ScaleInput {
    public static double scale(double dVal) {
            if (dVal < .1 && dVal > -.1) {
                return 0;
            }
            if (dVal < -.95) {
                return -1;
            }
            if (dVal > .95) {
                return 1;
            }

            if (dVal > 0) {
                return Math.pow(dVal, 2);
            }
            else {
                return -Math.pow(dVal, 2);
            }

    }
}
