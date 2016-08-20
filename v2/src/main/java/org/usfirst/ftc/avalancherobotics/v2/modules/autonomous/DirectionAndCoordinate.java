package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;

/**
 * Created by austinzhang on 8/13/16.
 */
public class DirectionAndCoordinate {

    private Direction direction;
    private Location location;

    public DirectionAndCoordinate(Direction direction, Location location) {
        this.direction = direction;
        this.location = location;
    }

    public void setCoordinate(Location location) {
        this.location = location;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }



}
