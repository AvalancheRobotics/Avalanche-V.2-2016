package org.usfirst.ftc.exampleteam.yourcodehere.Modules;

/**
 * Created by Austin on 6/3/2016.
 */
public class Position {
    String positionName;
    int value;

    public Position(String name, int value) {
        positionName = name;
        this.value = value;
    }

    @Override
    public String toString() {
        return positionName;
    }

    public String getName() {
        return positionName;
    }

    public int getValue() {
        return value;
    }

    public boolean equals(Position other) {
        return getValue() == other.getValue();
    }

}
