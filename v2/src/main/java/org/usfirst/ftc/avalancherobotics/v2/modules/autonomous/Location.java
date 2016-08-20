package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;

/**
 * Location
 */
public class Location {
    private int x;
    private int y;

    public Location(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public int getX() {return x;}

    public int getY() {return y;}

    public String toString() {
        return "("+ x + ", " + y + ")";
    }

    public boolean equals(Object o) {
        return x == ((Location) o).getX() && y == ((Location) o).getY();
    }
}
