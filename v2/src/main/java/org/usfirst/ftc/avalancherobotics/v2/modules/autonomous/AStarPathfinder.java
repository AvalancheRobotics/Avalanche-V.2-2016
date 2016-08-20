package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;


import java.util.LinkedList;

/**
 * A star Pathfinder
 * TESTED AND WORKING
 * COULD USE CLEANING
 */
public class AStarPathfinder {
    private Store store;

    private static final int DIAG_COST = 14;
    private static final int STRAIGHT_COST = 10;

    private SimplePriorityQueue open;
    private LinkedList<Cell> closed;


    public AStarPathfinder(Store store) {
        this.store = store;

        open = new SimplePriorityQueue<>();

        closed = new LinkedList<>();
    }

    public LinkedList<Location> findPath(Location start, Location end) {

        // Add start node to open
        open.add(store.field[start.getX()][start.getY()]);

        //loop
        while (!open.isEmpty()) {
            //Remove current cell from open and add to closed
            Cell current = (Cell) open.peekMin();
            closed.add((Cell) open.removeMin());

            //If path is found
            if (current == store.field[end.getX()][end.getY()]) {
                return tracePath(current);
            }

            // For each neighbor of the current node
            for (int x = -1; x < 2; x++) {
                for (int y = -1; y < 2; y++) {


                    Location currentLoc = current.location;
                    Cell neighbour = store.field[currentLoc.getX() + x][currentLoc.getY() + y];

                    //Make sure that the cell examined is not an obstacle and has not already been examined. And isn't the current cell
                    //Nothing happens if these criteria are not met
                    if (neighbour != current && neighbour.traversable && !cellInList(neighbour, closed)) {

                        //Calculate inherited score/dist from start
                        int gScore;

                        //If Movement is straight/not diagonal
                        if (x * y == 0) {
                            gScore = current.gCost + STRAIGHT_COST;
                        }
                        //If Movement is diagonal/not straight
                        else {
                            gScore = current.gCost + DIAG_COST;
                        }

                        //Calculate H Score
                        int hScore = findHeuristicCost(neighbour.location, end);

                        //If the cell is in the open queue, or if the new path to the cell is less than the old one.
                        if (!cellInQueue(neighbour, open) || neighbour.finalCost > gScore + hScore) {

                            //sets the parent to the current node
                            neighbour.parent = current;

                            //Assign new g cost
                            neighbour.gCost = gScore;

                            //Assign new f cost
                            neighbour.finalCost = findHeuristicCost(neighbour.location, end) + neighbour.gCost;

                            if (!cellInQueue(neighbour, open)) {
                                open.add(neighbour);
                            }
                        }

                    }
                }
            }
        }

        return null;
    }

    private LinkedList<Location> tracePath(Cell finalCell) {
        LinkedList<Location> path = new LinkedList<>();
        Cell currentCell = finalCell;
        while (currentCell != null) {
            path.add(0, currentCell.location);
            currentCell = currentCell.parent;
        }

        return path;
    }


    //Bad methods pretty useless
    /**GET RID OF*/

    private boolean cellInList(Cell cell, LinkedList<Cell> list) {
        return list.contains(cell);
    }

    private boolean cellInQueue(Cell cell, SimplePriorityQueue queue) {
        return queue.contains(cell);
    }

    /** GET RID OF*/

    private int findHeuristicCost(Location start, Location end) {
        int distMax = Math.max(Math.abs(start.getX() - end.getX()), Math.abs(start.getY() - end.getY()));
        int distMin = Math.min(Math.abs(start.getX() - end.getX()), Math.abs(start.getY() - end.getY()));
        return DIAG_COST * distMin + STRAIGHT_COST * (distMax - distMin);
    }

}
