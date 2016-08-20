package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;

public interface PriorityQueue<E extends Comparable<E>>
{
    boolean isEmpty();
    void add(E obj);
    E peekMin();  //returns minimum value without removing it
    E removeMin();  //removes and returns minimum value
}
