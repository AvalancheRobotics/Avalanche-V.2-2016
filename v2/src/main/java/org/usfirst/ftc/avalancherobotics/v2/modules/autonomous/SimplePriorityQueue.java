package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;

import java.util.ArrayList;

/**
 * This is a simple priority queue meant to be implemented in the AStar Pathfinding algorithm
 */
public class SimplePriorityQueue<E extends Comparable<E>> implements PriorityQueue<E> {
    ArrayList<E> list;

    public SimplePriorityQueue() {
        list = new ArrayList<E>();
    }

    @Override
    public boolean isEmpty() {
        return list.size() == 0;
    }

    @Override
    public void add(E obj) {
        list.add(obj);
        heapUp(list.size());
    }

    private E remove(int index) {
        return list.remove(index - 1);
    }

    private E set(int index, E element) {
        return list.set(index - 1, element);
    }

    private E get(int index) {
        return list.get(index - 1);
    }

    public boolean contains(E obj) {
        return list.contains(obj);
    }

    private ArrayList<E> returnList() {
        return list;
    }

    private void heapUp(int index) {
        if (index <= 1) {
            return;
        }
        if (get(index).compareTo(get(index / 2)) < 0) {
            E temp = get(index);
            set(index, get(index / 2));
            set(index / 2, temp);
            heapUp(index / 2);
        }
    }

    @Override
    public E peekMin() {
        if (list.size() == 0)
            return null;
        return get(1);
    }

    @Override
    public E removeMin() {
        if (list.size() == 0)
            return null;
        E temp = get(1);
        set(1, get(list.size()));
        list.remove(list.size() - 1);
        heapDown(1);
        return temp;
    }

    private void heapDown(int index) {

        if (list.size() > index * 2) {
            int tempIndex;
            if (get(index*2).compareTo(get(index*2+1)) < 0) {
                tempIndex = index * 2;
            }
            else {
                tempIndex = index * 2 + 1;
            }
            E temp = get(index);
            if (get(index).compareTo(get(tempIndex)) > 0) {
                set(index, get(tempIndex));
                set(tempIndex, temp);
                heapDown(tempIndex);
            }
        }

        if (list.size() == index * 2 && get(index).compareTo(get(index * 2)) > 0) {
            E temp = get(index);
            set(index, get(index * 2));
            set(index * 2, temp);

        }

    }


}
