package algorithms;

import java.awt.*;
import java.util.ArrayList;

public class MinHeap {


    public ArrayList<MinHeapObject> getList() {
        return list;
    }

    public static class MinHeapObject{
        protected Point pere;
        protected Point current;
        protected ArrayList<Edge> chemin;
        protected double distAvecPred; // Min heap determiné par ça!

        public MinHeapObject(Point pere, Point current, double distAvecPred) {
            this.pere = pere;
            this.current = current;
            this.distAvecPred = distAvecPred;
            this.chemin= new ArrayList<>();
        }

        public MinHeapObject() {
            this.pere = new Point();
            this.current = new Point();
            this.chemin = new ArrayList<>();
            this.distAvecPred = 0;
        }

        @Override
        public String toString() {
            return "MinHeapObject{" +
                    "dest=" + current +
                    ", distAvecPred=" + distAvecPred +
                    '}';
        }

        public Point getPere() {
            return pere;
        }

        public void setPere(Point pere) {
            this.pere = pere;
        }

        public Point getCurrent() {
            return current;
        }

        public void setCurrent(Point current) {
            this.current = current;
        }

        public ArrayList<Edge> getChemin() {
            return chemin;
        }

        public void setChemin(ArrayList<Edge> chemin) {
            this.chemin = chemin;
        }

        public double getDistAvecPred() {
            return distAvecPred;
        }

        public void setDistAvecPred(double distAvecPred) {
            this.distAvecPred = distAvecPred;
        }
    }

    private ArrayList<MinHeapObject> list;

    public MinHeap() {

        this.list = new ArrayList<MinHeapObject>();
    }

    public MinHeap(ArrayList<MinHeapObject> items) {

        this.list = items;
        buildHeap();
    }

    public void insert(MinHeapObject item) {

        list.add(item);
        int i = list.size() - 1;
        int parent = parent(i);

        while (parent != i && list.get(i).distAvecPred < list.get(parent).distAvecPred) {

            swap(i, parent);
            i = parent;
            parent = parent(i);
        }
    }

    public void buildHeap() {

        for (int i = list.size() / 2; i >= 0; i--) {
            minHeapify(i);
        }
    }

    public MinHeapObject extractMin() {

        if (list.size() == 0) {

            throw new IllegalStateException("MinHeap is EMPTY");
        } else if (list.size() == 1) {

            MinHeapObject min = list.remove(0);
            return min;
        }

        // remove the last item ,and set it as new root
        MinHeapObject min = list.get(0);
        MinHeapObject lastItem = list.remove(list.size() - 1);
        list.set(0, lastItem);

        // bubble-down until heap property is maintained
        minHeapify(0);

        // return min key
        return min;
    }

    public void decreaseKey(int i, MinHeapObject key) {

        if (list.get(i).distAvecPred < key.distAvecPred) {

            throw new IllegalArgumentException("Key is larger than the original key");
        }

        list.set(i, key);
        int parent = parent(i);

        // bubble-up until heap property is maintained
        while (i > 0 && list.get(parent).distAvecPred > list.get(i).distAvecPred) {

            swap(i, parent);
            i = parent;
            parent = parent(parent);
        }
    }

    private void minHeapify(int i) {

        int left = left(i);
        int right = right(i);
        int smallest = -1;

        // find the smallest key between current node and its children.
        if (left <= list.size() - 1 && list.get(left).distAvecPred < list.get(i).distAvecPred) {
            smallest = left;
        } else {
            smallest = i;
        }

        if (right <= list.size() - 1 && list.get(right).distAvecPred < list.get(smallest).distAvecPred) {
            smallest = right;
        }

        // if the smallest key is not the current key then bubble-down it.
        if (smallest != i) {

            swap(i, smallest);
            minHeapify(smallest);
        }
    }

    public MinHeapObject getMin() {

        return list.get(0);
    }

    public boolean isEmpty() {

        return list.size() == 0;
    }

    private int right(int i) {

        return 2 * i + 2;
    }

    private int left(int i) {

        return 2 * i + 1;
    }

    private int parent(int i) {

        if (i % 2 == 1) {
            return i / 2;
        }

        return (i - 1) / 2;
    }

    private void swap(int i, int parent) {

        MinHeapObject temp = list.get(parent);
        list.set(parent, list.get(i));
        list.set(i, temp);
    }

}
