package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Objects;

public class Tree2D {
  private Point root;
  private ArrayList<Tree2D> subtrees;

  public Tree2D (Point p, ArrayList<Tree2D> trees){
    this.root=p;
    this.subtrees=trees;
  }
  public Point getRoot(){
    return this.root;
  }
  public ArrayList<Tree2D> getSubTrees(){
    return this.subtrees;
  }
  public double distanceRootToSubTrees(){
    double d=0;
    for (int i=0;i<this.subtrees.size();i++) d+=subtrees.get(i).getRoot().distance(root);
    return d;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    Tree2D tree2D = (Tree2D) o;
    return Objects.equals(root, tree2D.root);
  }
}
