package algorithms;

import java.awt.Point;
import java.util.*;

public class DefaultTeam {

  public class Graph{
    int vertex;// un vertex représente un indice dans HitPoints
    LinkedList<Integer> list[];

    public Graph(int vertex){
      this.vertex = vertex;
      list = new LinkedList[vertex];
      for (int i =0; i < vertex ; i ++ ) list[i] = new LinkedList<>();
    }

    public void addEdge(int source, int destination){
      list[source].addFirst(destination);
      list[destination].addFirst(source);
    }

    public void printGraph(ArrayList<Point> points ){
      for (int i =0; i < vertex; i++){
        if(list[i].size()>0){
          System.out.println("Vertex "+ points.get(i) + "is connected to: " );
          for (int j=0; j < list[i].size(); j++){
            System.out.println(points.get(list[i].get(j)) + " ");
          }
          System.out.println();
        }
      }
    }

  }

  public Graph convertEdgeListToAdjacencyList(ArrayList<Edge> edjL, ArrayList<Point> hitPoints){
    Graph graph = new Graph(hitPoints.size());
    ArrayList< Edge> edgesVisited = new ArrayList<>();
    for (Point p : hitPoints) {
     // System.out.println(" Point : "+ p.x + " , "+ p.y);
      for (Edge e: edjL) {
          if (e.p.equals(p) ){
            if(!graph.list[hitPoints.indexOf(p)].contains(hitPoints.indexOf(e.q)))
              graph.addEdge(hitPoints.indexOf(e.p), hitPoints.indexOf(e.q));
          }if( e.q.equals(p)) {
              if(!graph.list[hitPoints.indexOf(p)].contains(hitPoints.indexOf(e.p))) {
                graph.addEdge(hitPoints.indexOf(e.q), hitPoints.indexOf(e.p));
          }
        }
      }
      //trier(graph.list[hitPoints.indexOf(p)]);
    }
    return graph;
  }


  public Tree2D calculSteiner(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {
    /* Step 1 : construire un graphe pondéré complet  */
    int [][] shortestPaths = calculShortestPaths(points,edgeThreshold);
    ArrayList<Edge> K = new ArrayList<>();
    for (Point p: hitPoints) {
      for (Point q: hitPoints) {
        if (p.equals(q) || contains(K,p,q)) continue;
        double poids = plusCourtChemin(p,q,shortestPaths,points);
        K.add(new Edge(p,q,poids));
      }
    }
    /* Step 2 */
    ArrayList<Edge> T0 = Kruskal(hitPoints,K);
    /* Step 3 */
    T0 =remplaceByShortestPath(T0,shortestPaths,points);
    /* Step 4*/
    ArrayList<Edge> Tprim =  Kruskal(points,T0);
    /* Step 5 */
    return edgesToTree(Tprim,Tprim.get(0).p);
  }

  public Tree2D calculSteinerBudget(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {
/*    Tree2D tre = new Tree2D(hitPoints.get(1),new ArrayList<>());
    ArrayList e = new ArrayList();
    e.add(tre);
    Tree2D tree = new Tree2D(hitPoints.get(0),e);*/

    Point maisonMere = hitPoints.get(0);
    double budget = 1664;
    int [][] shortestPaths = calculShortestPaths(points,edgeThreshold);
    ArrayList<Edge> K = new ArrayList<>();
    for (Point p: hitPoints) {
      for (Point q: hitPoints) {
        if (p.equals(q) || contains(K,p,q)) continue;
        double poids = plusCourtChemin(p,q,shortestPaths,points);
        K.add(new Edge(p,q,poids));
      }
    }
    // Step 2
    ArrayList<Edge> T0 = Kruskal(hitPoints,K);
    // Step 3
    T0 =remplaceByShortestPathBudget(T0,shortestPaths,points,hitPoints,budget, maisonMere);
     //Step 4
    ArrayList<Edge> Tprim =  Kruskal(points,T0);
    // Step 5
    return edgesToTree(Tprim,Tprim.get(0).p);

  }




  public ArrayList<Edge> Kruskal(ArrayList<Point> points,ArrayList<Edge> edges) {
    //KRUSKAL ALGORITHM, NOT OPTIMAL FOR STEINER!
    edges = sort(edges);

    ArrayList<Edge> kruskal = new ArrayList<Edge>();
    Edge current;
    NameTag forest = new NameTag(points);
    while (edges.size()!=0) {
      current = edges.remove(0);
      if (forest.tag(current.p)!=forest.tag(current.q)) {
        kruskal.add(current);
        forest.reTag(forest.tag(current.p),forest.tag(current.q));
      }
    }

    return kruskal;
  }

  private ArrayList<Edge> remplaceByShortestPath(ArrayList<Edge> t0Graph, int[][] shortestPath, ArrayList<Point> points){
    ArrayList<Edge> remplacement = new ArrayList<>();
    for (Edge e : t0Graph) {

      int indexI = points.indexOf(e.p);
      int indexFinal = points.indexOf(e.q);
      int k = shortestPath[indexI][indexFinal];
      remplacement.add(new Edge(e.p,points.get(k)));
      while(k!=indexFinal){
        remplacement.add(new Edge(points.get(k), points.get(shortestPath[k][indexFinal])) );
        k= shortestPath[k][indexFinal];
      }
    }

    return remplacement;

  }

  private ArrayList<Edge> remplaceByShortestPathBudget(ArrayList<Edge> t0Graph, int[][] shortestPath,
                                                       ArrayList<Point> points, ArrayList<Point> hitPoints,
                                                       double budget, Point maisonMere){
    double budgetActuel=0;
    ArrayList<Edge> remplacement = new ArrayList<>();
    Graph graph = convertEdgeListToAdjacencyList(t0Graph,hitPoints);
    //graph.printGraph(hitPoints);

    // chaque indice correspond à l'indice dans hitPoints
    boolean[] marked = new boolean[hitPoints.size()];

    MinHeap fifo = new MinHeap();
    // add (adds one to the end)
    fifo.insert(new MinHeap.MinHeapObject(maisonMere, maisonMere,0));
    marked[0]= true;
    MinHeap.MinHeapObject s;


    while(fifo.getList().size()!=0){
      System.out.println();
      s = fifo.extractMin();
      System.out.println(" S: "+ s+ " distance "+ s.distAvecPred);
      double apres = budgetActuel+s.distAvecPred;
      System.out.println("budgetActuel "+budgetActuel+ "budget Apres: "+apres );
      if(budgetActuel+s.distAvecPred< budget){
        budgetActuel += s.distAvecPred;
        System.out.println("ajout de "+ s.pere+ ", "+ s.current);
        if(s.chemin.isEmpty()) remplacement.add(new Edge(s.pere, s.current));
        else{
          for (Edge e : s.chemin) {
            remplacement.add(e);
          }
        }
        System.out.println("remplacement : "+ remplacement);
      }
      else{
        System.out.println("RETOUR budget Actuel "+ budgetActuel);
        System.out.println("RETOUR points "+ remplacement);
        return remplacement;
      }
      // pour chaque voisin
      for(int j = 0; j < graph.list[hitPoints.indexOf(s.current)].size(); j++){
        MinHeap.MinHeapObject voisin = new MinHeap.MinHeapObject();

        System.out.println("voisin "+ hitPoints.get(graph.list[hitPoints.indexOf(s.current)].get(j)));
        // s'il est pas marque
        if(!marked[graph.list[hitPoints.indexOf(s.current)].get(j)]){
          int indexI = points.indexOf(s.current);
          int indexFinal = points.indexOf(hitPoints.get(graph.list[hitPoints.indexOf(s.current)].get(j)));
          int k = shortestPath[indexI][indexFinal];

          double distance = points.get(indexI).distance(points.get(k));

          voisin.getChemin().add(new Edge(points.get(indexI),points.get(k)) );
          while(k!=indexFinal){
            distance += points.get(k).distance(points.get(shortestPath[k][indexFinal]));
            System.out.println("");
           voisin.getChemin().add(new Edge(points.get(k), points.get(shortestPath[k][indexFinal])) );
            k= shortestPath[k][indexFinal];
          }
          System.out.println("distance: "+ distance);
          voisin.setPere(s.current);
          voisin.setCurrent(hitPoints.get(graph.list[hitPoints.indexOf(s.current)].get(j)));
          voisin.setDistAvecPred(distance);
          fifo.insert(voisin);
          marked[graph.list[hitPoints.indexOf(s.current)].get(j)]=true;
        }
      }
    }

    return remplacement;

  }

  private double plusCourtChemin(Point i, Point j, int[][] shortestPaths, ArrayList<Point> points) {
    int indexI = points.indexOf(i);
    int indexFinal = points.indexOf(j);
    int k = shortestPaths[indexI][indexFinal];
    double weight = i.distance(points.get(k));
    while(k!=indexFinal){
      weight = points.get(k).distance(points.get(shortestPaths[k][indexFinal]));
      k= shortestPaths[k][indexFinal];
    }
    return weight;
  }






  public int[][] calculShortestPaths(ArrayList<Point> points, int edgeThreshold) {
    int[][] paths=new int[points.size()][points.size()];
    for (int i=0;i<paths.length;i++) for (int j=0;j<paths.length;j++) paths[i][j]=i;

    double[][] dist=new double[points.size()][points.size()];

    for (int i=0;i<paths.length;i++) {
      for (int j=0;j<paths.length;j++) {
        if (i==j) {dist[i][i]=0; continue;}
        if (points.get(i).distance(points.get(j))<=edgeThreshold) dist[i][j]=points.get(i).distance(points.get(j)); else dist[i][j]=Double.POSITIVE_INFINITY;
        paths[i][j]=j;
      }
    }

    for (int k=0;k<paths.length;k++) {
      for (int i=0;i<paths.length;i++) {
        for (int j=0;j<paths.length;j++) {
          if (dist[i][j]>dist[i][k] + dist[k][j]){
            dist[i][j]=dist[i][k] + dist[k][j];
            paths[i][j]=paths[i][k];

          }
        }
      }
    }

    return paths;
  }

  private boolean contains(ArrayList<Edge> edges,Point p,Point q){
    for (Edge e:edges){
      if (e.p.equals(p) && e.q.equals(q) ||
              e.p.equals(q) && e.q.equals(p) ) return true;
    }
    return false;
  }


  private Tree2D edgesToTree(ArrayList<Edge> edges, Point root) {
    ArrayList<Edge> remainder = new ArrayList<Edge>();
    ArrayList<Point> subTreeRoots = new ArrayList<Point>();
    Edge current;
    int i=0;
    while (edges.size()!=0) {
      i++;
      current = edges.remove(0);
        if (current.p.x==root.x && current.p.y==root.y) {
        subTreeRoots.add(current.q);
      } else {
        if (current.q.x==root.x && current.q.y==root.y) {
          subTreeRoots.add(current.p);
        } else {
          remainder.add(current);
        }
      }
    }

    ArrayList<Tree2D> subTrees = new ArrayList<Tree2D>();
    for (Point subTreeRoot: subTreeRoots) subTrees.add(edgesToTree((ArrayList<Edge>)remainder.clone(),subTreeRoot));

    return new Tree2D(root, subTrees);
  }


  private ArrayList<Edge> sort(ArrayList<Edge> edges) {
    if (edges.size()==1) return edges;

    ArrayList<Edge> left = new ArrayList<Edge>();
    ArrayList<Edge> right = new ArrayList<Edge>();
    int n=edges.size();
    for (int i=0;i<n/2;i++) { left.add(edges.remove(0)); }
    while (edges.size()!=0) { right.add(edges.remove(0)); }
    left = sort(left);
    right = sort(right);

    ArrayList<Edge> result = new ArrayList<Edge>();
    while (left.size()!=0 || right.size()!=0) {
      if (left.size()==0) { result.add(right.remove(0)); continue; }
      if (right.size()==0) { result.add(left.remove(0)); continue; }
      if (left.get(0).distance() < right.get(0).distance()) result.add(left.remove(0));
      else result.add(right.remove(0));
    }
    return result;
  }
}
class Edge {
  protected Point p,q;
  protected double weight;
  protected Edge(Point p,Point q){ this.p=p; this.q=q; }
  protected Edge(Point p,Point q, double weight){ this.p=p; this.q=q; this.weight=weight; }
  protected double distance(){ return p.distance(q); }

  @Override
  public String toString() {
    return "Edge{" +
            "p=" + p +
            ", q=" + q +
            '}';
  }
}
class NameTag {
  private ArrayList<Point> points;
  private int[] tag;
  protected NameTag(ArrayList<Point> points){
    this.points=(ArrayList<Point>)points.clone();
    tag=new int[points.size()];
    for (int i=0;i<points.size();i++) tag[i]=i;
  }
  protected void reTag(int j, int k){
    for (int i=0;i<tag.length;i++) if (tag[i]==j) tag[i]=k;
  }
  protected int tag(Point p){
    for (int i=0;i<points.size();i++) if (p.equals(points.get(i))) return tag[i];
    return 0xBADC0DE;
  }
}
