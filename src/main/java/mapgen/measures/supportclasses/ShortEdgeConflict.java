package mapgen.measures.supportclasses;

/**
 * Describes a minimal length conflict of an edge
 *
 * created on 		07.07.2005
 * @author sstein
 */
public class ShortEdgeConflict implements Cloneable {

  public int edgeRingIdx = 0;
  public int edgeLineIdx = 0;
  public int edgeStartPtIdx = 0;
  public int edgeEndPtIdx = 0;
  public double length = 0;

  public Object clone() {
    try {
      return super.clone();
    } catch (CloneNotSupportedException e) {
      System.out.println("ShortEdgeConflict: Error in clone() method");
      return null;
    }
  }

}
