package mapgen.measures.supportclasses;

/**
 * Describes a point which lies perceptual on a line of the
 * previous and next point
 *
 * created on 		12.12.2005
 * @author sstein
 */
public class PointInLineConflict implements Cloneable {

  public int pointRingIdx = 0;
  public int pointIdx = 0;
  public double distance = 0; //point to line(previous-next)

  public Object clone() {
    try {
      return super.clone();
    } catch (CloneNotSupportedException e) {
      System.out.println("ShortEdgeConflict: Error in clone() method");
      return null;
    }
  }

}
