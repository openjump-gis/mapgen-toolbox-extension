package mapgen.algorithms.jtssimplify;

import java.util.*;

/**
 * Simplifies a collection of TaggedLineStrings, preserving topology
 * (in the sense that no new intersections are introduced).
 */
public class TaggedLinesSimplifier
{
  private final LineSegmentIndex inputIndex = new LineSegmentIndex();
  private final LineSegmentIndex outputIndex = new LineSegmentIndex();
  private double distanceTolerance = 0.0;

  public TaggedLinesSimplifier()
  {

  }

  /**
   * Sets the distance tolerance for the simplification.
   * All vertices in the simplified geometry will be within this
   * distance of the original geometry.
   *
   * @param distanceTolerance the approximation tolerance to use
   */
  public void setDistanceTolerance(double distanceTolerance) {
    this.distanceTolerance = distanceTolerance;
  }

  /**
   * Simplify a collection of {@link TaggedLineString}s
   *
   * @param taggedLines the collection of lines to simplify
   */
  public void simplify(Collection taggedLines) {
    for (Iterator i = taggedLines.iterator(); i.hasNext(); ) {
      inputIndex.add((TaggedLineString) i.next());
    }
    for (Iterator i = taggedLines.iterator(); i.hasNext(); ) {
      TaggedLineStringSimplifier tlss
                    = new TaggedLineStringSimplifier(inputIndex, outputIndex);
      tlss.setDistanceTolerance(distanceTolerance);
      tlss.simplify((TaggedLineString) i.next());
    }
  }

}