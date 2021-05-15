package mapgen.algorithms.jtssimplify;

import org.locationtech.jts.geom.*;

/**
 * A {@link LineSegment} which is tagged with its location in a {@link Geometry}.
 * Used to index the segments in a geometry and recover the segment locations
 * from the index.
 */
public class TaggedLineSegment
    extends LineSegment
{
  private final Geometry parent;
  private final int index;

  public TaggedLineSegment(Coordinate p0, Coordinate p1, Geometry parent, int index) {
    super(p0, p1);
    this.parent = parent;
    this.index = index;
  }

  public TaggedLineSegment(Coordinate p0, Coordinate p1) {
    this(p0, p1, null, -1);
  }

  public Geometry getParent() { return parent; }
  public int getIndex() { return index; }
}