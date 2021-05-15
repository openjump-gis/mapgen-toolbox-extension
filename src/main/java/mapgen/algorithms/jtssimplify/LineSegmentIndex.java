package mapgen.algorithms.jtssimplify;

import java.util.*;

import mapgen.algorithms.jts17qtree.ItemVisitor;
import mapgen.algorithms.jts17qtree.Quadtree;

import org.locationtech.jts.geom.*;

/**
 * An index of {@link LineSegment}.
 */
public class LineSegmentIndex
{
  private final Quadtree index = new Quadtree();

  public LineSegmentIndex()
  {
  }

  public void add(TaggedLineString line) {
    TaggedLineSegment[] segs = line.getSegments();
    for (int i = 0; i < segs.length - 1; i++) {
      TaggedLineSegment seg = segs[i];
      add(seg);
    }
  }

  public void add(LineSegment seg)
  {
    index.insert(new Envelope(seg.p0, seg.p1), seg);
  }

  public void remove(LineSegment seg)
  {
    index.remove(new Envelope(seg.p0, seg.p1), seg);
  }

  public List query(LineSegment querySeg)
  {
    Envelope env = new Envelope(querySeg.p0, querySeg.p1);

    LineSegmentVisitor visitor = new LineSegmentVisitor(querySeg);
    index.query(env, visitor);
    List itemsFound = visitor.getItems();

//    List listQueryItems = index.query(env);
//    System.out.println("visitor size = " + itemsFound.size()
//                       + "  query size = " + listQueryItems.size());
//    List itemsFound = index.query(env);

    return itemsFound;
  }
}

/**
 * ItemVisitor subclass to reduce volume of query results.
 */
class LineSegmentVisitor implements ItemVisitor {

  // MD - only seems to make about a 10% difference in overall time.

  private final LineSegment querySeg;
  private List<LineSegment> items = new ArrayList();

  public LineSegmentVisitor(LineSegment querySeg) {
    this.querySeg = querySeg;
  }

  public void visitItem(Object item) {
    LineSegment seg = (LineSegment) item;
    if (Envelope.intersects(seg.p0, seg.p1, querySeg.p0, querySeg.p1))
      items.add(seg);
  }

  public List<LineSegment> getItems() { return items; }
}
