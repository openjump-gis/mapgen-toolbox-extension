package mapgen.measures;

import java.util.ArrayList;
import java.util.List;

import mapgen.geomutilities.PointLineDistance;
import mapgen.geomutilities.TangentAngleFunction;
import mapgen.measures.supportclasses.PointInLineConflict;
import mapgen.measures.supportclasses.PointInLineConflictList;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;

/**
 * Check if points are on line (main criteria is the change of direction)
 * and saves them in a List of type PointInLineConflict
 * <p>
 * created on 		01.12.2005
 *
 * @author sstein
 */
public class PointOnLine {

  private final List<Point> pointList = new ArrayList<>();
  private int nrOfPoints = 0;
  private int testedPoints = 0;
  private final PointInLineConflictList confList = new PointInLineConflictList();
  private boolean isConflict = false;

  /**
   * constructor 1
   *
   * @param myGeometry     a Geometry
   * @param distThreshold  distance threshold
   * @param angleThreshold angle threshold
   */
  public PointOnLine(Geometry myGeometry, double distThreshold, double angleThreshold) {

    if (myGeometry instanceof Polygon) {
      Polygon myPolygon = (Polygon) myGeometry;
      // calc smallest edge for ExteriorRing
      LineString myOuterRing = myPolygon.getExteriorRing();
      this.calcPoints(myOuterRing, distThreshold, angleThreshold, 0);
      // calc smallest edge for InterriorRings
      int nrIntRings = myPolygon.getNumInteriorRing();
      if (nrIntRings > 0) {
        for (int i = 0; i < nrIntRings; i++) {
          LineString myInnerRing = myPolygon.getInteriorRingN(i);
          this.calcPoints(myInnerRing, distThreshold, angleThreshold, i + 1);
        }
      }
    } else {
      // zeugs fuer point oder sowas
    }
  }

  /**
   * constructor 2
   *
   * @param myLine         a LineString
   * @param distThreshold  distance threshold
   * @param angleThreshold angle threshold
   */
  public PointOnLine(LineString myLine, double distThreshold, double angleThreshold) {
    this.calcPoints(myLine, distThreshold, angleThreshold, 0);
  }

  /**
   * calculates the points in a line and saves them in
   * a List <p>
   * the list can be recieved from outside using getPointList()
   *
   * @param myLine         a LineString
   * @param distThreshold  : distance point-edge
   * @param angleThreshold : maximum change of direction of edge
   */
  private void calcPoints(LineString myLine, double distThreshold, double angleThreshold, int noOfRing) {

    double nrPoints = myLine.getNumPoints();
    Coordinate[] coords = myLine.getCoordinates();

    TangentAngleFunction taf = new TangentAngleFunction(myLine.getCoordinates());
    double[] curvValues = taf.getCurv();
    GeometryFactory myGF = new GeometryFactory();
    int count = 0, countChecks = 0;
    double s = distThreshold + 1;
    //double x1, y1, x2, y2, dx, dy;
    //double xt1 = 0, yt1 = 0, xt2 = 0, yt2 = 0;

    //-- starting from second point - thus, start or endpoint is never deleted
    for (int i = 1; i < curvValues.length - 1; i++) {
      countChecks++;
      if (Math.abs(curvValues[i]) < angleThreshold) {
        PointLineDistance pld = new PointLineDistance(coords[i], coords[i - 1], coords[i + 1]);
        s = pld.getDistance();
        // ---- new part : returning a list ----------------
        if (s < distThreshold) {
          count = count + 1;
          PointInLineConflict sec = new PointInLineConflict();
          sec.pointRingIdx = noOfRing;
          sec.pointIdx = i;
          sec.distance = s;
          confList.add(sec);
          //--
          Point pt = myGF.createPoint(coords[i]);
          this.pointList.add((Point) pt.copy());
        }
      }
    } //end for
    int oldValue = this.nrOfPoints; //since we also check rings
    this.setNrOfPointsInLine(oldValue + count);
    this.testedPoints = (this.testedPoints + countChecks);
  }


  /**************** getters and setters ***************/
  public double getNrOfPointsInLine() {
    return this.nrOfPoints;
  }

  private void setNrOfPointsInLine(int nrPoints) {
    this.nrOfPoints = nrPoints;
  }

  /**
   * @return a List of points in a line (the first and last point has not been checked)
   */
  public List<Point> getPointList() {
    return this.pointList;
  }

  /**
   * @return Returns a List of conflicts with elements of type PointInLineConflict.
   */
  public PointInLineConflictList getConflicList() {
    return confList;
  }

  /**
   * @return Returns the hasConflicts.
   */
  public boolean hasConflicts() {
    if (this.nrOfPoints > 0) {
      this.isConflict = true;
    } else {
      this.isConflict = false;
    }
    return this.isConflict;
  }

  public boolean hasAllPointsOnLine() {
    boolean allEdgesTooShort = false;
    if (this.nrOfPoints == this.testedPoints) {
      allEdgesTooShort = true;
    }
    return allEdgesTooShort;
  }

  public int getTestedPoints() {
    return testedPoints;
  }
}
