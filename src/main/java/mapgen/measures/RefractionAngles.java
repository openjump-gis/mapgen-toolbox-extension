package mapgen.measures;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.swing.JFrame;

import mapgen.geomutilities.JumpAngle;
import mapgen.geomutilities.SecondGeodeticTask2d;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;

/**
 * Calculates the angle among the verticies of line or polygon objects
 * and  - if a threshold (in radians) is given - tests for angles below the
 * threshold
 * <p>
 * created on 		01.10.2004
 *
 * @author sstein
 */
public class RefractionAngles extends JFrame {

  private final double threshold;
  private final List<Point> minVerticesPointList = new ArrayList<>();
  private final List<Double> minAnglesList = new ArrayList<>();
  private final List<Double> angleList = new ArrayList<>();
  private final List<Double> distanceList = new ArrayList<>();
  private int nrOfToSmallAngles = 0;
  private double smallestAngle = Math.PI;
  private boolean hasToSmallAngles = false;

  /**
   * Constructor 0
   * calculates the angles for a Polygon or LineString
   *
   * @param myGeometry input Geometry
   */
  public RefractionAngles(Geometry myGeometry) {
    this.threshold = 0;
    if (myGeometry instanceof Polygon) {
      Polygon myPolygon = (Polygon) myGeometry;
      // calc smallest edge for ExteriorRing
      LineString myOuterRing = myPolygon.getExteriorRing();
      this.calcAngle(myOuterRing);
      // calc smallest edge for InterriorRings
      int nrIntRings = myPolygon.getNumInteriorRing();
      if (nrIntRings > 0) {
        for (int i = 0; i < nrIntRings; i++) {
          LineString myInnerRing = myPolygon.getInteriorRingN(i);
          this.calcAngle(myInnerRing);
        }
      }
    } else if (myGeometry instanceof LineString) {
      LineString myLine = (LineString) myGeometry;
      this.calcAngle(myLine);
    } else {
      // do something for points or coordinates
    }
  }

  /**
   * Constructor 0a
   * calculates the angles for a Polygon or LineString
   * checks if the angles are below a threshold
   *
   * @param myGeometry input Geometry
   * @param threshold  threshold
   */
  public RefractionAngles(Geometry myGeometry, double threshold) {
    this.threshold = threshold;
    if (myGeometry instanceof Polygon) {
      Polygon myPolygon = (Polygon) myGeometry;
      // calc smallest edge for ExteriorRing
      LineString myOuterRing = myPolygon.getExteriorRing();
      this.calcAngle(myOuterRing);
      // calc smallest edge for InterriorRings
      int nrIntRings = myPolygon.getNumInteriorRing();
      if (nrIntRings > 0) {
        for (int i = 0; i < nrIntRings; i++) {
          LineString myInnerRing = myPolygon.getInteriorRingN(i);
          this.calcAngle(myInnerRing);
        }
      }
    } else if (myGeometry instanceof LineString) {
      LineString myLine = (LineString) myGeometry;
      this.calcAngle(myLine);
    } else {
      // do something for points or coordinates
    }
  }

  /**
   * constructor 1a
   * calculates the angles below a threshold for a polygon
   * checking exterior ring and holes
   *
   * @param myPolygon a Polygon
   * @param threshold in Radian (not degree)
   */
  public RefractionAngles(Polygon myPolygon, double threshold) {

    this.threshold = threshold;
    // calc smallest edge for ExteriorRing
    LineString myOuterRing = myPolygon.getExteriorRing();
    this.calcAngle(myOuterRing);
    // calc smallest edge for InterriorRings
    int nrIntRings = myPolygon.getNumInteriorRing();
    if (nrIntRings > 0) {
      for (int i = 0; i < nrIntRings; i++) {
        LineString myInnerRing = myPolygon.getInteriorRingN(i);
        this.calcAngle(myInnerRing);
      }
    }
  }

  /**
   * constructor 1b
   * calculates the angles for a polygon
   * checking exterior ring and holes
   *
   * @param myPolygon a Polygon
   */
  public RefractionAngles(Polygon myPolygon) {

    this.threshold = 0;
    // calc smallest edge for ExteriorRing
    LineString myOuterRing = myPolygon.getExteriorRing();
    this.calcAngle(myOuterRing);
    // calc smallest edge for InterriorRings
    int nrIntRings = myPolygon.getNumInteriorRing();
    if (nrIntRings > 0) {
      for (int i = 0; i < nrIntRings; i++) {
        LineString myInnerRing = myPolygon.getInteriorRingN(i);
        this.calcAngle(myInnerRing);
      }
    }
  }

  /**
   * constructor 2a
   * calculates the angles of a line
   * and tests if they are below a given threshold value
   *
   * @param myLine    a LineString
   * @param threshold in Radian (not degree)
   */
  public RefractionAngles(LineString myLine, double threshold) {
    this.threshold = threshold;
    this.calcAngle(myLine);
  }

  /**
   * constructor 2b
   * calculates the angles of a line
   *
   * @param myLine a LineString
   */
  public RefractionAngles(LineString myLine) {
    this.threshold = 0;
    this.calcAngle(myLine);
  }

  /**
   * gets the angles for a LineString from Jump function
   * and checks against threshold
   *
   * @param myLine a LineString
   */
  private void calcAngle(LineString myLine) {

    //double nrPoints = myLine.getNumPoints();
    Coordinate[] myCoordinates = myLine.getCoordinates();
    GeometryFactory myGF = new GeometryFactory();
    //init minimum length
    double smallestAngle = this.smallestAngle;
    int count = 0;
    double alpha, x0, y0, s1, s2;
    // set for first point angle = 0
    Double temp1 = 0.0;
    this.angleList.add(temp1);
    this.distanceList.add(temp1);
    // calculate angle (and distance) and compare with threshold
    for (int i = 1; i < myCoordinates.length - 1; i++) {
      alpha = JumpAngle.angleBetween(myCoordinates[i],     // tail
          myCoordinates[i - 1],  // tip1
          myCoordinates[i + 1]); // tip2
      Double temp2 = alpha; //type cast
      this.angleList.add(temp2);
      // summed geometric distance between all 3 points
      s1 = SecondGeodeticTask2d.calcDistanceCoord(myCoordinates[i - 1], myCoordinates[i]);
      s2 = SecondGeodeticTask2d.calcDistanceCoord(myCoordinates[i + 1], myCoordinates[i]);
      Double s = s1 + s1; //type cast
      this.distanceList.add(s);
      // angle from algorithm below
      // double angle2 = angle3Points(myCoordinates, i);
      if (alpha < smallestAngle) {
        smallestAngle = alpha;
      }
      if (alpha < this.threshold) {
        this.hasToSmallAngles = true;
        Coordinate myCoo = myCoordinates[i];
        Point myProblemPoint = myGF.createPoint(myCoo);
        this.minVerticesPointList.add((Point) myProblemPoint.copy());
        Double val = alpha;
        this.minAnglesList.add(val);
      }
    }
    // return smallest length and nr of edges
    this.setSmallestAngle(smallestAngle);
    // add for last point angle = 0 or original angle if LineString is closed
    // so the number of angles is equal to number of points
    if (myLine.isClosed()) {
      alpha = JumpAngle.angleBetween(
          myCoordinates[0],            // tail
          myCoordinates[myCoordinates.length - 2],  //-1 = first and last point // tip1
          myCoordinates[1]);            // tip2
      Double temp2 = alpha; //type cast
      this.angleList.add(temp2);
      //change value of first angle from 0 to temp2
      this.angleList.set(0, temp2);
      // summed geometric distance between all 3 points
      s1 = SecondGeodeticTask2d.calcDistanceCoord(myCoordinates[myCoordinates.length - 2], myCoordinates[0]);
      s2 = SecondGeodeticTask2d.calcDistanceCoord(myCoordinates[1], myCoordinates[0]);
      Double s = s1 + s1; //type cast
      this.distanceList.add(s);
      this.distanceList.set(0, s);
    } else { // set to 0
      this.angleList.add(temp1);
      this.distanceList.add(temp1);
    }
  }

  /**
   * calculates angle between 3 points
   * returns angle in radians for the point in the middle
   * <p>
   * /    u * v      \
   * alpha  = acos | --------------  |
   * \ ||u|| * ||v|| /
   * <p>
   * this code is from Agent project .. but not used
   *
   * @param myCoords coordinate collection of the whole Line
   * @param position for which point the angle should be calculated
   * @return angle (in radians)
   */
  private double angle3Points(Coordinate[] myCoords, int position) {
    int i = position;
    double x1 = myCoords[i - 1].x;
    double y1 = myCoords[i - 1].y;
    double x2 = myCoords[i].x;
    double y2 = myCoords[i].y;
    double x3 = myCoords[i + 1].x;
    double y3 = myCoords[i + 1].y;

    double returnvalue;

    // calculate norm(distance) of u,v
    double d1 = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double d2 = Math.sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));

    // if one distance = 0; no angle exists (two identical points)
    if ((d1 == 0.0) || (d2 == 0.0)) {
      returnvalue = 0.0;
    } else {
      double BAx = (x1 - x2) / d1;
      double BAy = (y1 - y2) / d1;
      double BCx = (x3 - x2) / d2;
      double BCy = (y3 - y2) / d2;
      double cosin = (BAx * BCx + BAy * BCy);

      if (cosin >= 1.0) {
        returnvalue = 0.0;
      } else if (cosin <= -1.0) {
        returnvalue = Math.PI;
      } else {
        double vp = BAx * BCy - BAy * BCx;
        if (vp < 0.0) {
          returnvalue = -1 * Math.acos(cosin);
        } else {
          returnvalue = Math.acos(cosin);
        }
      }
    }
    return returnvalue;
  }

  /**
   * the distance for all Points of a Line or Ring
   * if object is a ring, first and last angle have a value
   * otherwise first and last point get zero angle value
   *
   * @return distances from p1 to p2 to p3, where the angle is in between
   */
  public double[] getDistances() {

    double[] dist = new double[this.distanceList.size()];
    int i = 0;
    for (Double d : this.distanceList) {
      dist[i++] = d;
    }
    return dist;
  }

  /**
   * the angles for all Points of a Line or Ring
   * if object is a ring, first and last angle have a value
   * otherwise first and last point get zero angle value
   *
   * @return angles as double array,
   * angle values in radians from 0..pi,
   * the refraction angles are always smaller than pi
   * and have no sign,
   */
  public double[] getAngles() {

    double[] angles = new double[this.angleList.size()];
    int i = 0;
    for (Double a : this.angleList) {
      angles[i++] = a;
    }
    return angles;
  }

  public double getSmallestAngle() {
    return smallestAngle;
  }

  private void setSmallestAngle(double smallestAngle) {
    this.smallestAngle = smallestAngle;
  }

  /**
   * this is only possible if a threshold was set
   *
   * @return double array of angles from 0..pi,
   * attention: the refraction angles are always smaller
   * than pi and have no sign,
   */
  public double[] getMinAngles() {

    double[] minAngles = new double[this.minAnglesList.size()];
    int i = 0;
    for (Double a : this.minAnglesList) {
      minAngles[i++] = a;
    }
    return minAngles;
  }

  public List<Point> getMinVerticesPointList() {
    return minVerticesPointList;
  }

  /**
   * this is only possible if a threshold was set
   */
  public int getNrOfToSmallAngles() {
    this.nrOfToSmallAngles = this.minAnglesList.size();
    return nrOfToSmallAngles;
  }

  public double getThreshold() {
    return threshold;
  }

}
