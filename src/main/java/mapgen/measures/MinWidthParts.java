package mapgen.measures;

import java.util.List;
import java.util.ArrayList;

import mapgen.geomutilities.PointLineDistance;
import mapgen.measures.supportclasses.MWConflictList;
import mapgen.measures.supportclasses.MinWidthConflict;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;


/**
 * Calculates the minimal distance between two non adjacent edges<p>
 * of a polygon shape / LineString <p>
 * calculation among point and edge<p>
 * (algorithm by Mats Bader, AGENT Project)<p>
 * preliminary condition: <p>
 * a) no self intersection of polygon / LineString<p>
 * b) polygon / LineString needs more than four vertices<p>
 * <p>
 * zero distances will not be detected (adjacent edges)
 * <p>
 * to get the conflicts use getMwcList()
 * <p>
 * created on 		29.09.2004
 * last modified: 	20.12.2004 extended by save conflicts as MWConflictList
 *
 * @author sstein
 */
public class MinWidthParts {

  private double threshold = 0;
  private double nrFoundVerticies = 0;
  private double nrOfChecks = 0;
  //    private ArrayList minWidthDoubleList = new ArrayList();
  private final List<LineString> dispVecPointEdgeLStringList = new ArrayList<>();
  private final List<LineString> dispVecPointPointLStringList = new ArrayList<>();
  private final List<LineString> minEdgeLineStringList = new ArrayList<>();
  private final List<Point> minVertexPointList = new ArrayList<>();
  private boolean outlineConflicts = false;
  private boolean hasHoles = false;
  private boolean[][] conflictMatrix;
  private boolean hasConflicts = false;
  private final MWConflictList mwclist = new MWConflictList();
  private double minWidth = 999999;

  /**
   * constructor
   *
   * @param myObject  Polygon or LineString
   * @param threshold threshold
   */
  public MinWidthParts(Geometry myObject, double threshold) {

    this.setThreshold(threshold);

    if (myObject instanceof Polygon) {
      Polygon myPolygon = (Polygon) myObject;
      //condition: more than five points = more than 4 edges
      if (myPolygon.getNumPoints() > 4) {
        this.calcMinWidth(myPolygon, threshold);
      }
    } else if (myObject instanceof LineString) {
      LineString myLine = (LineString) myObject;
      //condition: more than five points = more than 4 edges
      if (myLine.getNumPoints() > 4) {
        this.calcMinWidth(myLine, threshold);
      }
    } else {
      System.out.println("MinWidthParts: Input Object no Polygon or LineString");
    }
  }

  /**
   * @param myPolygon   a Polygon
   * @param myThreshold threshold
   */
  private void calcMinWidth(Polygon myPolygon, double myThreshold) {
    // codition b: more than 4 vertices is true
    /*****************************
     * problem: inner and outer rings
     * getCoordinates - first and last point are the same
     * for outer and inner rings
     * first outer ring coordinates, then inner ring:
     * example: outerring: point(0) = point(9)
     * 		   innerring1: point(10)= point(14)
     * 		   innerringx:
     ***************/
    int nrOfInnerRings = myPolygon.getNumInteriorRing();
    if (nrOfInnerRings > 0) {
      this.hasHoles = true;
      this.conflictMatrix = new boolean[nrOfInnerRings + 1][nrOfInnerRings + 1];
    } else {
      this.conflictMatrix = new boolean[1][1];
    }
    LineString outerRing = myPolygon.getExteriorRing();
    // test outer ring with it self
    boolean retval = this.selfTest(outerRing, 0);
    this.conflictMatrix[0][0] = retval;
    if (retval) {
      this.outlineConflicts = true;
    }
    // --- if inner rings exist
    if (this.hasHoles) {
      // test innerRings with itself
      for (int i = 0; i < nrOfInnerRings; i++) {
        LineString innerRingA = myPolygon.getInteriorRingN(i);
        retval = this.selfTest(innerRingA, i + 1);
        this.conflictMatrix[i + 1][i + 1] = retval;
      }
      // test outerRing with innerRings
      for (int i = 0; i < nrOfInnerRings; i++) {
        LineString innerRing = myPolygon.getInteriorRingN(i);
        retval = this.testWithOtherRings(outerRing, innerRing, 0, i + 1);
        this.conflictMatrix[0][i + 1] = retval;
      }
      // test innerRings with outerRings
      for (int i = 0; i < nrOfInnerRings; i++) {
        LineString innerRing = myPolygon.getInteriorRingN(i);
        retval = this.testWithOtherRings(innerRing, outerRing, i + 1, 0);
        this.conflictMatrix[i + 1][0] = retval;
      }

      // if more than one inner ring
      // test all inner rings against each another
      if (nrOfInnerRings > 1) {
        // -1 otherwise = selftest for last ring
        // produces error for j=i+1
        for (int i = 0; i < nrOfInnerRings; i++) {
          LineString innerRingA = myPolygon.getInteriorRingN(i);
          // it may not j=i+1 since every ring is testing its own points
          // against the other edges
          for (int j = 0; j < nrOfInnerRings; j++) {
            if (j != i) { //avoid self test
              LineString innerRingB = myPolygon.getInteriorRingN(j);
              retval = this.testWithOtherRings(innerRingA, innerRingB, i + 1, j + 1);
              this.conflictMatrix[i + 1][j + 1] = retval;
            }
          }
        }
      } //end if (more than one inner ring)

    } //end for (innerRing exist)

  } //end function

  private void calcMinWidth(LineString myLine, double myThreshold) {
    // codition b: more than 4 vertices is true
    this.conflictMatrix = new boolean[1][1];
    // test LineString with it self
    boolean retval = this.selfTest(myLine, 0);
    this.conflictMatrix[0][0] = retval;
    if (retval) {
      this.outlineConflicts = true;
    }

  } //end function

  /**
   * Test distance of points from one Linestring to edges from self LineString
   * but not for the nearest 2 edges
   *
   * @param Ring a ring
   */
  private boolean selfTest(LineString Ring, int index) {

    boolean conflict = false;
    // the ring has to have more than 5 points (= at least 5 edges)
    if (Ring.getNumPoints() > 5) {

      GeometryFactory myGF = new GeometryFactory();
      Coordinate[] edge = new Coordinate[2];
//	        GeometryLocation[] ClosestVector = null;
      LineString tempLine;
      double dist;

      // ==== loop for over all points from Ring =====
      // -1; otherwise last point will be tested twice
      for (int i = 0; i < Ring.getNumPoints() - 1; i++) {
        Point myPoint = Ring.getPointN(i);
        // -1; since edge(j,j+1)
        for (int j = 0; j < Ring.getNumPoints() - 1; j++) {
          //reduce calculations concerning adjacent edges
          if ( // don't check the next 2 edges
              (Math.abs(j - i) >= 2) &&
                  // don't check the previous two edges
                  (Math.abs(j + 1 - i) >= 2) &&
                  // if base point i = 0, don't check last two edges
                  ((i != 0) || (j < (Ring.getNumPoints() - 3))) && //if both = false => false
                  // if base point i = 1, don't check last edge
                  ((i != 1) || (j < (Ring.getNumPoints() - 2))) && //if both = false => false
                  // if point i = last-1 is base, don't check the first edge
                  ((i != Ring.getNumPoints() - 2) || (j > 0))
          ) {
            //--- new since faster
            Coordinate A = Ring.getCoordinateN(j);
            Coordinate B = Ring.getCoordinateN(j + 1);
            Coordinate P = myPoint.getCoordinate();
            PointLineDistance pld = new PointLineDistance(P, A, B);
            dist = pld.getDistance();
            boolean isPerpendicularPoint = pld.isPointOnLine();
            Coordinate rp = pld.getClosestPoint().getCoordinate();
            //--- old
            edge[0] = Ring.getCoordinateN(j);
            edge[1] = Ring.getCoordinateN(j + 1);
            tempLine = myGF.createLineString(edge);
//	                    DistanceOp distanceOp = new DistanceOp(myPoint, tempLine);
//	                    dist = distanceOp.distance();
//	                    ClosestVector = distanceOp.closestLocations(); 
//	                    // test if the point on edge is a node
//	                    boolean isPerpendicularPoint = true;
//	                    Coordinate p1 = ClosestVector[1].getCoordinate();
//	                    Coordinate pA = tempLine.getStartPoint().getCoordinate();
//	                    Coordinate pB = tempLine.getEndPoint().getCoordinate();
//	                    if( p1.equals2D(pA) || p1.equals2D(pB)){ 
//	                        isPerpendicularPoint = false; 
//	                        }
            this.nrOfChecks = this.nrOfChecks + 1;
            if ((dist != 0) &&
                (dist < this.getThreshold())
            ) {
              conflict = true;
              //========== calc disp vector and save as linestring =======
              double dxAct, dyAct, dxRef, dyRef, dxMove, dyMove, xNew, yNew;
//		                    dxAct = ClosestVector[0].getCoordinate().x - ClosestVector[1].getCoordinate().x;
//		                    dyAct = ClosestVector[0].getCoordinate().y - ClosestVector[1].getCoordinate().y;
              dxAct = P.x - rp.x;
              dyAct = P.y - rp.y;
              dxRef = dxAct * this.threshold / dist;
              dyRef = dyAct * this.threshold / dist;
              dxMove = dxRef - dxAct;
              dyMove = dyRef - dyAct;
              xNew = myPoint.getX() + dxMove;
              yNew = myPoint.getY() + dyMove;

              Coordinate[] dispVectorCoord = new Coordinate[2];

              dispVectorCoord[0] = myPoint.getCoordinate();
              dispVectorCoord[1] = new Coordinate(xNew, yNew);
              LineString dispVector = myGF.createLineString(dispVectorCoord);
              //======= save ======
              //Double Ddist = dist;
              //this.minWidthDoubleList.add(Ddist);
              // if not using object.clone the list links point on the
              // the last checked point and checked edge
              this.minEdgeLineStringList.add((LineString) tempLine.copy());
              this.minVertexPointList.add((Point) myPoint.copy());
              // ===== save to conflict list ======/
              MinWidthConflict mwc = new MinWidthConflict();
              mwc.ptHoleIdx = index;
              mwc.ptIdx = i;
              mwc.edgeHoleIdx = index;
              mwc.edgeStartPtIdx = j;
              mwc.edgeEndPtIdx = j + 1;
              mwc.distance = dist;
              mwc.ptDispDx = dxMove;
              mwc.ptDispDy = dyMove;
              mwc.ptEdgeConflict = isPerpendicularPoint;
              mwclist.add(mwc);
              //=========
              //--- save for minimal distance
              if (dist < this.minWidth) {
                this.minWidth = dist;
              }
              if (isPerpendicularPoint) {
                this.dispVecPointEdgeLStringList.add((LineString) dispVector.copy());
              } else {
                this.dispVecPointPointLStringList.add((LineString) dispVector.copy());
              }
            } // end if dist < threshold

          }//end if
        }  // end for edges
      } // end for test point
    } //end if
    return conflict;
  }

  /**
   * Test distance of points from one Linestring to edges of another LineString
   *
   * @param baseRing base LineString
   * @param testRing test ring LineString
   */
  public boolean testWithOtherRings(LineString baseRing, LineString testRing, int baseRingIndex, int testRingIndex) {
    GeometryFactory myGF = new GeometryFactory();
    LineString tempLine;
    Coordinate[] edge = new Coordinate[2];
    double dist;
    boolean conflict = false;

    // ==== loop for over all points from Ring =====
    // -1; otherwise last point will be tested twice
    for (int i = 0; i < baseRing.getNumPoints() - 1; i++) {
      Point myPoint = baseRing.getPointN(i);
      // -1; since edge(j,j+1)
      for (int j = 0; j < testRing.getNumPoints() - 1; j++) {
        //reduce calculations concerning adjacent edges
        //--- old
        edge[0] = testRing.getCoordinateN(j);
        edge[1] = testRing.getCoordinateN(j + 1);
        tempLine = myGF.createLineString(edge);
        //--- new since faster
        Coordinate A = testRing.getCoordinateN(j);
        Coordinate B = testRing.getCoordinateN(j + 1);
        Coordinate P = myPoint.getCoordinate();
        PointLineDistance pld = new PointLineDistance(P, A, B);
        dist = pld.getDistance();
        boolean isPerpendicularPoint = pld.isPointOnLine();
        Coordinate rp = pld.getClosestPoint().getCoordinate();
        this.nrOfChecks = this.nrOfChecks + 1;
        if ((dist != 0) &&
            (dist < this.getThreshold())
        ) {
          conflict = true;
          //========== calc disp vector and save as linestring =======
          double dxAct, dyAct, dxRef, dyRef, dxMove, dyMove, xNew, yNew;

          dxAct = P.x - rp.x;
          dyAct = P.y - rp.y;
          dxRef = dxAct * this.threshold / dist;
          dyRef = dyAct * this.threshold / dist;
          dxMove = dxRef - dxAct;
          dyMove = dyRef - dyAct;
          xNew = myPoint.getX() + dxMove;
          yNew = myPoint.getY() + dyMove;

          Coordinate[] dispVectorCoord = new Coordinate[2];

          dispVectorCoord[0] = myPoint.getCoordinate();
          dispVectorCoord[1] = new Coordinate(xNew, yNew);
          LineString dispVector = myGF.createLineString(dispVectorCoord);
          //======== save ===========
          Double Ddist = dist;
          //this.minWidthDoubleList.add(Ddist);
          // if nor using object.clone the list links point on the
          // the last checked point and checked edge
          this.minEdgeLineStringList.add((LineString) tempLine.copy());
          this.minVertexPointList.add((Point) myPoint.copy());
          // ===== save to conflict list ======/
          MinWidthConflict mwc = new MinWidthConflict();
          mwc.ptHoleIdx = baseRingIndex;
          mwc.ptIdx = i;
          mwc.edgeHoleIdx = testRingIndex;
          mwc.edgeStartPtIdx = j;
          mwc.edgeEndPtIdx = j + 1;
          mwc.distance = dist;
          mwc.ptDispDx = dxMove;
          mwc.ptDispDy = dyMove;
          mwc.ptEdgeConflict = isPerpendicularPoint;
          mwclist.add(mwc);
          //=========
          //--- save for minimal distance
          if (dist < this.minWidth) {
            this.minWidth = dist;
          }
          if (isPerpendicularPoint) {
            this.dispVecPointEdgeLStringList.add((LineString) dispVector.copy());
          } else {
            this.dispVecPointPointLStringList.add((LineString) dispVector.copy());
          }
        } // end if
      }  // end for edges
    } // end for test point
    return conflict;
  }

  /**
   * @return the number of all found conflicts:
   * point <=> edge & point <=> node
   * calculatet from minVertexPointList
   */
  public double getNrFoundVerticies() {
    this.nrFoundVerticies = this.minVertexPointList.size();
    return nrFoundVerticies;
  }

  /**
   * @return the number of found conflict:
   * point <=> edge
   * calculatet from minEdgeLineStringList
   */
  public double getNrFoundVertexEdgeConflicts() {
    return this.dispVecPointEdgeLStringList.size();
  }

  /**
   * @return the number of found conflict:
   * point <=> node
   * calculatet from minVertexPointList
   */
  public double getNrFoundVertexPointConflicts() {
    return this.dispVecPointPointLStringList.size();
  }

  public double getThreshold() {
    return threshold;
  }

  private void setThreshold(double threshold) {
    this.threshold = threshold;
  }

  /**
   * @return list with edges as LineStrings which are to close to
   * points obtained from getMinVertexPointList
   */
  public List<LineString> getMinEdgeLineStringList() {
    return minEdgeLineStringList;
  }

  /**
   * @return list with verticies (saved as Point objects) which are
   * to close to edges obtained from getMinEdgeLineStringList
   */
  public List<Point> getMinVertexPointList() {
    return minVertexPointList;
  }

  /**
   * @return list with displacement vectors among
   * point and edge node saved as LineString
   */
  public List<LineString> getDispVecPointPointLStringList() {
    return dispVecPointPointLStringList;
  }

  /**
   * @return list with displacement vectors among
   * point and perpendicular point on edge
   * saved as LineString
   */
  public List<LineString> getDispVecPointEdgeLStringList() {
    return dispVecPointEdgeLStringList;
  }

  public boolean hasHoles() {
    return this.hasHoles;
  }

  public boolean hasOutlineConflicts() {
    return this.outlineConflicts;
  }


  /**
   * returns the number of checks made
   * and is equal to max number of to small distances
   *
   * @return double/int value
   */
  public double getNrOfChecks() {
    return this.nrOfChecks;
  }

  /**
   * @return a matrix which says which elements (exterior ring
   * and holes) have conflicts with each other.
   * (index[0][0] is the polygon outline tested with
   * it self)
   */
  public boolean[][] getConflictMatrix() {
    return this.conflictMatrix;
  }

  /**
   * @return true if polygon (including holes) has conflict
   */
  public boolean hasConflicts() {
    for (int i = 0; i < this.conflictMatrix.length; i++) {
      for (int j = 0; j < this.conflictMatrix.length; j++) {
        if (this.conflictMatrix[i][j]) {
          this.hasConflicts = true;
        }
      }
    }
    return hasConflicts;
  }

  /**
   * @return a special List type of the conflicts as
   * MinWidthConflict type
   */
  public MWConflictList getMwclist() {
    return mwclist;
  }

  /**
   * @return the smallest distance found
   */
  public double getMinWidth() {
    return minWidth;
  }
}