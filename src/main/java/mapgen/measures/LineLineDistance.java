package mapgen.measures;


import java.util.ArrayList;
import java.util.List;

import mapgen.geomutilities.PointLineDistance;
import mapgen.geomutilities.SecondGeodeticTask2d;
import mapgen.measures.supportclasses.MWConflictList;
import mapgen.measures.supportclasses.MinWidthConflict;

import org.jmat.MatlabSyntax;
import org.jmat.data.Matrix;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.IntersectionMatrix;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;

/**
 * checks if two lines<p>
 * - intersects<p>
 * - have a minimum distance,<p>
 * and (if under a given threshold) use calcDisplacementVectors();
 * to calculates the displacement vectors for the conflict points
 * (of outer ring) or centroid - which  depends on type of conflict,<p>
 * <p>
 * If dist[m] < field: "crucialDistance", then it is complicate to give a
 * displacement direction.<p>
 * Information if line A could be part of a line network is give
 * via: hasIdentityFirstPtLineA() or hasIdentityLastPtLineA().
 * <p>
 * to get the conflicts use getMwcList()
 * <p>
 * <p>
 * created on 		08.12.2004
 * last modified:
 * 20.12.2004
 * 18.08.2005 modification for dx,dy matrix store on pt-pt conlficts
 *
 * @author sstein
 */
public class LineLineDistance {

  //public List<LineString> geometryList = new ArrayList();
  private final LineString linA;
  private final LineString linB;
  private final Geometry bufferLineA;
  private final Geometry bufferLineB;
  //-- if objects not intersect
//  private ArrayList minWidthDoubleList = new ArrayList();
  private final List<LineString> dispVecPtEgLSthisPolyList = new ArrayList<>();
  private final List<LineString> dispVecPtPtLSthisPolyList = new ArrayList<>();
  private final List<LineString> dispVecPtEgLSOtherPolyList = new ArrayList<>();
  private final List<LineString> dispVecPtPtLSOtherPolyList = new ArrayList<>();
  private Matrix dxConflictMatrix;
  private Matrix dyConflictMatrix;
  //private List minEdgeLineStringList = new ArrayList();
  //private List minVertexPointList = new ArrayList();
  private double shortestDistance = 999999.99; //init value 999km
  // ---
  private final MWConflictList mwclist = new MWConflictList();
  private boolean hasConflict = false;
  private boolean linesCross = false;
  private boolean areToClose = false;
  private final double minDist;
  private final double signatureA;
  private final double signatureB;
  private final double threshold;
  private boolean identityFirstPtLineA;
  private boolean identityLastPtLineA;
  //-- if dist[m] < field: "crucialDistance" it is complicate to give a
  //   displacement direction and has influence on field identityFirstPtLineA
  //   and on identityFirstPtLineA
  private double crucialDistance = 0.1;

  /**
   * tests if two lines have a distance conflict
   * - check for intersection
   * - checks if the minimum buffer is joint/touched
   * to calculate the dsiplacement use calcDisplacementVectors();
   *
   * @param lineToTest          the LineString to test
   * @param otherLine           anoter LineString
   * @param minDist             distance in metres to seperate objects
   * @param lineASignatureWidth radius of line signatur [in metres]
   * @param lineBSignatureWidth radius of line signatur[in metres]
   */
  public LineLineDistance(LineString lineToTest, LineString otherLine,
                          double minDist, double lineASignatureWidth, double lineBSignatureWidth) {
    this.minDist = minDist;
    this.signatureA = lineASignatureWidth;
    this.signatureB = lineBSignatureWidth;
    this.linA = lineToTest;
    this.linB = otherLine;
    this.threshold = this.minDist + this.signatureA + this.signatureB;
    this.identityFirstPtLineA = false;
    this.identityLastPtLineA = false;

    //--IntersectionMatrix
    IntersectionMatrix myIM = this.linA.relate(this.linB);
    /**
     System.out.println("IntersectionMat: top right-bottom left : "
     + myIM.get(0,0) +  " " +  myIM.get(0,1) +  " " + myIM.get(0,2) +  ","
     + myIM.get(1,0) +  " " +  myIM.get(1,1) +  " " + myIM.get(1,2) +  ","
     + myIM.get(2,0) +  " " +  myIM.get(2,1) +  " " + myIM.get(2,2) );
     **/
    //-- test if Lines (without signature cross/intersect (JTS function)
    //   this should not happen!
    // but exclude case there one point is on one edge => I1 + E2 =-1 && E1+ I2 = +1
    //if ( (myIM.matches("FF*FF****") == false) && (myIM.matches("F***0****") == false)){
    if (!myIM.matches("FF*FF****")) {
      this.hasConflict = true;
      this.linesCross = true;
      System.out.println("LineLineDistance: lines do Cross");
    }
    //else{
    //LineA has buffer of size = signatureA + minDistance
    if (this.signatureA > 0) {
      this.bufferLineA = this.linA.buffer(this.minDist + this.signatureA);
    } else {
      this.bufferLineA = this.linA.buffer(this.minDist);
    }
    if (this.signatureB > 0) {
      this.bufferLineB = this.linB.buffer(this.signatureB);
    } else {
      this.bufferLineB = this.linB;
    }
    if (!bufferLineA.disjoint(this.bufferLineB)) {
      // false disjoint = intersects
      this.hasConflict = true;
      this.areToClose = true;
    }
    //}
  }

  /**
   * calculate the displacement vectors for the specific
   * conflict case
   * a) overlapping geometries: dispVectorOnCentroid for
   * intersecting/overlapping geometries for application on
   * the centroid of p1.
   * b) line is within the minimum distance: Displacement
   * vectors on the specific vertex, which are to narrow.
   * given back via dispVecPointEdgeLStringList and
   * dispVecPointPointLStringList and further a mean disp. vector
   * is calculated for the centroid of p1 (given back via
   * dispVectorOnCentroid).
   */
  public void calcDisplacementVectors() {

    //-- init disp matrix for snakes displacement
    //   should be set even if no conflicts appear
    //   to avaoid error messages
    int nrPtBaseRing = linA.getNumPoints();
    int nrPtTestRing = linB.getNumPoints();
    this.dxConflictMatrix = MatlabSyntax.zeros(nrPtBaseRing, nrPtTestRing);
    this.dyConflictMatrix = MatlabSyntax.zeros(nrPtBaseRing, nrPtTestRing);

    if (this.areToClose) {
      //-- if polygons are disjoint but within the buffer (treshold)
      // test cross wise (point to edge)

      this.testLines(this.linA, this.linB, 1);
      this.testLines(this.linB, this.linA, 2);
    }

  }

  /**
   * If polygons are disjoint:
   * Test distance of points from one Linestring to edges of another LineString
   *
   * @param baseRing the base Ring
   * @param testRing the test Ring
   * @param originalPolygon of them is the polygon,
   *                 need to change direction of displacement vector
   */
  private void testLines(LineString baseRing,
                         LineString testRing,
                         int originalPolygon) {

    // ***********  if rings are seperated **************/
    GeometryFactory myGF = new GeometryFactory();
    //       Coordinate[] edge = new Coordinate[2];

    //LineString tempLine = null;
    double dist;
    // ==== loop for over all points =====
    for (int i = 0; i < baseRing.getNumPoints(); i++) {
      Point myPoint = baseRing.getPointN(i);
      // -1; since edge(j,j+1)
      for (int j = 0; j < testRing.getNumPoints() - 1; j++) {
        //reduce calculations concerning adjacent edges
        //---- new 14.12.2004 ---------
        Coordinate A = testRing.getCoordinateN(j);
        Coordinate B = testRing.getCoordinateN(j + 1);
        Coordinate P = myPoint.getCoordinate();
        PointLineDistance pld = new PointLineDistance(P, A, B);
        dist = pld.getDistance();
        boolean isPerpendicularPoint = pld.isPointOnLine();
        Coordinate rp = pld.getClosestPoint().getCoordinate();
//	                    LineString ClosestVector = myGF.createLineString(edge);
//	                    //---------- old ------------------------
//	                    tempLine = myGF.createLineString(edge);
//	                    DistanceOp distanceOp = new DistanceOp(myPoint, tempLine);
//	                    dist = distanceOp.distance();                    
//	                    ClosestVector = distanceOp.closestLocations();
//	                    // save the shortestDistance
//	                    if (dist < this.shortestDistance){this.shortestDistance=dist;}
//	                    // test if the point on edge is a node
//	                    isPerpendicularPoint = true;
//	                    Coordinate p1 = ClosestVector[1].getCoordinate();
//	                    Coordinate pA = tempLine.getStartPoint().getCoordinate();
//	                    Coordinate pB = tempLine.getEndPoint().getCoordinate();
//	                    if( p1.equals2D(pA) || p1.equals2D(pB)){ 
//	                        isPerpendicularPoint = false; 
//	                        } 
        //-- calc displacementVector
        if (dist < this.threshold) {
          //========== calc disp vector and save as linestring =======
          double xNew = 0, yNew = 0, dxMove = 0, dyMove = 0;
          // dist > 10cm .. since numerical problems lead to distance like 3.1*10^-14
          if (dist > this.crucialDistance) {
            double dxAct, dyAct, dxRef, dyRef;
            //---- old ---
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
          } else {
            //-- if dist = 0, then dont calc disp vector
            // for polygons the direction towards poly centroid was used
            if ((originalPolygon == 1) && (i == 0)) {
              this.identityFirstPtLineA = true;
              //System.out.println("LineLineDistance: first point of Line A has identity to point of other Line");
            } else if ((originalPolygon == 1) && (i == baseRing.getNumPoints() - 1)) {
              this.identityLastPtLineA = true;
              //System.out.println("LineLineDistance: last point of Line A has identity to point of other Line");
            } else {
              System.out.println("LineLineDistance: point distance is smaller " +
                  "than: " + this.crucialDistance + "[m] => no displacement " +
                  "direction can be calculated");
            }
          }
          Coordinate[] dispVectorCoord = new Coordinate[2];
          dispVectorCoord[0] = myPoint.getCoordinate();
          dispVectorCoord[1] = new Coordinate(xNew, yNew);
          LineString dispVector = myGF.createLineString(dispVectorCoord);
          //======== save ===========
          //Double Ddist = dist;
          //this.minWidthDoubleList.add(Ddist);
          // displacement for point into right direction
          dxMove = -1 * dxMove;
          dyMove = -1 * dyMove;
          // if not using object.clone the list links point on the
          // the last checked point and checked edge
          //this.minEdgeLineStringList.add(tempLine.clone());
          //this.minVertexPointList.add(myPoint.clone());
          // ===== save to conflict list ======/
          MinWidthConflict mwc = new MinWidthConflict();
          if (originalPolygon == 1) {
            mwc.ptHoleIdx = 0;
            mwc.edgeHoleIdx = 1;
          } else {
            mwc.ptHoleIdx = 1;
            mwc.edgeHoleIdx = 0;
          }
          mwc.ptIdx = i;
          mwc.edgeStartPtIdx = j;
          mwc.edgeEndPtIdx = j + 1;
          mwc.distance = dist;
          mwc.ptDispDx = dxMove;
          mwc.ptDispDy = dyMove;
          mwc.ptEdgeConflict = isPerpendicularPoint;
          mwclist.add(mwc);
          //====================
          if (isPerpendicularPoint) { //pt-edge conflict
            if (originalPolygon == 1) {
              // conflict detected by base line
              this.dispVecPtEgLSthisPolyList.add((LineString) dispVector.copy());
              dxConflictMatrix.set(i, j, dxMove);
              dyConflictMatrix.set(i, j, dyMove);
              //[sstein] added saving on 18.08.2005
              // to ensure parallel displacement
              dxConflictMatrix.set(i, j + 1, dxMove);
              dyConflictMatrix.set(i, j + 1, dyMove);
            } else {
              // displacement for point into right direction
              // turned back since detected by other line
              dxMove = -1 * dxMove;
              dyMove = -1 * dyMove;
              // if conflict was detected by other line
              // the displacement has to be applied to the edge
              // and not to a single point
              this.dispVecPtEgLSOtherPolyList.add((LineString)dispVector.copy());
              // look if conflict is already saved
              double oldValuedx1 = dxConflictMatrix.get(j, i);
              double oldValuedy1 = dyConflictMatrix.get(j, i);
              double oldValuedx2 = dxConflictMatrix.get(j + 1, i);
              double oldValuedy2 = dyConflictMatrix.get(j + 1, i);
              //- save for first edge point
              if ((oldValuedx1 != 0) || (oldValuedy1 != 0)) {
                double dx = (dxMove + oldValuedx1) / 2.0;
                double dy = (dyMove + oldValuedy1) / 2.0;
                dxConflictMatrix.set(j, i, dx);
                dyConflictMatrix.set(j, i, dy);
              } else {// no previous values saved on this position
                dxConflictMatrix.set(j, i, dxMove);
                dyConflictMatrix.set(j, i, dyMove);
              }
              //- save for second edge point
              if ((oldValuedx2 != 0) || (oldValuedy2 != 0)) {
                double dx = (dxMove + oldValuedx2) / 2.0;
                double dy = (dyMove + oldValuedy2) / 2.0;
                dxConflictMatrix.set(j + 1, i, dx);
                dyConflictMatrix.set(j + 1, i, dy);
              } else {// no previous values saved on this position
                dxConflictMatrix.set(j + 1, i, dxMove);
                dyConflictMatrix.set(j + 1, i, dyMove);
              }
            }
          } else {//pt-pt conflict
            if (originalPolygon == 1) {
              this.dispVecPtPtLSthisPolyList.add((LineString)dispVector.copy());
              //[sstein] saving changed on 18.08.2005
              double distA = SecondGeodeticTask2d.calcDistanceCoord(A, P);
              double distB = SecondGeodeticTask2d.calcDistanceCoord(B, P);
              if (distA < distB) {
                //save on first edge point index=j
                dxConflictMatrix.set(i, j, dxMove);
                dyConflictMatrix.set(i, j, dyMove);
              } else {//distA > distB
                //save on second edge point index=j+1
                dxConflictMatrix.set(i, j + 1, dxMove);
                dyConflictMatrix.set(i, j + 1, dyMove);
              }
            } else {
              this.dispVecPtPtLSOtherPolyList.add((LineString)dispVector.copy());
              // displacement for point into right direction
              // turned back since detected by other line
              dxMove = -1 * dxMove;
              dyMove = -1 * dyMove;
              //[sstein] 18.08.2005 changed :
              //The values can be directly saved to the closest point
              //since conflict exists only with one point if the config is not perpendicular?
              //Therefore find the point index of A oder B which is the closest and save.
              double distA = SecondGeodeticTask2d.calcDistanceCoord(A, P);
              double distB = SecondGeodeticTask2d.calcDistanceCoord(B, P);
              if (distA < distB) {
                //save on first edge point index=j
                double oldValuedx1 = dxConflictMatrix.get(j, i);
                double oldValuedy1 = dyConflictMatrix.get(j, i);
                if ((oldValuedx1 != 0) || (oldValuedy1 != 0)) {
                  double dx = (dxMove + oldValuedx1) / 2.0;
                  double dy = (dyMove + oldValuedy1) / 2.0;
                  dxConflictMatrix.set(j, i, dx);
                  dyConflictMatrix.set(j, i, dy);
                } else {// no previous values saved on this position
                  dxConflictMatrix.set(j, i, dxMove);
                  dyConflictMatrix.set(j, i, dyMove);
                }
              } else {//distA > distB
                //save on second edge point index=j+1
                double oldValuedx2 = dxConflictMatrix.get(j + 1, i);
                double oldValuedy2 = dyConflictMatrix.get(j + 1, i);
                if ((oldValuedx2 != 0) || (oldValuedy2 != 0)) {
                  double dx = (dxMove + oldValuedx2) / 2.0;
                  double dy = (dyMove + oldValuedy2) / 2.0;
                  dxConflictMatrix.set(j + 1, i, dx);
                  dyConflictMatrix.set(j + 1, i, dy);
                } else {// no previous values saved on this position
                  dxConflictMatrix.set(j + 1, i, dxMove);
                  dyConflictMatrix.set(j + 1, i, dyMove);
                }
              } //if distA > distB
            }//if original Polygon
          } //pt-pt conflict
        } // end if (dist < this.threshold)
      }  // end for edges
    } // end for test point
  }

  /******************** getters and setter **********************/

  /**
   * @return list with displacement vectors among
   * point and edge node for Base-Polygon
   * saved as LineString
   */
  public List<LineString> getDispVecListPtPtLSThisLine() {
    return dispVecPtPtLSthisPolyList;
  }

  /**
   * @return list with displacement vectors among
   * point and perpendicular point on edge
   * for Base-Polygon
   * saved as LineString
   */
  public List<LineString> getDispVecListPtEgLSThisLine() {
    return dispVecPtEgLSthisPolyList;
  }

  /**
   * @return list with displacement vectors among
   * point and edge node for Line
   * saved as LineString
   */
  public List<LineString> getDispVecListPtPtLSOtherLine() {
    return this.dispVecPtPtLSOtherPolyList;
  }

  /**
   * @return list with displacement vectors among
   * point and perpendicular point on edge
   * for Line
   * saved as LineString
   */
  public List<LineString> getDispVecListPtEgLSOtherLine() {
    return this.dispVecPtEgLSOtherPolyList;
  }


  /**
   * @return the shortest distance between two objects
   * measured from the polygon outlines (not the centroids);
   * returns -1 if distance > 999999.99m
   */
  public double getShortestDistance() {
    if (this.shortestDistance == 999999.99) {
      this.shortestDistance = -1;
    }
    return this.shortestDistance;
  }

  /**
   * Do geometries cross each other?
   * one might be inside an other one,
   *
   * @return true for line cross
   */
  public boolean haveLinesCross() {
    return linesCross;
  }

  /**
   * @return true if a conflict between the objects exits, means
   * if to narrow or overlap.
   */
  public boolean haveConflict() {
    return hasConflict;
  }

  /**
   * @return the geometry of the buffered Line
   * which characterizes the minimum distance and signature
   * width to stay away from each other
   */
  public Geometry getLineABufferGeom() {
    return this.bufferLineA;
  }

  /**
   * @return the geometry of the buffered other line,
   * buffer should be equal to signature width
   */
  public Geometry getLineBBufferGeom() {
    return this.bufferLineB;
  }

  /**
   * @return true if line without signature is within
   * buffer distance of poly1
   */
  public boolean haveToShortDistance() {
    return this.areToClose;
  }

  /**
   * @return a Matrix containing the
   * displacement vector element dx.
   * Call first method calcDisplacementVectors().
   */
  public Matrix getDxConflictMatrix() {
    return this.dxConflictMatrix;

  }

  /**
   * @return a Matrix containing the
   * displacement vector element dy.
   * Call first method calcDisplacementVectors().
   */
  public Matrix getDyConflictMatrix() {
    return this.dyConflictMatrix;
  }

  public double getMinDistThreshold() {
    return minDist;
  }

  public double getSignatureAWidthInMetres() {
    return this.signatureA;
  }

  public double getSignatureBWidthInMetres() {
    return this.signatureB;
  }

  /**
   * @return distance thereshold in metres there no displacement
   * vector is calculated since it is complicate to
   * give a displacement direction.
   * Has influence on fields identityFirstPtLineA
   * identityLastPtLineA
   */
  public double getCrucialDistance() {
    return crucialDistance;
  }

  /**
   * @param crucialDistance distance thereshold in metres there no
   *                        displacement vector is calculated since it is complicate to
   *                        give a displacement direction.
   *                        Has influence on fields identityFirstPtLineA
   *                        identityLastPtLineA
   */
  public void setCrucialDistance(double crucialDistance) {
    this.crucialDistance = crucialDistance;
  }

  /**
   * If point is identic to other line, than line could be part
   * of a network and should not be moved.
   *
   * @return identity = true if
   * Dist(VertexOfLineA_0 : VertexOfLineB) < crucialDistance
   */
  public boolean hasIdentityFirstPtLineA() {
    return identityFirstPtLineA;
  }

  /**
   * If point is identic to other line, than line could be part
   * of a network and should not be moved.
   *
   * @return identity = true if
   * Dist(VertexOfLineA_end : VertexOfLineB) < crucialDistance
   */
  public boolean hasIdentityLastPtLineA() {
    return identityLastPtLineA;
  }

  /**
   * @return a special List type of the conflicts as
   * MinWidthConflict type
   */
  public MWConflictList getMwclist() {
    return mwclist;
  }
}
