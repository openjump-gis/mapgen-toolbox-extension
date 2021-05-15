package mapgen.measures;


import java.util.ArrayList;
import java.util.List;

import mapgen.algorithms.Rotate;
import mapgen.geomutilities.FirstGeodeticTask2d;
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
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.operation.distance.DistanceOp;


/**
 * checks if a polygon and a line:
 * - intersects
 * - touches (have one common edge)
 * - (do not intersect and touch) have a distance,
 * <p>
 * and (if under a given threshold) use calcDisplacementVectors();
 * to calculates the displacement vectors for the conflict points
 * (of outer ring) or centroid - which  depends on type of conflict,
 * <p>
 * to get the conflicts use getMwcList()
 * <p>
 *
 * created on 		06.12.2004
 * last modified:
 * 20.12.2004
 * 18.08.2005 modification for dx,dy matrix store on pt-pt conlficts
 *
 * @author sstein
 */
public class PolygonLineDistance {

  public List<Geometry> geometryList = new ArrayList<>();
  private double angle;
  private final Polygon p1;
  private final LineString lin;
  private final Geometry bufferPoly;
  private final Geometry bufferLine;
  // if objects not intersect
  //private ArrayList minWidthDoubleList = new ArrayList();
  private final List<LineString> dispVecPtEgLSthisPolyList = new ArrayList<>();
  private final List<LineString> dispVecPtPtLSthisPolyList = new ArrayList<>();
  private final List<LineString> dispVecPtEgLSOtherPolyList = new ArrayList<>();
  private final List<LineString> dispVecPtPtLSOtherPolyList = new ArrayList<>();
  private Matrix dxConflictMatrix;
  private Matrix dyConflictMatrix;
  private double shortestDistance = 999999.99; //init value 999km
  // for intersection
  private LineString dispVectorOnCentroid = null;
  // ---
  private final MWConflictList mwclist = new MWConflictList();
  private boolean hasConflict = false;
  private boolean linesCross = false;
  private boolean commonEdge = false;
  private boolean areToClose = false;
  private final double minDist;
  private final double signature;
  // if dist[m] < field: "crucialDistance" it is complicate to give a
  // displacement direction
  private double crucialDistance = 0.1;

  /**
   * tests if two polygons have a distance conflict
   * - check for a common edge
   * - check for intersection
   * - checks if the minimum buffer is joint/touched
   * to calculate the displacement use calcDisplacementVectors();
   *
   * @param poly1              input Polygon
   * @param line               input LineString
   * @param minDist            distance in metres to separate objects
   * @param lineSignatureWidth radius of line signature [in metres]
   */
  public PolygonLineDistance(Polygon poly1, LineString line, double minDist, double lineSignatureWidth) {
    this.minDist = minDist;
    this.signature = lineSignatureWidth;
    this.p1 = poly1;
    this.lin = line;

    //--IntersectionMatrix
    IntersectionMatrix myIM = p1.relate(this.lin);
    System.out.println("IntersectionMat: tr-bl : "
        + myIM.get(0, 0) + " " + myIM.get(0, 1) + " " + myIM.get(0, 2) + ","
        + myIM.get(1, 0) + " " + myIM.get(1, 1) + " " + myIM.get(1, 2) + ","
        + myIM.get(2, 0) + " " + myIM.get(2, 1) + " " + myIM.get(2, 2));

    //-- test if have common edge
    if (myIM.matches("F***1****")) {
      this.commonEdge = true;
    }
    // test if Polygon and Line cross/intersect (JTS function)
    // but exclude case there one point is on one edge => I1 + E2 =-1 && E1+ I2 = +1
    if (!myIM.matches("FF*FF****")) {
      //*********
      // intersect
      // here displacement vector is calculated for centroid
      //***************************/
      this.hasConflict = true;
      this.linesCross = true;
    }
    //else{
    this.bufferPoly = p1.buffer(this.minDist);
    if (this.signature > 0) {
      this.bufferLine = line.buffer(this.signature);
    } else {
      this.bufferLine = line;
    }
    if (!bufferPoly.disjoint(this.bufferLine)) {
      // false disjoint = intersects
      this.hasConflict = true;
      this.areToClose = true;
    }
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
    int nrPtBaseRing = p1.getExteriorRing().getNumPoints();
    int nrPtTestRing = lin.getNumPoints();
    this.dxConflictMatrix = MatlabSyntax.zeros(nrPtBaseRing, nrPtTestRing);
    this.dyConflictMatrix = MatlabSyntax.zeros(nrPtBaseRing, nrPtTestRing);

    if (this.commonEdge) {
      // don't do something .. but you can merge the polys
    } else if (this.linesCross) {
      this.calcDispVectorForIntersection((Polygon)p1.copy(), this.lin);
    } else if (this.areToClose) {
      //-- if polygons are disjoint but within the buffer (treshold)
      // test cross wise (point to edge)

      this.testWithOtherRings(p1.getExteriorRing(), this.lin, 1);
      this.testWithOtherRings(this.lin, p1.getExteriorRing(), 2);
      this.calcMeanDisplacementForPolygon();
    }

  }

  /**
   * If polygons are disjoint:
   * Test distance of points from one Linestring to edges of another LineString
   *
   * @param baseRing base ring
   * @param testRing ring to test
   * @param originalPolygon    of them is the polygon,
   *                 need to change direction of displacement vector
   */
  private void testWithOtherRings(LineString baseRing,
                                  LineString testRing,
                                  int originalPolygon) {
    // **********  if rings are seperated **************
    GeometryFactory myGF = new GeometryFactory();
    //Coordinate[] edge = new Coordinate[2];

    //GeometryLocation[] ClosestVector = null;
    //LineString tempLine = null;
    double dist;
    // ==== loop for over all points =====
    for (int i = 0; i < baseRing.getNumPoints(); i++) {
      Point myPoint = baseRing.getPointN(i);
      // -1; since edge(j,j+1)
      for (int j = 0; j < testRing.getNumPoints() - 1; j++) {
        //reduce calculations concerning adjacent edges
        Coordinate A = testRing.getCoordinateN(j);
        Coordinate B = testRing.getCoordinateN(j + 1);
        Coordinate P = myPoint.getCoordinate();
        PointLineDistance pld = new PointLineDistance(P, A, B);
        dist = pld.getDistance();
        boolean isPerpendicularPoint = pld.isPointOnLine();
        Coordinate rp = pld.getClosestPoint().getCoordinate();

        //-- calc displacementVector
        if (dist < (this.minDist + this.signature)) {
          //========== calc disp vector and save as linestring =======
          double xNew, yNew, dxMove, dyMove;
          // dist > 10cm .. since numerical problems lead to distance like 3.1*10^-14
          if (dist > this.crucialDistance) {
            double dxAct, dyAct, dxRef, dyRef;
            dxAct = P.x - rp.x;
            dyAct = P.y - rp.y;
            dxRef = dxAct * (this.minDist + this.signature) / dist;
            dyRef = dyAct * (this.minDist + this.signature) / dist;
            dxMove = dxRef - dxAct;
            dyMove = dyRef - dyAct;
            xNew = myPoint.getX() + dxMove;
            yNew = myPoint.getY() + dyMove;
          } else {
            //-- if dist = 0, then calc disp vector towards poly centroid
            Point centroid = this.p1.getCentroid();
            double angle = SecondGeodeticTask2d.calcAngle2Points(myPoint, centroid);
            Point newPoint = FirstGeodeticTask2d.getPoint(myPoint, angle, (this.minDist + this.signature));
            xNew = newPoint.getX();
            yNew = newPoint.getY();
            dxMove = newPoint.getX() - myPoint.getX();
            dyMove = newPoint.getY() - myPoint.getY();
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
              this.dispVecPtEgLSthisPolyList.add((LineString)dispVector.copy());
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
              //[sstein] 18.08.2005 changed (see LineLineDistance.java)
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
            }
          }
        }// end if dist < dist+signature
      }  // end for edges
    } // end for test point
  }

  private void calcMeanDisplacementForPolygon() {
    // uses point-edge conlficts only
    int total = 0;
    double sumdx = 0, sumdy = 0;
    double angle, distance, deast, dnorth;
    for (int i = 0; i < this.dispVecPtEgLSthisPolyList.size(); i++) {
      LineString myDispV = this.dispVecPtEgLSthisPolyList.get(i);
      angle = SecondGeodeticTask2d.calcAngle2Points(myDispV.getStartPoint(),
          myDispV.getEndPoint());
      distance = myDispV.getLength();
      deast = distance * Math.cos(angle);
      dnorth = distance * Math.sin(angle);
      sumdx = sumdx + deast;
      sumdy = sumdy + dnorth;
      total = total + 1;
    }

    for (int i = 0; i < this.dispVecPtEgLSOtherPolyList.size(); i++) {
      LineString myDispV = this.dispVecPtEgLSOtherPolyList.get(i);
      angle = SecondGeodeticTask2d.calcAngle2Points(myDispV.getStartPoint(),
          myDispV.getEndPoint());
      // turn displacement direction if conflict was reported from
      // other LineString than from base poly
      angle = angle + Math.PI;

      distance = myDispV.getLength();
      deast = distance * Math.cos(angle);
      dnorth = distance * Math.sin(angle);
      sumdx = sumdx + deast;
      sumdy = sumdy + dnorth;
      total = total + 1;
    }

    /*** this is not wished - polygon contacts should be kept
     // also use point-poin conflicts for case of the identic points
     for(int i = 0; i < this.dispVecPtPtLSthisPolyList.size(); i++){
     LineString myDispV = (LineString)this.dispVecPtPtLSthisPolyList.get(i);
     angle = SecondGeodeticTask2d.calcAngle2Points(myDispV.getStartPoint(),
     myDispV.getEndPoint());
     // turn displacement direction if conflict was reported from
     // other polygon than from base poly
     angle = angle+ Math.PI;

     distance = myDispV.getLength();
     deast = distance * Math.cos(angle);
     dnorth = distance * Math.sin(angle);
     sumdx = sumdx + deast;
     sumdy = sumdy + dnorth;
     total= total +1;
     }
     ****/


    double dx = sumdx / (double) total;
    double dy = sumdy / (double) total;
    Point centerp1 = p1.getCentroid();
    Coordinate[] coord = new Coordinate[]{
        new Coordinate(centerp1.getX(), centerp1.getY()),
        new Coordinate(centerp1.getX() + dx, centerp1.getY() + dy)
    };
    GeometryFactory myGF = new GeometryFactory();
    this.dispVectorOnCentroid = myGF.createLineString(coord);

  }

  private void calcDispVectorForIntersection(Polygon p1, LineString line) {
    //-- calc the intersection geometry
    Geometry intersection = p1.intersection(line);
    //-- get Centroids of p1
    JtsCentroid c1 = new JtsCentroid(p1);
    Point center1 = c1.getGravityCentroid();
    this.geometryList.add(center1.copy());
    //-- get shortest Point from centroid to intersection line
    Coordinate[] closestC = DistanceOp.nearestPoints(intersection, center1);
    Point closestP = new GeometryFactory().createPoint(closestC[0]);
    //-- get horizontal angle of vector between the closest point
    //   and the centroid
    //   to get the direction for translation vector
    double angle = SecondGeodeticTask2d.calcAngle2Points(center1, closestP);
    //-- rotate geometry by modified angle
    Geometry rotGeometry = Rotate.rotate(center1, Math.PI / 2.0 - angle, p1);
    this.geometryList.add(rotGeometry.copy());
    //-- calculate Bounding Rectangle
    Geometry envelope = rotGeometry.getEnvelope();
    this.geometryList.add(envelope.copy());
    //-- calculate length to displace from envelope height
    Coordinate[] evCoords = envelope.getCoordinates();
    double s = Math.abs(evCoords[0].y - evCoords[1].y);
    if (s == 0) {
      s = Math.abs(evCoords[0].y - evCoords[2].y);
    }
    // distance to move = centroid distance to hull - centroid distance to closest point
    double cc = closestP.distance(center1);
    double distToMove = s / 2.0 - cc + this.minDist + this.signature;
    //-- get Point to displace to
    Point toPoint = FirstGeodeticTask2d.getPoint(center1, angle + Math.PI, distToMove);
    this.geometryList.add(toPoint.copy());
    //-- get disp vector as LineString
    Coordinate[] dispVectorCoord = new Coordinate[2];

    dispVectorCoord[0] = center1.getCoordinate();
    dispVectorCoord[1] = toPoint.getCoordinate();
    LineString dispVector = new GeometryFactory().createLineString(dispVectorCoord);
    this.geometryList.add(dispVector.copy());

    //-- set return values
    this.dispVectorOnCentroid = dispVector;
    this.angle = angle * 180 / Math.PI;
    this.shortestDistance = -1 * distToMove;

  }

  /******************** getters and setter **********************/

  /**
   * @return list with displacement vectors among
   * point and edge node for Base-Polygon
   * saved as LineString
   */
  public List<LineString> getDispVecListPtPtLSThisPoly() {
    return dispVecPtPtLSthisPolyList;
  }

  /**
   * @return list with displacement vectors among
   * point and perpendicular point on edge
   * for Base-Polygon
   * saved as LineString
   */
  public List<LineString> getDispVecListPtEgLSThisPoly() {
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
   *
   * @return widths/distances under threshold as double array
   */
  /**
   public double[] getMininmalDistances(){

   double[] width = new double[this.minWidthDoubleList.size()];
   int i=0;
   Double value;
   for (Iterator iter = minWidthDoubleList.iterator(); iter.hasNext();) {
   value = (Double) iter.next();
   width[i] = value.doubleValue();
   i = i+1;
   }
   return width;
   }
   **/

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
   * get Displacementvector for first polygon
   * (on centroid) if objetcs do intersect
   *
   * @return LineString (second point is toPoint)
   */
  public LineString getDispVectorOnCentroid() {
    return dispVectorOnCentroid;
  }

  /**
   * @return true if a conflict between the objects exits, means
   * if to narrow or overlap. Attention: but is false if objects
   * have a common edge
   */
  public boolean haveConflict() {
    return hasConflict;
  }


  /**
   * only usefull for intersection
   *
   * @return (angle in rad) of the connection vector
   * between the polygon centroid and the closest point of line
   */
  public double getAngle() {
    return angle;
  }

  /**
   * @return the geometry of the buffered polygon
   * which characterizes the minimum distance to stay away
   * from each other (without line signatur width)
   */
  public Geometry getPolyBufferGeom() {
    return this.bufferPoly;
  }

  /**
   * @return the geometry of the buffered line,
   * buffer should be equal to signature width
   */
  public Geometry getLineBufferGeom() {
    return this.bufferLine;
  }

  /**
   * @return true if polygon and line (without signature) have
   * a common edge. This might be important if objects should be
   * aggregated instead displaced
   */
  public boolean haveCommonEdge() {
    return commonEdge;
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
   * displacement vector element dx
   */
  public Matrix getDxConflictMatrix() {
    return this.dxConflictMatrix;

  }

  /**
   * @return a Matrix containing the
   * displacement vector element dy
   */
  public Matrix getDyConflictMatrix() {
    return this.dyConflictMatrix;
  }

  public double getMinDisthreshold() {
    return minDist;
  }

  public double getSignatureWidthInMetres() {
    return signature;
  }

  /**
   * @return distance threshold in metres there
   * it is complicate to give a correct displacement direction.
   */
  public double getCrucialDistance() {
    return crucialDistance;
  }

  /**
   * @param crucialDistance distance thereshold in metres there
   *                        it is complicate to give a corrct displacement direction.
   */
  public void setCrucialDistance(double crucialDistance) {
    this.crucialDistance = crucialDistance;
  }

  /**
   * @return a special List type of the conflicts as
   * MinWidthConflict type
   */
  public MWConflictList getMwclist() {
    return mwclist;
  }

}
    