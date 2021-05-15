package mapgen.algorithms.polygons;


import mapgen.geomutilities.LineIntersection;
import mapgen.measures.supportclasses.MWConflictList;
import mapgen.measures.supportclasses.MinWidthConflict;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

/**
 * Displaced the two nearest non-adjacent parts without
 * modifing the global shape of the polygon. <p>
 * Algorithm proposed by M. Barrault (Agent Delivery D1).<p>
 * Get resulting geometry using getOutPolygon().
 * <p>
 * ******************
 * Comment: solve MWP conflicts in an iterative way
 * (in combination with MinWidthParts measure)
 * *****************
 * TODO:
 * - deal with points which are on the start or end of the polygon linestring<p>
 * - look if changed edges are to short (ShortestEdge) after change<p>
 * - deal with second point of Point-Point conflicts instead with the whole edge<p>
 * <p>
 * created on 		07.01.2005
 *
 * @author sstein
 */
public class BuildingEnlargeWidthLocaly {

  private Polygon inPolygon;
  private Polygon outPolygon = null;
  private MWConflictList mwcList;
  private boolean fixVertex;
  //private ArrayList intersectionPoints = new ArrayList();

  /**
   * calls calculate() automatically
   *
   * @param geom
   * @param conflictList
   * @param fixVertex    should the conflict vertex be fixed and only the edge displaced?
   */
  public BuildingEnlargeWidthLocaly(Polygon geom, MWConflictList conflictList, boolean fixVertex) {
    this.inPolygon = geom;
    this.mwcList = conflictList;
    this.fixVertex = fixVertex;
    //=========
    this.calculate();
  }

  /**
   * sstein:
   * the calculation is done by a parallel displacement of the vertex
   * and adjacent edges. The new vertices of the adjacent edges are
   * calculated by the intersection of the the displaced line segments
   * with the previous and following segment. <p>
   * regarding fixVertex = true<p>
   * only edge will be displaced otherwise
   * edge and vertex are displaced by half distance
   * <p>
   * TODO: what about ShortestEdge length?
   */
  public void calculate() {
    Polygon tempGeom = (Polygon) this.inPolygon.copy();
    double dx, dy;
    //-- get smallest conflict
    MinWidthConflict mc = mwcList.getStrongestConflict();
    //--get displacement argument for vertex
    if (!this.fixVertex) {
      // shift only half distance
      dx = mc.ptDispDx / 2.0;
      dy = mc.ptDispDy / 2.0;
    } else {
      dx = mc.ptDispDx;
      dy = mc.ptDispDy;
    }
    //-------------
    // displace vertex
    // here the two adjacent edges have to be changed as well
    //-------------
    if (!this.fixVertex) {
      LineString ring;
      if (mc.ptHoleIdx > 0) {
        //-- Idx-1 since 0 is value for exterior ring for MWconflict
        //   but in JTS 0 is first interior ring
        ring = tempGeom.getInteriorRingN(mc.ptHoleIdx - 1);
      } else {
        ring = tempGeom.getExteriorRing();
      }
      int nrPoints = ring.getNumPoints();
      Coordinate vertex, vahead, vnext, vaahead, vnnext;
      //-- second point  &&   last-1 point
      if ((mc.ptIdx > 0) && (mc.ptIdx < nrPoints - 1)) {
        vertex = ring.getCoordinateN(mc.ptIdx);
        vahead = ring.getCoordinateN(mc.ptIdx - 1);
        vnext = ring.getCoordinateN(mc.ptIdx + 1);
        //-- save old coordinates for intersection
        Coordinate vaheadOld = (Coordinate) vahead.clone();
        Coordinate vnextOld = (Coordinate) vnext.clone();
        //-- next points for intersection
        if (mc.ptIdx == 1) {
          vaahead = ring.getCoordinateN(nrPoints - 2);
        } else {
          vaahead = ring.getCoordinateN(mc.ptIdx - 2);
        }
        if (mc.ptIdx == nrPoints - 2) {
          vnnext = ring.getCoordinateN(1);
        } else {
          vnnext = ring.getCoordinateN(mc.ptIdx + 2);
        }
        //-- move points
        vertex.x = vertex.x + dx;
        vertex.y = vertex.y + dy;
        vahead.x = vahead.x + dx;
        vahead.y = vahead.y + dy;
        vnext.x = vnext.x + dx;
        vnext.y = vnext.y + dy;
        //-- calculate new position for vahead and vnext
        //   by line intersection
        try {
          // the coordinates have to be set separate,
          // otherwise the assignment to the polygon vertex fails
          LineIntersection lp = LineIntersection.intersectionPoint(vertex, vahead, vaheadOld, vaahead);
          vahead.x = lp.getCoordinate().x;
          vahead.y = lp.getCoordinate().y;
          LineIntersection ln = LineIntersection.intersectionPoint(vertex, vnext, vnextOld, vnnext);
          vnext.x = ln.getCoordinate().x;
          vnext.y = ln.getCoordinate().y;
          //---
                        /*
                        Point p1 = new GeometryFactory().createPoint(vahead);
                        this.intersectionPoints.add(p1);                        
                        Point p2 = new GeometryFactory().createPoint(vnext);
                        this.intersectionPoints.add(p2);
                        */
          //--- set changes of first point as well to
          //    last point (for ring) and conversely
          if (mc.ptIdx == 1) {
            ring.getCoordinateN(nrPoints - 1).x = vahead.x;
            ring.getCoordinateN(nrPoints - 1).y = vahead.y;
          }
          if (mc.ptIdx == nrPoints - 2) {
            ring.getCoordinateN(0).x = vnext.x;
            ring.getCoordinateN(0).y = vnext.y;
          }
        } catch (Exception e) {
          System.out.println("BuildingEnlargeWidthLocaly: line intersection could not be calculated");
        }
      } else {
        System.out.println("vertex is first or last point.");
        System.out.println("set field >>fixVertex = true<<");
        this.fixVertex = true;
        dx = mc.ptDispDx;
        dy = mc.ptDispDy;
      }
    }//end if(fixVertex == false)

    //-------------
    // displace edge if point edge conflict
    //-------------
    //            if (mc.ptEdgeConflict == true){
    //*************
    //commented "if" => displaces now every conflict case
    //*************
    LineString ring2;
    if (mc.edgeHoleIdx > 0) {
      //-- Idx-1 since 0 is value for exterior ring for MWconflict
      //   but in JTS 0 is first interrior ring
      ring2 = tempGeom.getInteriorRingN(mc.edgeHoleIdx - 1);
    } else {
      ring2 = tempGeom.getExteriorRing();
    }
    int nrPoints2 = ring2.getNumPoints();
    Coordinate vstart, vsstart, vend, veend, vstartOld, vendOld;
    vstart = ring2.getCoordinateN(mc.edgeStartPtIdx);
    vstartOld = (Coordinate) vstart.clone();
    vend = ring2.getCoordinateN(mc.edgeEndPtIdx);
    vendOld = (Coordinate) vend.clone();
    //next points needed for intersection
    if (mc.edgeStartPtIdx == 0) {
      vsstart = ring2.getCoordinateN(nrPoints2 - 2);
    } else {
      vsstart = ring2.getCoordinateN(mc.edgeStartPtIdx - 1);
    }
    if (mc.edgeEndPtIdx == nrPoints2 - 1) {
      veend = ring2.getCoordinateN(1);
    } else {
      veend = ring2.getCoordinateN(mc.edgeEndPtIdx + 1);
    }
    //-- move points
    //   .. but to opposite direction!!!
    vstart.x = vstart.x - dx;
    vstart.y = vstart.y - dy;
    vend.x = vend.x - dx;
    vend.y = vend.y - dy;
    //-- calculate new position for vstart and vend
    //   by line intersection
    try {
      // the coordinates have to be set separate,
      // otherwise the assignment to the polygon vertex fails
      LineIntersection lp = LineIntersection.intersectionPoint(vstart, vend, vstartOld, vsstart);
      vstart.x = lp.getCoordinate().x;
      vstart.y = lp.getCoordinate().y;
      LineIntersection ln = LineIntersection.intersectionPoint(vstart, vend, vendOld, veend);
      vend.x = ln.getCoordinate().x;
      vend.y = ln.getCoordinate().y;

      //--- if startpointIdx = 0 then last point of ring
      //    has to be set as well and vice versa
      if (mc.edgeStartPtIdx == 0) {
        ring2.getCoordinateN(nrPoints2 - 1).x = vstart.x;
        ring2.getCoordinateN(nrPoints2 - 1).y = vstart.y;
      }
      if (mc.edgeEndPtIdx == nrPoints2 - 1) {
        ring2.getCoordinateN(0).x = vend.x;
        ring2.getCoordinateN(0).y = vend.y;
      }

    } catch (Exception e) {
      System.out.println("BuildingEnlargeWidthLocaly: line intersection could not be calculated");
    }
    if (tempGeom.isValid())
      this.outPolygon = (Polygon)tempGeom.copy();
    else throw new RuntimeException("Invalid geometry produced: " +tempGeom);
  }

  /*************** getters and setters ****************/

  public Polygon getInPolygon() {
    return inPolygon;
  }

  public void setInPolygon(Polygon inPolygon) {
    this.inPolygon = inPolygon;
  }

  public Polygon getOutPolygon() {
    return outPolygon;
  }

  public void setOutPolygon(Polygon outPolygon) {
    this.outPolygon = outPolygon;
  }

  /**
   * if yes, only edge will be displaced otherwise
   * edge and vertex are displaced by half distance
   *
   * @return
   */
  public boolean isFixVertex() {
    return fixVertex;
  }

  /**
   * if yes, only edge will be displaced otherwise
   * edge and vertex are displaced by half distance
   *
   * @param fixVertex
   */
  public void setFixVertex(boolean fixVertex) {
    this.fixVertex = fixVertex;
  }

  public MWConflictList getMwcList() {
    return mwcList;
  }

  public void setMwcList(MWConflictList mwcList) {
    this.mwcList = mwcList;
  }

}
