package mapgen.algorithms.polygons;


import mapgen.algorithms.PolyRingIndex;
import mapgen.geomutilities.ModifyPolygonPoints;
import mapgen.measures.supportclasses.PointInLineConflict;
import mapgen.measures.supportclasses.PointInLineConflictList;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

/**
 * Deletes the points on a line which could emerge from the union operation of
 * adjacent polygons.
 * It deletes the points only if the angle is about 180degrees and the
 * distance (point, to new building wall) is smaller than a threshold.
 * <p>
 * created on 		01.12.2005 based on BuildingOutlineSimplify
 *
 * @author sstein
 */
public class BuildingDeletePointsInLine {

  private Polygon inPolygon;
  private Polygon outPolygon;
  private final PointInLineConflictList seList;
  private boolean couldNotSolve = false;
  private boolean alreadySimple = false;

  public BuildingDeletePointsInLine(Polygon geom, PointInLineConflictList conflictList) {
    this.inPolygon = (Polygon) geom.copy();
    this.outPolygon = (Polygon) geom.copy();
    this.seList = conflictList;
    //=========
    if ((this.inPolygon.getNumInteriorRing() == 0) &&
        this.inPolygon.getExteriorRing().getNumPoints() <= 5) {
      System.out.println("BuildingOutlineSimplify: Polygon is already simple!");
      this.couldNotSolve = true;
      this.alreadySimple = true;
    } else {
      this.calculate();
    }
  }

  public void calculate() {
    Polygon tempGeom = (Polygon) this.inPolygon.copy();
    //-- go backwards through the list
    //   since deleting points changes the point index
    for (int i = seList.size(); i >= 1; i--) {
      PointInLineConflict plc = seList.get(i - 1);
      System.out.println("BuildingDeletePointsOnLine: delete point: " + plc.pointIdx);
      tempGeom = this.deleteEdge(plc, tempGeom);
    }
    this.outPolygon = (Polygon) tempGeom.copy();
  }

  /**
   * does the calculations to solve the problem
   *
   * @param plc : the conflict
   * @param p   : the polygon with conflicts
   * @return the modified polygon
   */
  private Polygon deleteEdge(PointInLineConflict plc, Polygon p) {

    Polygon newP = (Polygon) p.copy();
    //-- get ring / LineString
    LineString ls0;
    if (plc.pointRingIdx > 0) {
      ls0 = newP.getInteriorRingN(plc.pointRingIdx - 1);
    } else {//edgeRingIdx == 0
      ls0 = newP.getExteriorRing();
    }
    PolyRingIndex idx = new PolyRingIndex(plc.pointIdx, ls0.getNumPoints());
    newP = ModifyPolygonPoints.deletePoint(newP, plc.pointRingIdx, idx.i);
    this.couldNotSolve = false;
    return newP;
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
   * Is simple means the polygon has only four edges
   *
   * @return
   */
  public boolean isAlreadySimple() {
    return alreadySimple;
  }
}
