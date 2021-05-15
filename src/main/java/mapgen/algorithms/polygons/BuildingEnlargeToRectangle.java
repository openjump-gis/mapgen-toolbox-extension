package mapgen.algorithms.polygons;

import mapgen.measures.OrientationMBR;
import mapgen.measures.PolygonShapeMeasures;
import org.locationtech.jts.geom.Polygon;

/**
 * Simplification and enlargement of building. Computes the minimum
 * bounding rectangle and scales it to a user defined size.
 * Further it checks if a minimal elongation value is fullfilled.<p>
 * Algorithm proposed by Mats Bader (Agent Delivery D1)
 * <p>
 * created on 		09.01.2005
 *
 * @author sstein
 */
public class BuildingEnlargeToRectangle {

  private Polygon inPolygon;
  private Polygon outPolygon = null;
  private double finalArea;
  private double elongationThreshold = 0.25;

  /**
   * initialises and calls calculate()
   *
   * @param poly                building geometry
   * @param finalArea           as minimum area size in m^2
   * @param elongationThreshold minimum value of elongation
   *                            (ratio: witdh/length = between 0 and 1)
   */
  public BuildingEnlargeToRectangle(Polygon poly, double finalArea, double elongationThreshold) {
    this.inPolygon = poly;
    this.finalArea = finalArea;
    this.elongationThreshold = elongationThreshold;
    this.calculate();
  }

  /**
   * creates mbr with specified area size
   * initialises and calls calculate()
   *
   * @param poly      building geometry
   * @param finalArea as minimum area size in m^2
   */
  public BuildingEnlargeToRectangle(Polygon poly, double finalArea) {
    this.inPolygon = poly;
    this.finalArea = finalArea;
    this.calculate();
  }


  /**
   * creates MBR with same area as input polygon
   * initialises and calls calculate()
   *
   * @param poly building geometry
   *             //@param finalArea as minimum area size in m^2
   */
  public BuildingEnlargeToRectangle(Polygon poly) {
    this.inPolygon = poly;
    this.finalArea = 0;
    this.calculate();
  }


  public void calculate() {

    OrientationMBR oMbr = new OrientationMBR(this.inPolygon);
    Polygon mbr = oMbr.getMbr();
    this.outPolygon = mbr;
    //--------------------
    // check minElongation
    //--------------------
    double width = oMbr.getMbrWidth();
    double length = oMbr.getMbrLength();
    double elongation = PolygonShapeMeasures.calcBuildingElongation(width, length);
    if (elongation < this.elongationThreshold) {
      //-- changeElongation
      //  goal elongation is with the following solution approximately reached
      double newWidth = this.elongationThreshold * length;
      double scaleFactor = newWidth / width;
      double angle = oMbr.getMbrOrientation() + Math.PI / 2.0;
      PolygonChangeElongation pce = new PolygonChangeElongation(oMbr.getMbr(), scaleFactor, angle);
      this.outPolygon = pce.getOutPolygon();
      //-- checkNewElongation
      /**
       OrientationMBR oMbrN = new OrientationMBR(this.outPolygon);
       double widthN = oMbrN.getMbrWidth();
       double lengthN = oMbrN.getMbrLength();
       double elongationN = PolygonShapeMeasures.calcBuildingElongation(widthN,lengthN);
       if(elongationN < this.elongationThreshold){
       System.out.println("BuildingEnlargeToRectangle: change elongation to minElongation failed!");
       }
       **/
    }
    //--------------------
    // check minArea
    //--------------------
    double mbrArea = this.outPolygon.getArea();
    //-- resize to the mbr
    if (this.finalArea == 0) { // if no final area is specified then resize to orginal area
      this.finalArea = this.inPolygon.getArea();
    }
    //  goal area size is with the following solution approximately reached
    double areaFactor = this.finalArea / mbrArea;
    double scaleFactor = Math.sqrt(areaFactor);
    OrientationMBR oMbrN = new OrientationMBR(this.outPolygon);
    double angle = oMbrN.getStatOrientation(); // use wall statistical weight
    //-- btw: the strechPolygon method might be the same as PolygonChangeElongation method
    //-- stretch in one mbr dircetion
    StretchPolygon.stretchPolygon(this.outPolygon, angle, scaleFactor);
    //-- stretch in other mbr dircetion
    StretchPolygon.stretchPolygon(this.outPolygon, angle + Math.PI / 2.0, scaleFactor);
    //-- checkAreaSize
    /**
     double areaN = this.outPolygon.getArea();
     if(areaN < this.finalArea){
     System.out.println("BuildingEnlargeToRectangle: change area to minArea failed!");
     }
     **/
  }
  /******************** getters and setters ***************/

  /**
   * @return minimal area in m^2
   */
  public double getFinalArea() {
    return finalArea;
  }

  /**
   * @param finalArea: minimal area in m^2
   */
  public void setFinalArea(double finalArea) {
    this.finalArea = finalArea;
  }

  public Polygon getInPolygon() {
    return inPolygon;
  }

  public void setInPolygon(Polygon inPolygon) {
    this.inPolygon = inPolygon;
  }

  public Polygon getOutPolygon() {
    return outPolygon;
  }

  public double getElongationThreshold() {
    return elongationThreshold;
  }

  /**
   * elongation is the ratio: width/length
   *
   * @param elongationThreshold mimimum value between 0 and 1
   */
  public void setElongationThreshold(double elongationThreshold) {
    this.elongationThreshold = elongationThreshold;
  }
}	
