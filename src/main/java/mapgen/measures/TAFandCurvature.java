package mapgen.measures;

import mapgen.geomutilities.TangentAngleFunction;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

/**
 * 	Calculates - azimut
 * 			   - distances
 * 			   - tangent angle function
 * 			   - curvature
 * 			   - variance, deviation
 *  using TangenAngleFunction() class
 * 
 * TODO: deal with inner rings for polygons as well
 *       not only for exterior ring
 *
 * created on 		17.11.2004
 * @author sstein 
 */
public class TAFandCurvature{
        
    private final double curvThreshold;
    private TangentAngleFunction myTAF = null;
    
    /**
     * Constructor 0
     * calculates the angles for a Polygon or LineString,
     * calcs curvature and their variance,
     * for Polygons only exterior ring is taken into account
     * @param myGeometry a Geometry
     */
    public TAFandCurvature(Geometry myGeometry){
        this(myGeometry, Math.PI*2);
    }
    
    /**
     * Constructor 0a
     * calculates the angles for a Polygon or LineString
     * calcs curvature variance for curvature values below a threshold
     * for Polygons only exterior ring is taken into account 
     * @param myGeometry a Geometry
     * @param threshold a threshold
     */
    public TAFandCurvature(Geometry myGeometry, double threshold){
        this.curvThreshold = threshold;
        if (myGeometry instanceof Polygon){            
            Polygon myPolygon = (Polygon)myGeometry;            
            // calc for ExteriorRing
            LineString myOuterRing = myPolygon.getExteriorRing();
            this.calcTafAndCurv(myOuterRing);
            /**********
            // calc for InterriorRings
            int nrIntRings = myPolygon.getNumInteriorRing();
            if (nrIntRings > 0){
                for (int i = 0; i < nrIntRings; i++) {
                    LineString myInnerRing = myPolygon.getInteriorRingN(i);               
                    this.calcTafAndCurv(myInnerRing);
                }
            }
            ***********/
        }
        else if(myGeometry instanceof LineString){
            LineString myLine = (LineString)myGeometry;
            this.calcTafAndCurv(myLine);
        }
        else{
            // do something for points or coordinates
        }

    }
    
    /**
     * constructor 1a
     * calculates the angles, curvature ando so on for a polygon,
     * only exterior ring is taken into account 
     * @param myPolygon a Polygon
     * @param threshold in Radian (not degree)
     */
    public TAFandCurvature(Polygon myPolygon, double threshold){
        
        this.curvThreshold = threshold;
        // calc  for ExteriorRing
        LineString myOuterRing = myPolygon.getExteriorRing();
        this.calcTafAndCurv(myOuterRing);
        /**
        // calc for InterriorRings
        int nrIntRings = myPolygon.getNumInteriorRing();
        if (nrIntRings > 0){
            for (int i = 0; i < nrIntRings; i++) {
                LineString myInnerRing = myPolygon.getInteriorRingN(i);               
                this.calcTafAndCurv(myInnerRing);
             }
        }
        **/
    }

    /**
     * constructor 1b
     * calculates the angles, curvature ando so on for a polygon
     * only exterior ring is taken into account 
     * @param myPolygon a Polygon
     */
    public TAFandCurvature(Polygon myPolygon){        
        this(myPolygon ,Math.PI*2);
    }

    /**
     * constructor 2a
     * calculates the angles, curvature and so on of a line 
     * and tests if they are below a given threshold value  
     * @param myLine a LineString
     * @param threshold in Radian (not degree)
     */
    public TAFandCurvature(LineString myLine, double threshold){
        this.curvThreshold = threshold;
        this.calcTafAndCurv(myLine);
    }
    
    /**
     * constructor 2b
     * calculates the angles, curvature and so on of a line 
     * @param myLine a LineString
     */
    public TAFandCurvature(LineString myLine){
        this.curvThreshold = Math.PI*2;
        this.calcTafAndCurv(myLine);
    }
    

    /** 
     * @param myLine a LineString
     * calculates line specific values using 
     * TangenAngleFunction() class
     */
    private void calcTafAndCurv(LineString myLine){
        
        Coordinate[] myCoordinates = myLine.getCoordinates();
        myTAF = new TangentAngleFunction(myCoordinates);
        myTAF.setCurvThreshold(this.curvThreshold);
    }


    public double[] getCurvature(){
        return myTAF.getCurv();
    }

    public double getCurvatureDeviation(){
        return myTAF.getCurvDeviation();
    }

    public double getCurvatureVariance(){
        return myTAF.getCurvVar();
    }
    
    public double getCurvatureMean(){
        return myTAF.getCurvMean();
    }    
    
    public double[] getAzimut(){
        return myTAF.getAzimut();
    }
    
    /**
     * @return
     * if less than 10 points, line curvature variance is 
     * statistical not confident
     */
    public double getNrVariancePoints(){
        return myTAF.getNrVariancePoints();
    }
  
    public double getNrVertices(){
        return myTAF.getNrVertices();
    }

    /**
     * threshold to calculate variance
     * only smaller curv values are taken into account 
     * @return the curvature threshold
     */
    public double getCurvThreshold(){
        return myTAF.getCurvThreshold();
    }
    
    /**
     * threshold to calculate variance
     * only smaller curv values are taken into account 
     * @param value new curvature threshold value
     */
    public void setCurvThreshold(double value){
        myTAF.setCurvThreshold(value);
    }
    
    /**
     * The distance for all Points of a Line or Ring
     * 
     * @return distances between the verticies
     */
    public double[] getDistances(){
        return myTAF.getDistances();
    }
    
    /**
     * The angles for all Points of a Line or Ring
     * 
     * @return angles as double array, angle values in radians
     */
    public double[] getTAF(){        
        return myTAF.getTaf();              
    }
    
    public double getDistanceDeviation(){
        return myTAF.getDistanceDeviation();
    }
    
    public double getDistanceVariance(){
        return myTAF.getDistanceVariance();
    }
    
    public double getDistanceMean(){
        return myTAF.getDistanceMean();
    }
    
    
}
       
