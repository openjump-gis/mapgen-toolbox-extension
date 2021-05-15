package mapgen.geomutilities;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;

/**
 *
 * description:
 * 	calculate the angle of a vector to the horizontal;
 *  calculate the distance between two points;
 *
 * created on 		19.11.2004
 * @author sstein
 */
public class SecondGeodeticTask2d {

    /**
     * calculate the angle of a vector given by first and last point.
     * Computes the angle of edge to the horizontal from 0 .. 2*Pi.
     * @param p1from point 1
     * @param p2to point 2
     * @return angle in radian
     */
    public static double calcAngle2Coords(Coordinate p1from, Coordinate p2to){
        
        double x1 = p1from.x;        
        double y1 = p1from.y;
        double x2 = p2to.x;
        double y2 = p2to.y;
        double dx = x2 - x1;
        double dy = y2 - y1;
        return calcAngle(dx,dy);
    }
    
    /**
     * calculate the angle of a vector given by first and last point.
     * Computes the angle of edge to the horizontal from 0 .. 2*Pi.
     * @param p1from point 1
     * @param p2to point 2
     * @return angle in radian
     */
    public static double calcAngle2Points(Point p1from, Point p2to){
        
        double x1 = p1from.getX();        
        double y1 = p1from.getY();
        double x2 = p2to.getX();
        double y2 = p2to.getY();
        double dx = x2 - x1;
        double dy = y2 - y1;
        return calcAngle(dx,dy);
    }

    /**
     * calculate the angle of a vector given coordinate
     * differences of first and last point.
     * Computes the angle of edge to the horizontal from 0 .. 2*Pi. 
     * @param deast easting diff
     * @param dnorth northing diff
     * @return angle in radian
     */
    public static double calcAngle(double deast, double dnorth){
            
   	 	// Compute angle of edge to the horizontal.        
        double angle = 0;
        if ((deast != 0.0) || (dnorth!=0.0)){
            double rawAngle = Math.atan(dnorth/deast);
            if      ((deast>0 ) && (dnorth>0)) {angle=rawAngle;} 
            else if ((deast>0 ) && (dnorth<0)) {angle=rawAngle+2.0*Math.PI;}
            else if ((deast<0 ) && (dnorth<0)) {angle=rawAngle+Math.PI;} 
            else if ((deast<0 ) && (dnorth>0)) {angle=rawAngle+Math.PI;}
            else if ((deast==0) && (dnorth<0)) {angle=Math.PI*3.0/2; }
            else if ((deast==0) && (dnorth>0)) {angle=Math.PI/2.0; }
            else if ((dnorth==0) && (deast>0)) {angle=0;}
            else if ((dnorth==0) && (deast<0)) {angle=Math.PI;}             
        }        
        else{
            System.out.println("points are identic");
        }
        return angle;
    }
    
    /**
     * 
     * @param p1from jts point 1
     * @param p2to jts point 2
     * @return distance between points
     */    
    public static double calcDistancePoints(Point p1from, Point p2to){        
        return calcDistanceCoord(p1from.getCoordinate(), p2to.getCoordinate());
    }

    /**
     * 
     * @param p1 jts coordinate of point 1
     * @param p2 jts coordinate of point 2
     * @return distance between points
     */
    public static double calcDistanceCoord(Coordinate p1, Coordinate p2){        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return Math.sqrt(dx*dx + dy*dy);
    }
 
    /**
     * The azimuth is calculated between start and endpoint of the LineString. 
     * The lowest point on the Y-axes is used as base point. Hence, return values
     * will be between -90 and +90 degrees 
     * @param firstPCAsLineString
     * @return angle in Degrees
     */
    public static double calcAzimuth(LineString firstPCAsLineString) {
    	double azimut;
    	double horizAngle;
    	//-- get the start and endpoint
    	Point ptA = firstPCAsLineString.getStartPoint();
    	Point ptB = firstPCAsLineString.getEndPoint();
		  //-- find lower point and calculate azimut
    	if (ptA.getY() < ptB.getY()){
    		horizAngle = SecondGeodeticTask2d.calcAngle2Points(ptA, ptB);
    	}
    	else{
    		horizAngle = SecondGeodeticTask2d.calcAngle2Points(ptB, ptA);
    	}
    	//-- horiz. Angle should be only between 0...+180 as we checked the lower point first
    	if (horizAngle <= (Math.PI/2.0)){
    		azimut = (Math.PI/2.0) - horizAngle;
    	}
    	else{
    		azimut = -1*(horizAngle - (Math.PI/2.0));
    	}
		return (azimut*180.0/Math.PI);
	}
}
