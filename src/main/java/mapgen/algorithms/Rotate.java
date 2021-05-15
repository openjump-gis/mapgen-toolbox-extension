/***********************************************
 * created on 		24.09.2004
 * last modified: 	24.09.2004
 * 
 * author:			sstein
 * 
 * description:
 * 	 rotates Geometry objects
 * 
 ***********************************************/
package mapgen.algorithms;

import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Polygon;

/**
 * @description:
 * 	rotates Geometry objects  		
 *  
 * @author sstein
 *
 */
public class Rotate {


    public static Polygon rotate(Point ancorPoint, double angle, Polygon myPolygon){
        
        Polygon newGeom;        
        // outer ring
        LineString outRing = myPolygon.getExteriorRing();
        LinearRing outerRing = rotate(ancorPoint,angle,outRing);
        // inner rings
        int numInnerRings = myPolygon.getNumInteriorRing();
        LineString[] innerRings = new LineString[numInnerRings];
        LinearRing[] innerLinearRings = new LinearRing[numInnerRings];
        for (int j = 0; j < numInnerRings; j++){            
        	innerRings[j] =  myPolygon.getInteriorRingN(j);
        	innerLinearRings[j] = rotate(ancorPoint,angle,innerRings[j]);        	
        }
        GeometryFactory myFac = new GeometryFactory();
        newGeom = myFac.createPolygon(outerRing,innerLinearRings);
                                   
        return newGeom;
    }

    private static LinearRing rotate(Point ancorPoint, double angle, LineString myLineString){
        
        double x1, y1, xn ,yn;
        LinearRing newGeom;        
        
        Coordinate[] coordList = myLineString.getCoordinates();
        int nrPoints = myLineString.getNumPoints();
        
        double x0 = ancorPoint.getX();
        double y0 = ancorPoint.getY();
                        
        for (int i = 0; i < nrPoints; i++) {           
            x1 = coordList[i].x;
            y1 = coordList[i].y;

            xn = (x1 - x0)* Math.cos(angle) - Math.sin(angle)*(y1 - y0) + x0;
            yn = (x1 - x0)* Math.sin(angle) + Math.cos(angle)*(y1 - y0) + y0;
            
            coordList[i].x = xn;
            coordList[i].y = yn;            
        }
                         
            GeometryFactory myFac = new GeometryFactory();
            newGeom = myFac.createLinearRing(coordList);            
                                   
        return newGeom;
    }

    public static Point rotate(Point ancorPoint, double angle, Point myPoint){

        double x1, y1, xn ,yn;                
                
        double x0 = ancorPoint.getX();
        double y0 = ancorPoint.getY();
                        
        x1 = myPoint.getX();
        y1 = myPoint.getY();

        xn = (x1 - x0)* Math.cos(angle) - Math.sin(angle)*(y1 - y0) + x0;
        yn = (x1 - x0)* Math.sin(angle) + Math.cos(angle)*(y1 - y0) + y0;
        
        GeometryFactory myFac = new GeometryFactory();
        Coordinate myCoo = new Coordinate(xn, yn);
        Point newGeom = myFac.createPoint(myCoo);            
                                   
       return newGeom;
    }    
    
    public static void rotate(Coordinate ancorPoint, double angle, Coordinate myPoint){

        double x1, y1, xn ,yn;                
                
        double x0 = ancorPoint.x;
        double y0 = ancorPoint.y;
                        
        x1 = myPoint.x;
        y1 = myPoint.y;

        xn = (x1 - x0)* Math.cos(angle) - Math.sin(angle)*(y1 - y0) + x0;
        yn = (x1 - x0)* Math.sin(angle) + Math.cos(angle)*(y1 - y0) + y0;
        
        myPoint.x = xn;
        myPoint.y = yn;
    }    
    
    
    public static LineString rotateLS(Point ancorPoint, double angle, LineString myLineString){
        
        double x1, y1, xn ,yn;
        LineString newGeom;        
        
        Coordinate[] coordList = myLineString.getCoordinates();
        int nrPoints = myLineString.getNumPoints();
        
        double x0 = ancorPoint.getX();
        double y0 = ancorPoint.getY();
                        
        for (int i = 0; i < nrPoints; i++) {           
            x1 = coordList[i].x;
            y1 = coordList[i].y;

            xn = (x1 - x0)* Math.cos(angle) - Math.sin(angle)*(y1 - y0) + x0;
            yn = (x1 - x0)* Math.sin(angle) + Math.cos(angle)*(y1 - y0) + y0;
            
            coordList[i].x = xn;
            coordList[i].y = yn;            
        }
                         
            GeometryFactory myFac = new GeometryFactory();
            newGeom = myFac.createLineString(coordList);            
                                   
        return newGeom;
    }
    
}
