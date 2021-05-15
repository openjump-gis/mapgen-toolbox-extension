/***********************************************
 * created on 		19.11.2004
 * last modified: 	
 * 
 * author:			sstein
 * 
 * description:
 * 
 * 
 ***********************************************/
package mapgen.geomutilities;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;

/** 
 * description:
 *  calculates point with a given angle and distance
 *  angle turns from east=0 over north=pi/2 to 2*pi (=east again)
 * 
 * @author sstein
 *
 */
public class FirstGeodeticTask2d {

    /**
     * calculates point with a given angle and distance
     * angle turns from east=0 over north=pi/2 to 2*pi (=east again)
     *  
     * @param basePoint
     * @param angle
     * @param distance
     * @return
     */
    public static Point getPoint(Point basePoint, double angle, double distance){
        
        double deast = distance * Math.cos(angle);
        double dnorth = distance * Math.sin(angle);
        
        double x = basePoint.getX() + deast;
        double y = basePoint.getY() + dnorth;
        
        GeometryFactory myGF = new GeometryFactory();
        
        Point resultPoint = myGF.createPoint(new Coordinate(x,y));
        return resultPoint;
        
    }

    /**
     * calculates point with a given angle and distance
     * angle turns from east=0 over north=pi/2 to 2*pi (=east again)
     *  
     * @param basePoint
     * @param angle
     * @param distance
     * @return
     */
    public static Coordinate getCoordinate(Coordinate basePoint, double angle, double distance){
        
        double deast = distance * Math.cos(angle);
        double dnorth = distance * Math.sin(angle);
        
        double x = basePoint.x + deast;
        double y = basePoint.y + dnorth;
                
        Coordinate resultPoint = new Coordinate(x,y);
        return resultPoint;        
    }
    
}
