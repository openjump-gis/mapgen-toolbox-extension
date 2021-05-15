package mapgen.geomutilities;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;

/**
 * description
 *  calculates the distance from a point to a  edge (of two points) and 
 *  the closest point (either the perpendicular point or the end or start point).
 *  <p>
 *  algorithm from:
 *   http://www.faqs.org/faqs/graphics/algorithms-faq/
 *
 * created on 		14.12.2004
 * last modified: 	15.12.2004 (coorected: sign error for r)
 * @author sstein
 */
public class PointLineDistance{

    private Point resPoint = null;
    private double distance = 0;
    private boolean pointOnLine = true;
    
    public PointLineDistance(Coordinate p, Coordinate A, Coordinate B){
        
        this.pointOnLine = true;
        this.distancePointLinePerpendicular(p,A,B);
        if (!this.pointOnLine){
            this.calcWithStartEndPoint(p,A,B);
        }
        
    }
    
    private void distancePointLinePerpendicular(Coordinate p, Coordinate A, Coordinate B)
    {
      // use comp.graphics.algorithms Frequently Asked Questions method
      /*(2)
                       (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
                  s = -----------------------------
                                       L^2

                  Then the distance from C to P = |s|*L.
          */

      double Lsquared = (B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y);
      
      double r = ((p.x - A.x) *(B.x - A.x) + (p.y - A.y)*(B.y - A.y) ) / Lsquared; 
              
      // s to indicate the location along pc
      double s = ((A.y - p.y) *(B.x - A.x) - (A.x - p.x)*(B.y - A.y) ) / Lsquared;

      if ( (r >0) && (r < 1)){
          this.pointOnLine = true;
          
          double dist = Math.abs(s) * Math.sqrt(Lsquared);      
          this.distance = dist;
          
          double x = A.x + r * (B.x - A.x);
          double y = A.y + r * (B.y - A.y);
          
          GeometryFactory gf = new GeometryFactory();
          Point resultPoint = gf.createPoint(new Coordinate(x,y));
          this.resPoint = resultPoint;
          }
      else{
          this.pointOnLine = false;
      }      
    }
    
    private void calcWithStartEndPoint(Coordinate p, Coordinate A, Coordinate B){
        
        double dxA = p.x - A.x;
        double dyA = p.y - A.y;
        double sA = Math.sqrt(dxA*dxA + dyA*dyA);
        
        double dxB = p.x - B.x;
        double dyB = p.y - B.y;
        double sB = Math.sqrt(dxB*dxB + dyB*dyB);
        
        GeometryFactory gf = new GeometryFactory();
        if(sB < sA){
            this.distance = sB;
            this.resPoint = gf.createPoint((Coordinate)B.clone());             
        }
        else{
            this.distance = sA;
            this.resPoint = gf.createPoint((Coordinate)A.clone());                         
        }
    }   
    
    public double getDistance() {
        return distance;
    }
    
    public Point getClosestPoint() {
        return resPoint;
    }
    

    public boolean isPointOnLine() {
        return pointOnLine;
    }
}
