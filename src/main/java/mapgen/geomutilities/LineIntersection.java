package mapgen.geomutilities;

import java.awt.geom.Point2D;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineSegment;

/**
 * description:
 *  calculates the intersection point of two lines.<p> 
 *  algorithm from:
 *   http://www.faqs.org/faqs/graphics/algorithms-faq/
 *
 * created on 		14.09.2004
 * @author ingo
 */
public class LineIntersection {

    private final Coordinate  coord;

    // r and s will be calculated on demand
    private final double      rNumerator;
    private final double      rDenominator;
    private final double      sNumerator;
    private final double      sDenominator;

    public LineIntersection(Coordinate coordIn, double rNumeratorIn, double rDenominatorIn, double sNumeratorIn, double sDenominatorIn){
        coord           = coordIn;
        rNumerator      = rNumeratorIn;
        rDenominator    = rDenominatorIn;
        sNumerator      = sNumeratorIn;
        sDenominator    = sDenominatorIn;
    }
    
    /**
     * the intersection-point must not lie on one of the both described line-segments!
     *  
     * @return  intersection-coordinate if exist otherwise null
     */
    public Coordinate getCoordinate(){
        return coord;
    }
    
    /**
     * the intersection-point must not lie on one of the both described line-segments!
     *  
     * @return  intersection-coordinate if exist otherwise null
     */
    public Point2D getPoint2D(){
        return new Point2D.Double(coord.x, coord.y);
    }
    
    /**
     * @return true if an intersection on the segment or the line exists otherwise false
     */
    public boolean intersectionOnStraightLine(){
        return coord != null;
    }

    /**
     * @return true if an intersection on the segment or the line does not(!) exists otherwise false
     */
    public boolean noIntersectionOnStraightLine(){
        return (!intersectionOnStraightLine());
    }
    
    /*
     *     If 0<=r<=1 & 0<=s<=1, intersection exists
     *         r<0 or r>1 or s<0 or s>1 line segments do not intersect
     */
    /**
     * @return true if an intersection on the segment exists otherwise false
     * (also if intersection is on the straight line)
     */
    public boolean intersectionOnSegment(){
        if ( (coord != null) && (rDenominator != 0.0) ){
            double r = rNumerator/rDenominator;
            double s = sNumerator/sDenominator;
            if (  (r >= 0.0) && (r <= 1.0) && (s >= 0.0) && (s <= 1.0) ) {
                return true;
            }
        }
        return false;
    }

    /**
     * @return true if an intersection on the segment does not exist otherwise false
     */
    public boolean noIntersectionOnSegment(){
        return (!intersectionOnSegment());
    }

    /**
     * If the denominator in eqn 1 (in r) is zero, AB & CD are parallel
     * @return true if the segments/lines AB and CD are parallel
     */
    public boolean parallel(){
        if (rDenominator == 0.0) {
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * If the numerator in eqn 1 (in r) is also zero, AB & CD are collinear.
     * @return true if the segments/lines AB and CD are collinear
     */
    public boolean collinear(){
        if ( (rNumerator == 0.0) && (rDenominator == 0.0) ){
            return true;
        } else {
            return false;
        }
    }

    /**
     * If r>1, P is located on extension of AB
     * @return true if IntersectionPoint is located on the extension of AB; otherswise false (also if parallel etc)
     */
    public boolean extensionAB(){
        if (intersectionOnStraightLine() &&  (rDenominator != 0.0) && ((rNumerator / rDenominator) > 1.0) ){
            return true;
        }
        return false;
    }

    /**
     * If r<0, P is located on extension of BA
     * @return true if IntersectionPoint is located on the extension of BA; otherswise false (also if parallel etc)
     */
    public boolean extensionBA(){
        if (intersectionOnStraightLine() && (rDenominator != 0.0) && ((rNumerator / rDenominator) < 0.0) ){
            return true;
        }
        return false;
    }

    /**
     * If s>1, P is located on extension of CD
     * @return true if IntersectionPoint is located on the extension of CD; otherswise false (also if parallel etc)
     */
    public boolean extensionCD(){
        if (intersectionOnStraightLine() && (sDenominator != 0.0) && ((sNumerator / sDenominator) > 1.0) ){
            return true;
        }
        return false;
    }

    /**
     * If s<0, P is located on extension of DC
     * @return true if IntersectionPoint is located on the extension of DC; otherswise false (also if parallel etc)
     */
    public boolean extensionDC(){
        if (intersectionOnStraightLine() && (sDenominator != 0.0) && ((sNumerator / sDenominator) < 0.0) ){
            return true;
        }
        return false;
    }


    public static LineIntersection intersectionPoint(Point2D a, Point2D b, Point2D c, Point2D d) throws Exception {
        return intersectionPoint(new Coordinate(a.getX(), a.getY()), new Coordinate(b.getX(), b.getY()), new Coordinate(c.getX(), c.getY()), new Coordinate(d.getX(), d.getY()) );
    }

    public static LineIntersection intersectionPoint(LineSegment a, LineSegment b) throws Exception {
        return intersectionPoint( a.getCoordinate(0), a.getCoordinate(1), b.getCoordinate(0), b.getCoordinate(1) );
    }

    public static LineIntersection intersectionPoint(LineSegment a, Coordinate p0, Coordinate p1) throws Exception {
        return intersectionPoint( a.getCoordinate(0), a.getCoordinate(1), p0, p1 );
    }

    /**
     * This method is based on the "comp.graphics.algorithms Frequently Asked Questions" @link http://www.faqs.org/faqs/graphics/algorithms-faq/
     * "Subject 1.03: How do I find intersections of 2 2D line segments?"
     * The comments are from the "comp.graphics.algorithms Frequently Asked Questions"
     * @return the LineIntersection
     */
    public static LineIntersection intersectionPoint(Coordinate a, Coordinate b, Coordinate c, Coordinate d) throws Exception {
        LineIntersection res = null;
        Coordinate       p   = null;
//Out.println(Out.GEOM, "a: "+a);
//Out.println(Out.GEOM, "b: "+b);
//Out.println(Out.GEOM, "c: "+c);
//Out.println(Out.GEOM, "d: "+d);
        double     r            = 0.0;
        double     rNumerator   = 0.0;
        double     rDenominator = 0.0; 
        double     s            = 0.0;
        double     sNumerator   = 0.0;
        double     sDenominator = 0.0; 

        /*
         *     Let A,B,C,D be 2-space position vectors.  Then the directed line
         *     segments AB & CD are given by:
         * 
         *         AB=A+r(B-A), r in [0,1]
         *         CD=C+s(D-C), s in [0,1]
         * 
         *     If AB & CD intersect, then
         * 
         *         A+r(B-A)=C+s(D-C), or
         * 
         *         Ax+r(Bx-Ax)=Cx+s(Dx-Cx)
         *         Ay+r(By-Ay)=Cy+s(Dy-Cy)  for some r,s in [0,1]
         * 
         *     Solving the above for r and s yields
         * 
         *             (Ay-Cy)(Dx-Cx)-(Ax-Cx)(Dy-Cy)
         *         r = -----------------------------  (eqn 1)
         *             (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)
         * 
         *             (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
         *         s = -----------------------------  (eqn 2)
         *             (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)
         */
        try {
            rNumerator      = (a.y-c.y)*(d.x-c.x)-(a.x-c.x)*(d.y-c.y);
            rDenominator    = (b.x-a.x)*(d.y-c.y)-(b.y-a.y)*(d.x-c.x);
            
            sNumerator      = (a.y-c.y)*(b.x-a.x)-(a.x-c.x)*(b.y-a.y);
            sDenominator    = (b.x-a.x)*(d.y-c.y)-(b.y-a.y)*(d.x-c.x);
        } catch (Exception e){
//            throw ExceptionList.passException(e, "Error calculating numerator and denominator of r and s.");
        }

        // mal schaun ob parallel oder collinear
        /*
         *         If the denominator in eqn 1 (in r) is zero, AB & CD are parallel
         *         If the numerator in eqn 1 (in r) is also zero, AB & CD are collinear.
         */
        if (rDenominator == 0.0){
            // parallel
            return new LineIntersection(null, rNumerator, rDenominator, sNumerator, sDenominator);
        }

        if ( (rNumerator == 0.0) && (rDenominator == 0.0) ){
            // collinear
            return new LineIntersection(null, rNumerator, rDenominator, sNumerator, sDenominator);
        }

        try {
            r = rNumerator / rDenominator;
            s = sNumerator / sDenominator;
        } catch (Exception e) {
//            throw ExceptionList.passException(e, "Error calculating r and s.");
        }

        // intersection point
        /*
         *     Let P be the position vector of the intersection point, then
         *   
         *         P=A+r(B-A) or
         *   
         *         Px=Ax+r(Bx-Ax)
         *         Py=Ay+r(By-Ay)
         */
        try {
            p = new Coordinate();
            p.x = a.x + r*(b.x-a.x);
            p.y = a.y + r*(b.y-a.y);
        } catch (Exception e) {
//            throw ExceptionList.passException(e, "Error  calculating intersection point.");
        }

        return new LineIntersection(p, rNumerator, rDenominator, sNumerator, sDenominator);
    }
    
    public String toString(){
        String res = "";
        if (coord != null){
            res += "coordinate: "+coord + ", ";
        } else {
            res += "no interection, ";
        }
        if (collinear()){
            res += "is collinear, ";
        } else {
            res += "is not collinear, ";
        }
        if (parallel()){
            res += "is parallel";
        } else {
            res += "is not parallel";
        }
        return res;
    }

    public String toStringDetail(){
        String res = "";
        if (coord != null){
            res += "coordinate: "+coord + ", ";
        } else {
            res += "no interection, ";
        }
        if (collinear()){
            res += "is collinear, ";
        } else {
            res += "is not collinear, ";
        }
        if (parallel()){
            res += "is parallel, ";
        } else {
            res += "is not parallel, ";
        }
        res += "rDenominator: " + rDenominator +", ";
        res += "rNumerator: " + rNumerator +", ";
        res += "sDenominator: " + sDenominator +", ";
        res += "sNumerator: " + sNumerator +", ";
        if (intersectionOnSegment()){
            res += "is intersectionOnSegment, ";
        } else {
            res += "is not intersectionOnSegment, ";
        }
        if (intersectionOnStraightLine()){
            res += "is intersectionOnStraightLine, ";
        } else {
            res += "is not intersectionOnStraightLine, ";
        }
        if (extensionAB()){
            res += "is extensionAB, ";
        } else {
            res += "is not extensionAB, ";
        }
        if (extensionBA()){
            res += "is extensionBA, ";
        } else {
            res += "is not extensionBA, ";
        }
        if (extensionCD()){
            res += "is extensionCD, ";
        } else {
            res += "is not extensionCD, ";
        }
        if (extensionDC()){
            res += "is extensionDC, ";
        } else {
            res += "is not extensionDC, ";
        }
        return res;
    }

    /*
     * comp.graphics.algorithms Frequently Asked Questions:
     * 
     * Subject 1.03: How do I find intersections of 2 2D line segments?
     * 
     *     This problem can be extremely easy or extremely difficult; it
     *     depends on your application. If all you want is the intersection
     *     point, the following should work:
     * 
     *     Let A,B,C,D be 2-space position vectors.  Then the directed line
     *     segments AB & CD are given by:
     * 
     *         AB=A+r(B-A), r in [0,1]
     *         CD=C+s(D-C), s in [0,1]
     * 
     *     If AB & CD intersect, then
     * 
     *         A+r(B-A)=C+s(D-C), or
     * 
     *         Ax+r(Bx-Ax)=Cx+s(Dx-Cx)
     *         Ay+r(By-Ay)=Cy+s(Dy-Cy)  for some r,s in [0,1]
     * 
     *     Solving the above for r and s yields
     * 
     *             (Ay-Cy)(Dx-Cx)-(Ax-Cx)(Dy-Cy)
     *         r = -----------------------------  (eqn 1)
     *             (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)
     * 
     *             (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
     *         s = -----------------------------  (eqn 2)
     *             (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)
     * 
     *     Let P be the position vector of the intersection point, then
     * 
     *         P=A+r(B-A) or
     * 
     *         Px=Ax+r(Bx-Ax)
     *         Py=Ay+r(By-Ay)
     * 
     *     By examining the values of r & s, you can also determine some
     *     other limiting conditions:
     * 
     *         If 0<=r<=1 & 0<=s<=1, intersection exists
     *             r<0 or r>1 or s<0 or s>1 line segments do not intersect
     * 
     *         If the denominator in eqn 1 is zero, AB & CD are parallel
     *         If the numerator in eqn 1 is also zero, AB & CD are collinear.
     * 
     *     If they are collinear, then the segments may be projected to the x- 
     *     or y-axis, and overlap of the projected intervals checked.
     * 
     *     If the intersection point of the 2 lines are needed (lines in this
     *     context mean infinite lines) regardless whether the two line
     *     segments intersect, then
     * 
     *         If r>1, P is located on extension of AB
     *         If r<0, P is located on extension of BA
     *         If s>1, P is located on extension of CD
     *         If s<0, P is located on extension of DC
     * 
     *     Also note that the denominators of eqn 1 & 2 are identical.
     * 
     *     References:
     * 
     *     [O'Rourke (C)] pp. 249-51
     *     [Gems III] pp. 199-202 "Faster Line Segment Intersection,"
     */
    
}
