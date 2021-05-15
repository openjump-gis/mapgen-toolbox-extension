package mapgen.measures;

/**
 * Calculates different shape measures for a polygon
 *  input values have to be given
 *
 * created on 		23.09.2004
 * last modified: 	16.03.2005 (20.10.04 ShapeIndex measure was wrong computed)
 * 							   (02.12.04 change to static methodes and renaming)
 * 							   (16.03.05 new Schumm shape index)
 * @author sstein 
 */
public class PolygonShapeMeasures {
    
    /**
     * calculates shape Index 
     * @param perimeter
     * @param area
     * @return value >= 1 (for circle = 1)
     */
    public static double calcShapeIndex(double perimeter, double area){        
        double si = perimeter/(2*Math.sqrt(Math.PI*area));        
        return si;
    }
    
    /**
     * calculates the fractal dimension of a polygon 
     * @param perimeter
     * @param area
     * @return value between 1 (simple shape) and 2 (complex shape)
     */
    public static double calcFractalDim(double perimeter, double area){
        double fd =  (2* Math.log(perimeter))/Math.log(area);
        return fd;
    }
    
    /**
     * calculates the compactness of a polygon (comparable to shape index) 
     * @param perimeter
     * @param area
     * @return value between 0 and 1
     */
    public static double calcCompactness(double perimeter, double area){
        double com = (2*Math.sqrt(Math.PI*area))/perimeter;
        return com;
    }

    /**
     * calculates the elongation of a building or a polygon  
     * @param width
     * @param length
     * @return real value
     */
    public static double calcBuildingElongation(double width, double length){
        double el= width/length;
        return el;
    }
    
    /**
     * calculates the concavity of a building
     * @param area
     * @param convexHullArea
     * @return real value
     */
    public static double calcBuildingConcavity(double area, double convexHullArea){
        double con= area/convexHullArea;
        return con;
    }
    
    /**
     * calcualates a shape index proposed by Schumm (1956), see further MacEachren 1985
     * @param longestAxis (is equal to the diameter of a circumscribing circle)
     * @param area
     * @return shape index (normalized)
     */
    public static double calcSchummShapeIndex(double longestAxis, double area){       
        double si = Math.sqrt(area / Math.PI) / (0.5*longestAxis); 
        return si;
    }
}
