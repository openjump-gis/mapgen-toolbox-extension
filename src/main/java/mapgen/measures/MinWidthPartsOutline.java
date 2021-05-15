package mapgen.measures;

import java.util.List;
import java.util.ArrayList;

import mapgen.geomutilities.PointLineDistance;
import mapgen.measures.supportclasses.MWConflictList;
import mapgen.measures.supportclasses.MinWidthConflict;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;


/**
 * 
 * Calculates the minimal distance between two non adjacent edges<p>
 *		of a polygon shape / LineString <p>
 *		calculation among point and edge<p>
 * 		(algorithm by Mats Bader, AGENT Project)<p>
 * preliminary condition: <p>
 * 		a) no self intersection of polygon / LineString<p>
 * 		b) polygon / LineString needs more than four vertices<p>
 * <p>
 * zero distances will not be detected (adjacent edges)
 * <p> 
 * to get the conflicts use getMwcList()
 *
 * created on 		17.12.2004
 * last modified: 	20.12.2004 extended by save conflicts as MWConflictList
 * @author sstein
 */
public class MinWidthPartsOutline {

    private double threshold = 0;
    private double nrFoundVerticies = 0;
    private double nrOfChecks = 0;
//    private ArrayList minWidthDoubleList = new ArrayList();
    private final List<LineString> dispVecPointEdgeLStringList = new ArrayList<>();
    private final List<LineString> dispVecPointPointLStringList = new ArrayList<>();
    private final List<LineString> minEdgeLineStringList = new ArrayList<>();
    private final List<Point> minVertexPointList = new ArrayList<>();
    private boolean outlineConflict = false;
    //-------
    private double minWidth = 999999;
    private final int[] minWidthPtIndex = new int[3];
    private LineString minWidthDispPointVector = null;
    private boolean minWidthIsPointEdgeConfl = true;
    private final MWConflictList mwclist = new MWConflictList();
    
    /**
     * 
     * @param geom LineString or Polygon
     * @param minDistance in m
     */
    public MinWidthPartsOutline(Geometry geom, double minDistance){
        
        this.setThreshold(minDistance);
        
        if(geom instanceof Polygon){
            Polygon myPolygon = (Polygon)geom;
            //condition: more than five points = more than 4 edges
            if(myPolygon.getNumPoints() > 4 ){
            	this.calcMinWidthOutline(myPolygon, threshold);
            }
        }
        else if(geom instanceof LineString){
                LineString myLine = (LineString)geom;
                //condition: more than three points = more than 2 edges
                if(myLine.getNumPoints() > 3 ){
                	this.calcMinWidthOutline(myLine, threshold);
                }
        }
        else{
            System.out.println("MinWidthPartsOutline: Input Object no Polygon or LineString");
        }                
        
    }
    
    /**
     * 
     * @param myPolygon a Polygon
     * @param myThreshold the threshold
     */
    private void calcMinWidthOutline(Polygon myPolygon, double myThreshold){
        // condition : more than 4 vertices is true 
        LineString outerRing = myPolygon.getExteriorRing();
        // test outer ring with it self
        this.outlineConflict = this.selfTest(outerRing);
     
    } //end function

    private void calcMinWidthOutline(LineString myLine, double myThreshold){
        // condition b: more than 3 vertices is true          
        // test LineString with it self
        this.outlineConflict = this.selfTest(myLine);
     
    } //end function
    
    /**
     * Test distance of points from one Linestring to edges from self LineString
     * but not for the nearest 2 edges
     * @param Ring
     */
	private boolean selfTest(LineString Ring){
	    
	    boolean conflict = false;
	    // the ring has to have more than 3 points (= at least 3 edges)
	    if(Ring.getNumPoints() > 3){	        
	        GeometryFactory myGF = new GeometryFactory();
	        Coordinate[] edge = new Coordinate[2];
	        LineString tempLine;
	        double dist;
	        
	        // ==== loop for over all points from Ring =====
	        // -1; otherwise last point will be tested twice
	        for (int i = 0; i < Ring.getNumPoints()-1 ; i++) {
	            Point myPoint = Ring.getPointN(i);
	            					// -1; since edge(j,j+1) 	            
	            for (int j = 0; j < Ring.getNumPoints()-1; j++){
	                //reduce calculations concerning adjacent edges
	                if( // don't check the next 2 edges
	                   (Math.abs(j-i) >= 2) &&
	                   // don't check the previous two edges
	                   (Math.abs(j+1-i) >= 2) &&
	                   // if base point i = 0, don't check last two edges
	                   ((i!=0) || (j<(Ring.getNumPoints()-3)))&& //if both = false => false
	                   // if base point i = 1, don't check last edge
	                   ((i!=1) || (j<(Ring.getNumPoints()-2)))&& //if both = false => false
	                   // if point i = last-1 is base, don't check the first edge
	                   ((i!=Ring.getNumPoints()-2) || (j > 0))
	                   ){
	                    //--- old
	                    edge[0] = Ring.getCoordinateN(j);
	                    edge[1] = Ring.getCoordinateN(j+1);
	                    tempLine = myGF.createLineString(edge);
	                    //-- new
	                    Coordinate A = Ring.getCoordinateN(j);
	                    Coordinate B = Ring.getCoordinateN(j+1);
	                    Coordinate P = myPoint.getCoordinate();
	                    PointLineDistance pld = new PointLineDistance(P,A,B);
	                    dist = pld.getDistance();
	                    boolean isPerpendicularPoint = pld.isPointOnLine();
	                    Coordinate rp = pld.getClosestPoint().getCoordinate();
	                    this.nrOfChecks = this.nrOfChecks +1;
		                if (	(dist != 0) && 
		                        (dist < this.getThreshold())   
		                    ){
		                    conflict = true;
		                    //========== calc disp vector and save as linestring =======
		                    double dxAct,dyAct, dxRef, dyRef, dxMove, dyMove, xNew, yNew;
		                    dxAct = P.x - rp.x; 
		                    dyAct = P.y - rp.y;    
		                    dxRef = dxAct * this.threshold / dist;
		                    dyRef = dyAct * this.threshold / dist;
		                    dxMove = dxRef - dxAct;
		                    dyMove = dyRef - dyAct;
		                    xNew = myPoint.getX() + dxMove;
		                    yNew = myPoint.getY() + dyMove;
		                    
		                    Coordinate[] dispVectorCoord = new Coordinate[2];		                    ;
		                    dispVectorCoord[0] = myPoint.getCoordinate();
		                    dispVectorCoord[1] = new Coordinate(xNew, yNew);		                    
		                    LineString dispVector = myGF.createLineString(dispVectorCoord);
		                    //======= save ======
		                    Double Ddist = dist;
		                    //this.minWidthDoubleList.add(Ddist);
		                    // if not using object.clone the list links point on the
		                    // the last checked point and checked edge
		                    this.minEdgeLineStringList.add((LineString)tempLine.copy());
		                    this.minVertexPointList.add((Point)myPoint.copy());
		                    // ===== save to conflict list ======/
		                    MinWidthConflict mwc = new MinWidthConflict();
		                    mwc.ptHoleIdx = 0;
		                    mwc.ptIdx = i;
		                    mwc.edgeHoleIdx = 0;
		                    mwc.edgeStartPtIdx = j;
		                    mwc.edgeEndPtIdx = j+1;
		                    mwc.distance = dist;
		                    mwc.ptDispDx = dxMove;
		                    mwc.ptDispDy = dyMove;
		                    mwc.ptEdgeConflict = isPerpendicularPoint; 
		                    mwclist.add(mwc);
		                    
		                    // ============
		                    if(isPerpendicularPoint){
		                        this.dispVecPointEdgeLStringList.add((LineString)dispVector.copy());
		                    }
		                    else{
		                        this.dispVecPointPointLStringList.add((LineString)dispVector.copy());
		                    }
		                    //--- save for minimal distance 
		                    if(dist < this.minWidth){
		                        this.minWidth = dist;
		                        this.minWidthDispPointVector = (LineString)dispVector.copy();
		                        this.minWidthPtIndex[0] = i; //point
		                        this.minWidthPtIndex[1] = j; // start edge point 
		                        this.minWidthPtIndex[2] = j+1; // end edge point
		                        this.minWidthIsPointEdgeConfl = isPerpendicularPoint;
		                    }
 
		                } // end if dist < threshold

	                }//end if
	            }  // end for edges
	        } // end for test point
	    } //end if
	    return conflict;
	}
	
	/********************* getters and setters **************/
	/**
	 * 
	 * @return the number of all found conflicts:
	 * 			point <=> edge & point <=> node
	 * 			calculatet from minVertexPointList 
	 */			 
    public double getNrFoundVerticies() {
        this.nrFoundVerticies = this.minVertexPointList.size();
        return nrFoundVerticies;
    }
	
    /**
	 * 
	 * @return the number of found conflict:
	 * 				point <=> edge 
	 * 			calculatet from minEdgeLineStringList 
	 */			 
    public double getNrFoundVertexEdgeConflicts() {        
        return this.dispVecPointEdgeLStringList.size();
    }
	
    /**
	 * 
	 * @return the number of found conflict:
	 * 			 	point <=> node
	 * 			calculatet from minVertexPointList
	 */			 
    public double getNrFoundVertexPointConflicts() {        
        return this.dispVecPointPointLStringList.size();
    }
    
    public double getThreshold() {
        return threshold;
    }
   
    private void setThreshold(double threshold) {
        this.threshold = threshold;
    }
    /**
     * 
     * @return list with edges as LineStrings which are to close to
     * 		   points obtained from getMinVertexPointList 

     */
    public List getMinEdgeLineStringList() {
        return minEdgeLineStringList;
    }
    /**
     * 
     * @return list with verticies (saved as Point objects) which are 
     * 			to close to edges obtained from getMinEdgeLineStringList 
     */
    public List getMinVertexPointList() {
        return minVertexPointList;
    }
    
    /**
     * 
     * @return list with displacement vectors among
     * 		    point and edge node saved as LineString 
     */
    public List getDispVecPointPointLStringList() {
        return dispVecPointPointLStringList;
    }
    
    /**
     * 
     * @return list with displacement vectors among
     * 		    point and perpendicular point on edge 
     * 			saved as LineString 
     */
    public List getDispVecPointEdgeLStringList() {
        return dispVecPointEdgeLStringList;
    }
        
    public boolean hasOutlineConflict() {
        return this.outlineConflict;
    }    

    /**
    public double[] getMinWidths(){
        
        double[] width = new double[this.minWidthDoubleList.size()];
        int i=0;
        Double value;
        for (Iterator iter = minWidthDoubleList.iterator(); iter.hasNext();) {
            value = (Double) iter.next();
            width[i] = value.doubleValue();
            i = i+1;
        }
       return width;
    }
    **/
    
    /**
     * returns the number of checks made
     * and is equal to max number of to small distances
     * @return double/int value 
     */
    public double getNrOfChecks() {
        return this.nrOfChecks;
    }

    /*************** new 17.12.2004 ***************/
    /**
     * 
     * @return distance of the strongest conflict,
     * where distance is bigger than zero
     */
    public double getMinWidth() {
        return minWidth;
    }
    
    /** 
     * @return the displacement vector for the strongest
     * conflict
     */
    public LineString getMinWidthDispPointVector() {
        return minWidthDispPointVector;
    }
    
    /**
     * @return the indizes of the points which have the
     * strongest outline conflict. The first value is the
     * single point, the second index is the edge start point
     * the third index is the edge end point
     */
    public int[] getMinWidthPtIndex() {
        return minWidthPtIndex;
    }
    
    public boolean isMinWidthPointEdgeConflict() {
        return minWidthIsPointEdgeConfl;
    }
    
    /**
     * 
     * @return a special List type of the conflicts as 
     * MinWidthConflict type 
     */
    public MWConflictList getMwclist() {
        return mwclist;
    }
}