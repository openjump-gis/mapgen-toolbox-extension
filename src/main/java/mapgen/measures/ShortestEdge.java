package mapgen.measures;

import java.util.ArrayList;
import java.util.List;

import mapgen.measures.supportclasses.ShortEdgeConflict;
import mapgen.measures.supportclasses.ShortEdgeConflictList;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

/**
 * Calculates to short edges of a polygon (for outerRing and holes)
 *  or lineStrings and saves them in a List of LineStrings  and in a 
 *  list with special type ShortEdgeConflict. <p> 
 *  The lists can be obtained using getXXX() methods
 *
 * created on 		28.09.2004
 * last modified: 	07.07.2005 (ShortEdgeConflictList)
 * @author sstein 
 */
public class ShortestEdge {
    
    private final List<LineString> LineStringList = new ArrayList<>();
    //private LineString shortestEdge = null;
    private double length = 0;
    private double nrOfToShortEdges = 0;
    private double nrOfTestedEdges = 0;
    private final ShortEdgeConflictList confList = new ShortEdgeConflictList();
    private boolean isConflict = false;

    /**
     * constructor 1
     * @param myGeometry input Geometry
     * @param threshold threshold
     */
    public ShortestEdge(Geometry myGeometry, double threshold){
        
        if (myGeometry instanceof Polygon){
            Polygon myPolygon = (Polygon)myGeometry;
            // calc smallest edge for ExteriorRing
            LineString myOuterRing = myPolygon.getExteriorRing();
            this.calcShortestEdge(myOuterRing, threshold, 0);
            // calc smallest edge for InterriorRings
            int nrIntRings = myPolygon.getNumInteriorRing();
            if (nrIntRings > 0){
                for (int i = 0; i < nrIntRings; i++) {
                    LineString myInnerRing = myPolygon.getInteriorRingN(i);               
                    this.calcShortestEdge(myInnerRing, threshold, i+1);
                }
            }
        }
        else{
            // zeugs fuer point oder sowas  
        }                
    }

    /**
     * constructor 2
     * @param myLine a LineString
     * @param threshold threshold
     */
    public ShortestEdge(LineString myLine, double threshold){
        this.calcShortestEdge(myLine,threshold, 0);
    }
    
    /**
     * calculates to short edges of a LineString and saves them in
     * a List of LineStrings
     * the list can be recieved from outside using getLineStringList()
     *   
     * @param myLine a LineString
     * @param myThreshold the threshold
     * @param noOfRing number of rings
     */
    private void calcShortestEdge(LineString myLine, double myThreshold, int noOfRing){
              
        //double nrPoints = myLine.getNumPoints();
        Coordinate[] myCoordinates = myLine.getCoordinates();
        //init minimum length
        double smallestLength = myThreshold;
        
        GeometryFactory myGF = new GeometryFactory();        
        double count = 0;
        double s,x1,y1,x2,y2,dx,dy;
        //double xt1=0, yt1=0, xt2=0, yt2=0;
        // calculate lengths of segments and compare with threshold
        for (int i = 0; i < myCoordinates.length-1; i++) {
            this.nrOfTestedEdges = this.nrOfTestedEdges +1;
            x1 = myCoordinates[i].x;
            y1 = myCoordinates[i].y;
            x2 = myCoordinates[i+1].x;
            y2 = myCoordinates[i+1].y;
            dx = x2 - x1;
            dy = y2 - y1;
            s = Math.sqrt(dx*dx + dy*dy);
            if (s < smallestLength){
                smallestLength = s;
                //xt1=x1; xt2=x2; yt1=y1; yt2=y2;
            }
            // ---- new part : returning a list ----------------
            if (s < myThreshold){
                count = count+1;
                ShortEdgeConflict sec = new ShortEdgeConflict();
                sec.edgeRingIdx = noOfRing;
                sec.edgeStartPtIdx=i;
                sec.edgeEndPtIdx=i+1;
                sec.edgeLineIdx=i;
                sec.length=s;
                confList.add(sec);
                //--
                Coordinate[] edge = new Coordinate[]{
                		new Coordinate(x1,y1),
                		new Coordinate(x2,y2)
						};                
                LineString mySmallestEdge = myGF.createLineString(edge);
                LineStringList.add((LineString)mySmallestEdge.copy());
            }            
        } //end for           
        // return smallest length and nr of edges 
        this.setLength(smallestLength);
        double oldValue = this.nrOfToShortEdges; //since we also check rings
        this.setNrOfToShortEdges(oldValue + count);
        // ---- end new part -------------------
        /******* old part - return smallest edge ********************        
        // return smallest edge and length
        if (count > 0){
            int alreadyFound = this.getNrOfToShortEdges();
            // if no edge has found previously (=> from exterior or other
            // interior rings, the founded edge will be set
            if (alreadyFound == 0 ){                
                Coordinate[] edge = new Coordinate[]{
                        		new Coordinate(xt1,yt1),
                        		new Coordinate(xt2,yt2)
        						};
                GeometryFactory myGF = new GeometryFactory();
                LineString mySmallestEdge = myGF.createLineString(edge);
                this.setShortestEdge(mySmallestEdge);
                this.setLength(smallestLength);
                this.setNrOfToShortEdges(count);
            }
            else{ 
                // if the edge saved in alreadyFound is longer
                // then save new one
                if (this.getLength() > smallestLength){
                    Coordinate[] edge = new Coordinate[]{
                    		new Coordinate(xt1,yt1),
                    		new Coordinate(xt2,yt2)
    						};
                    GeometryFactory myGF = new GeometryFactory();
                    LineString mySmallestEdge = myGF.createLineString(edge);
                    this.setShortestEdge(mySmallestEdge);
                    this.setLength(smallestLength);
                    this.setNrOfToShortEdges(count + alreadyFound);                    
                }
                
            }
        }
        *********** end old part *********************************/        
    }


    /**
     * @return smallest length
     */
    public double getLength() {
        return length;
    }    

    private void setLength(double length) {
        this.length = length;
    }

    public double getNrOfToShortEdges() {
        return nrOfToShortEdges;
    }

    private void setNrOfToShortEdges(double nrToShortEdges) {
        this.nrOfToShortEdges = nrToShortEdges;
    }

    /**
     * 
     * @return a List of to short Elements (saved as LineStrings)
     */
    public List<LineString> getLineStringList() {
        return LineStringList;
    }

    public double getNrOfTestedEdges() {
        return this.nrOfTestedEdges;
    }
	/**
	 * @return Returns a List of conflicts with elements of type ShortEdgeConflict.
	 */
	public ShortEdgeConflictList getConflicList() {
		return confList;
	}
	/**
	 * @return Returns the hasConlficts.
	 */
	public boolean hasConflicts() {
	  this.isConflict = this.nrOfToShortEdges > 0;
		return this.isConflict;
	}
}
