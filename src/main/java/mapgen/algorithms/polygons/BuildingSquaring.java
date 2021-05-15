package mapgen.algorithms.polygons;

import java.util.ArrayList;
import java.util.List;

import mapgen.algorithms.Rotate;
import mapgen.geomutilities.FirstGeodeticTask2d;
import mapgen.geomutilities.LineIntersection;
import mapgen.geomutilities.ModifyPolygonPoints;
import mapgen.geomutilities.SecondGeodeticTask2d;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

/**
 * description:
 *  Squares the walls of a building polygon. Not every edge
 *  is squared (depending on the thresholds). <p>
 *  Method does not support polygon holes, they are kept as they are.<p>
 *	Algorithm proposed by Mats Bader and A. Edwardes 
 *  (Agent Delivery D1) but is modified. 
 *
 * TODO: Michael Michaud has provided some sample buildings (see JPP bug report #145 SquareBuildingPlugIn of MapGen Toolbox)
 * with situations that the algorithm does not handle yet. This needs to be addressed at some point.
 *
 * created on 		12.01.2005
 * @author sstein
 *
 */
public class BuildingSquaring {
    
    private Polygon inPolygon;
    private Polygon outPolygon = null;
    private double angleThreshold = 5; // in degree
    private double angleMaxValue = 30; 
    private double maxPointDisplacement = 5; // in m
    //-- the directions and lists are valid only for the
    //   actual polygon ring
    private double mainDirection = 0; // in rad
    private double secondDirection = 0;
    //-- the arrayLists contain the index of the starting point 
    private final List<Integer> mainDirectionEdges = new ArrayList<>();
    private final List<Integer> secondDirectionEdges = new ArrayList<>();
    private final List<Integer> otherDirectionEdges = new ArrayList<>();
    //private ArrayList intersectionPoints = new ArrayList();    
    //private ArrayList rotEdges = new ArrayList();
    
    /**
     * initialises and calls calculate()
     * @param poly building geometry
     * @param maxPointDisplacement maximum distance a point
     * 		  is allowed to move in m
     * @param angleThreshold in degree. Maximum value where an edge 
     * 	      direction different from primary or secondary
     *        edge direction is considered as the same direction.		  
     * 		  Value must be smaller than 45 degree. 
     */    
    public BuildingSquaring(Polygon poly, double maxPointDisplacement, double angleThreshold){
        this.inPolygon = poly;
        this.angleThreshold = angleThreshold;
        this.maxPointDisplacement = maxPointDisplacement;
        this.calculate();        
    }

    /**
     * initialises and calls calculate()
     * @param poly building geometry
     * @param angleThreshold in degree. Maximum value where an edge 
     * 	      direction different from primary or secondary
     *        edge direction is considered as the same direction.		  
     * 		  Value must be smaller than 45 degree. 
     */    
    public BuildingSquaring(Polygon poly, double angleThreshold){
        this.inPolygon = poly;
        this.angleThreshold = angleThreshold;
        this.maxPointDisplacement = 999999.0;
        this.calculate();        
    }

    /**
     * function which does the squaring and the necessary
     * calculations for the Polygon
     */    
    public void calculate(){
        this.outPolygon = (Polygon)this.inPolygon.copy();
        //-- check if threshold < 45 degree
        if (this.angleThreshold > 45){
            this.angleThreshold = this.angleMaxValue;
            System.out.println("BuildingSquaring.calculate: reset angleThreshold to maxValue: " + this.angleMaxValue);
        }
        //-- square outer ring
        LineString exteriorRing = this.outPolygon.getExteriorRing();
        this.calculateForOneRing(exteriorRing,0);
        //--square holes        
        int nrRings = this.outPolygon.getNumInteriorRing();
        if (nrRings > 0){
            for (int i = 0; i < nrRings; i++) {
                LineString interiorRing = this.outPolygon.getInteriorRingN(i); 
                this.calculateForOneRing(interiorRing,i+1);
            }
        }
    }
    
    /**
     * function which does the squaring and the necessary
     * calculations for one Polygon Ring
     * @param ls = polygon ring to square
     * @param  ringIndex = nr of ring (exteriorRing = 0, innerRings = idx+1)
     * 					   for homogenisation in modifyFollwingEdges();
     */
    private void calculateForOneRing(LineString ls, int ringIndex){
        //-- clear lists
        this.mainDirectionEdges.clear();
        this.secondDirectionEdges.clear();
        this.otherDirectionEdges.clear();
        
        this.findMainDirections(ls);
        this.sortEdges(ls);       
        //--delete following edges
        if(ls.getNumPoints() > 5){
	        boolean edgesDeleted = this.modifyFollowingEdges(ringIndex,this.mainDirectionEdges);
	        if(edgesDeleted == true){            
	            if(ringIndex == 0){
	                ls = this.outPolygon.getExteriorRing();
	            }
	            else{
	                ls = this.outPolygon.getInteriorRingN(ringIndex-1);
	            }
	            //-- clear lists
	            this.mainDirectionEdges.clear();
	            this.secondDirectionEdges.clear();
	            this.otherDirectionEdges.clear();
	            
	            this.findMainDirections(ls);
	            this.sortEdges(ls);                   
	        }
	        edgesDeleted = this.modifyFollowingEdges(ringIndex, this.secondDirectionEdges);
	        if(edgesDeleted == true){            
	            if(ringIndex == 0){
	                ls = this.outPolygon.getExteriorRing();
	            }
	            else{
	                ls = this.outPolygon.getInteriorRingN(ringIndex-1);
	            }
	            //-- clear lists
	            this.mainDirectionEdges.clear();
	            this.secondDirectionEdges.clear();
	            this.otherDirectionEdges.clear();
	            
	            this.findMainDirections(ls);
	            this.sortEdges(ls);                   
	        }
        } //end if -- delete following points
        //---
        this.square(this.mainDirectionEdges, this.mainDirection,ls);
        this.square(this.secondDirectionEdges, this.secondDirection,ls);
    }
    
    /**
     * calculate the main direction using the longest edge of the polygon from
     * the polygon exterior ring.
     *
     */
    private void findMainDirections(LineString ls){      
        Coordinate[] coords = ls.getCoordinates(); 
        double maxLength = 0;
        double mainAngle = 0;
        double length;
        for (int i = 0; i < coords.length-1; i++) { 
            length = SecondGeodeticTask2d.calcDistanceCoord(coords[i],coords[i+1]);
            if(length > maxLength){
                maxLength = length;
                mainAngle = SecondGeodeticTask2d.calcAngle2Coords(coords[i],coords[i+1]);
            }
        }
        //-- angle direction can have a value from 0 to pi (not 2*pi)
        if (mainAngle > Math.PI){
            mainAngle = mainAngle - Math.PI;
        }
        this.mainDirection = mainAngle;
        //-- calc second angle
        if (mainAngle > Math.PI/2.0){
            this.secondDirection = mainAngle - Math.PI/2.0;
        }
        else{
            this.secondDirection = mainAngle +  Math.PI/2.0;
        }
    }
    
    /**
     * sort the edges into the 3 lists:
     * mainDirectionEdges, secondDirectionEdges, otherDirectionEdges.
     * Edges put in otherDirectionEdges are not squared.
     * 
     */
    private void sortEdges(LineString ls){       
        Coordinate[] coords = ls.getCoordinates();       
        double angle, deltaA, deltaB;
        for (int i = 0; i < coords.length-1; i++) { 
            angle = SecondGeodeticTask2d.calcAngle2Coords(coords[i],coords[i+1]);
            //-- angle direction can have a value from 0..pi (not 2*pi)
            if (angle > Math.PI){
                angle = angle - Math.PI;
            }
            //-- get positiv angle difference
            if (this.mainDirection > angle){
            	deltaA = this.mainDirection - angle;
            }
            else{
            	deltaA = angle - this.mainDirection;
            }
            if (deltaA > Math.PI/2.0){
            	deltaA = Math.PI - deltaA;
            }
            //---
            if (this.secondDirection > angle){
            	deltaB = this.secondDirection - angle;
            }
            else{
            	deltaB = angle - this.secondDirection;
            }
            if (deltaB > Math.PI/2.0){
            	deltaB = Math.PI - deltaB;
            }            
            //---
            if(deltaA < (this.angleThreshold * Math.PI /180)){
                this.mainDirectionEdges.add(i);
            }
            else if(deltaB < (this.angleThreshold * Math.PI /180)){
                this.secondDirectionEdges.add(i);
            }
            else{
                this.otherDirectionEdges.add(i);
            }
        }        
    }
    
    /**
     * square edges by rotating the edge around its center 
     * point and calculate the new corner coordinates of 
     * the intersecting lines 
     * @param edgeIndexList
     * @param newDirection
     */
    private void square(List edgeIndexList, double newDirection, LineString ls){
        //-- square all edges in list
        for (int i = 0; i < edgeIndexList.size(); i++) {
            LineString ring2 = ls; 
            int nrPoints2 = ring2.getNumPoints();
            Coordinate vstart, vsstart, vend, veend, vstartOld, vendOld;
            Integer value = (Integer)edgeIndexList.get(i);
            int idx = value;
            vstart = ring2.getCoordinateN(idx);
            vstartOld = (Coordinate)vstart.clone();
            vend = ring2.getCoordinateN(idx+1);
            vendOld = (Coordinate)vend.clone();
            //next points needed for intersection
            if (idx == 0){
                vsstart = ring2.getCoordinateN(nrPoints2-2);
            }
            else{
                vsstart = ring2.getCoordinateN(idx-1);
            }
            if ((idx+1) == nrPoints2-1){
                veend = ring2.getCoordinateN(1);
            }
            else{
                veend = ring2.getCoordinateN(idx+1+1);
            }
            //--------------------------------------
            // rotate edge points into maindirection
            //--------------------------------------
            double oldDirection = SecondGeodeticTask2d.calcAngle2Coords(vstart,vend);
            double length = SecondGeodeticTask2d.calcDistanceCoord(vstart,vend);
            //-- calc center point for rotation before change of angle            
            Coordinate center = FirstGeodeticTask2d.getCoordinate(vstart,oldDirection,(length/2.0));
            //-- angle direction can have a value from 0..pi (not 2*pi)
            if (oldDirection > Math.PI){
                oldDirection = oldDirection - Math.PI;
            }
            double delta = oldDirection - newDirection;
            
            Rotate.rotate(center,-1*delta,vstart);            
            Rotate.rotate(center,-1*delta, vend);
            /*
            Coordinate[] cs = {vstart, vend};
            LineString ls = new GeometryFactory().createLineString(cs);
            this.rotEdges.add((LineString)ls.clone());
            */
            //-------------------------------------------
            // calculate definitv new position for vstart 
            // and vend by line intersection
            //-------------------------------------------
            try{
                // the coordinates have to be set separate,
                // otherwise the assignment to the polygon vertex fails
                LineIntersection lp = LineIntersection.intersectionPoint(vstart,vend,vstartOld,vsstart);
                vstart.x = lp.getCoordinate().x;
                vstart.y = lp.getCoordinate().y;
                LineIntersection ln = LineIntersection.intersectionPoint(vstart,vend,vendOld,veend);
                vend.x = ln.getCoordinate().x;
                vend.y = ln.getCoordinate().y;
                // --------------------------------------
                // check if points aren't displace so much
                // --------------------------------------
                double dist1 = SecondGeodeticTask2d.calcDistanceCoord(vstart,vstartOld);
                double dist2 = SecondGeodeticTask2d.calcDistanceCoord(vend,vendOld);
                if ((dist1 > this.maxPointDisplacement) || dist2 > this.maxPointDisplacement){
                  //-- reset points
                  System.out.println("BuildingSquaring: Points displaced to much, reseting coords");
                  vstart.x = vstartOld.x; vstart.y = vstartOld.y;
                  vend.x = vendOld.x; vend.y = vendOld.y;
                }
                //---
                /*
                Point p1 = new GeometryFactory().createPoint(vstart);
                this.intersectionPoints.add(p1);                        
                Point p2 = new GeometryFactory().createPoint(vend);
                this.intersectionPoints.add(p2);
                */
                //----------------------------------------
                // if startpointIdx = 0 then last point of ring
                //    has to be set as well and vice versa
                //----------------------------------------
	            if (idx == 0){
	                ring2.getCoordinateN(nrPoints2-1).x = vstart.x;
	                ring2.getCoordinateN(nrPoints2-1).y = vstart.y;    	                
	            }
	            if ((idx+1) == nrPoints2-1){
	                ring2.getCoordinateN(0).x = vend.x;
	                ring2.getCoordinateN(0).y = vend.y;    	                
	            }                    
	            
            }
            catch(Exception e){
                System.out.println("BuildingSquaring: line intersection could not be calculated");
            }
        }
    }
    
    /**
     * deletes the middle point if two following edges are found in the list 
     * @param ringIndex : in which Ring is the point to delete
     * 					 for ExteriorRing : ringIndex=0; 
     */
    private boolean modifyFollowingEdges(int ringIndex, List edgeList){
        int noDeletedPoints = 0;
    
        if (edgeList.size() > 0){
	        //-- search Following points in List
	        Integer idx = (Integer)edgeList.get(0);
	        int previousIdx = idx;
	        for (int i=0; i < edgeList.size(); i++) {
	            idx = (Integer)edgeList.get(i);            
	            if (idx.intValue() == (previousIdx+1)){
	                //--delete in Polygon        
	                this.outPolygon = ModifyPolygonPoints.deletePoint(this.outPolygon,
	                        			ringIndex,(idx.intValue()- noDeletedPoints));
	                noDeletedPoints = noDeletedPoints+1;	                
	                //-- and do not add to list
	            }
	            previousIdx = idx.intValue();
	        }
        }
        boolean edgesDeleted = false;
        if (noDeletedPoints > 0){
            edgesDeleted = true;
            };
        return edgesDeleted;
    }
    /******************** getters and setters ***************/
    
    public Polygon getInPolygon() {
        return inPolygon;
    }
    public void setInPolygon(Polygon inPolygon) {
        this.inPolygon = inPolygon;
    }
    /**
     * Retrieve the Squaring Result.
     * Note, the resulting geometry needs to be checked for validity (using geom.isValid()).<p>
     * Also a test of the polygon/building area before and after squaring helps to identify
     * problematic squaring results (e.g. when line intersections could not be calculated). 
     * @return
     */
    public Polygon getOutPolygon() {
        return outPolygon;
    }
    public double getAngleThreshold() {
        return angleThreshold;
    }
    /**
     * 
     * @param angleThreshold in degree
     */
    public void setAngleThreshold(double angleThreshold) {
        this.angleThreshold = angleThreshold;
    }
    public double getMaxPointDisplacement() {
        return maxPointDisplacement;
    }
    public void setMaxPointDisplacement(double maxPointDisplacement) {
        this.maxPointDisplacement = maxPointDisplacement;
    }
    /**
     * the primary direction is only valid for the actual/last 
     * polygon ring/hole. The secondary direction is perpendicular
     * to this direction.
     * @return in radian
     */
    public double getMainDirection() {
        return mainDirection;
    }    
    /*
    public ArrayList getIntersectionPoints() {
        return intersectionPoints;
    }    
    public ArrayList getRotEdges() {
        return rotEdges;
    }
    */
    public double getAngleMaxValue() {
        return angleMaxValue;
    }
    public void setAngleMaxValue(double angleMaxValue) {
        this.angleMaxValue = angleMaxValue;
    }
}	
