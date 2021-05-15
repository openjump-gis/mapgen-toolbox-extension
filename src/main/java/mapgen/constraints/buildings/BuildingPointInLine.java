/***********************************************
 * created on 		01.12.2005
 * last modified: 	
 * 
 * author:			sstein
 *    
 *  
 *  ensures that no points on a line appear which cause an
 *  invisible difference in wall direction
 *  primary search/filter value is maxAngleDiff of wall direction 
 ***********************************************/
package mapgen.constraints.buildings;

import mapgen.constraints.MicroConstraint;
import mapgen.measures.PointOnLine;

import org.locationtech.jts.geom.Geometry;

/**
 * @description
 *  constrait by AGENT Project  
 *
 * @author sstein
 *
 */
public class BuildingPointInLine extends MicroConstraint{
    
    public PointOnLine measure;
    private Geometry myGeometry;
    
    public BuildingPointInLine(Geometry geomObject, double minDistance, double maxAngleDiff){ 
        this.myGeometry = geomObject;
        this.measure = new PointOnLine(geomObject, minDistance,maxAngleDiff);
        this.setMeasureName("PointOnLine");
        this.setConstraintName("BPointOnLine");
        
        this.setGoalValue(minDistance);
        this.setFlexibility(0);         
        //this.setPriority(1);
        //this.setImportance(1);
        this.computeConstraint();
    }
    
    public void computeConstraint(){
        this.computeCurrentValue();
        this.computeSeverity();
    }
    
    protected void computeCurrentValue(){
        this.setCurrentValue(this.measure.getNrOfPointsInLine());
    }
    
    /**
     * using ShortestEdge Measure and the found number of to short edges 
     *  severity is ratio = conflict edges / tested edges : 0(=good) or 1 (=bad)  
     */
    public void computeSeverity(){
        //-- calc severity : ratio conflict points / all points 
        //					1=bad or 0=good
        if (this.measure.getNrOfPointsInLine() > 0){
            this.severity = this.measure.getNrOfPointsInLine() / this.measure.getTestedPoints();
        }
        else{
            this.severity = 0;
        }


    }
}
