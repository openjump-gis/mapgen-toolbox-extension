/***********************************************
 * created on 		07.07.2005
 * last modified: 	
 * 
 * author:			sstein
 *   
 *  constrait by AGENT Project
 *  
 *  ensures that not invisible short edges appear using 
 *  ShortestEdge measure
 * 
 ***********************************************/
package mapgen.constraints.buildings;

import mapgen.constraints.MicroConstraint;
import mapgen.measures.ShortestEdge;

import org.locationtech.jts.geom.Geometry;

/**
 * @description
 *  constrait by AGENT Project  
 *
 * @author sstein
 *
 */
public class BuildingShortestEdge extends MicroConstraint{
    
    public ShortestEdge measure;
    private Geometry myGeometry;
    
    public BuildingShortestEdge(Geometry geomObject, double minDistance, double flexibilityInPercent){ 
        this.myGeometry = geomObject;
        this.measure = new ShortestEdge(geomObject, minDistance);
        this.setMeasureName("ShortestEdge");
        this.setConstraintName("BShortestEdge");
        
        this.setGoalValue(minDistance);
        this.setFlexibility(flexibilityInPercent);         
        //this.setPriority(1);
        //this.setImportance(1);
        this.computeConstraint();
    }
    
    public void computeConstraint(){
        this.computeCurrentValue();
        this.computeSeverity();
    }
    
    protected void computeCurrentValue(){
        this.setCurrentValue(this.measure.getNrOfToShortEdges());
    }
    
    /**
     * using ShortestEdge Measure and the found number of to short edges 
     *  severity is ratio = conflict edges / tested edges : 0(=good) or 1 (=bad)  
     */
    public void computeSeverity(){
        //-- calc isfullfilled with tolerance value
        double newDistance = this.goalValue - this.goalValue/100*this.flexibility;
        ShortestEdge flexibleMeasure = new ShortestEdge(this.myGeometry,newDistance);
        if (flexibleMeasure.getNrOfToShortEdges() > 0){
            this.setIsfullfilled(false);
        }
        else{
            this.setIsfullfilled(true);
        }
        //-- calc severity : ratio conflict edges / all edges 
        //					1=bad or 0=good
        if (this.measure.getNrOfToShortEdges() > 0){
            this.severity = this.measure.getNrOfToShortEdges() / this.measure.getNrOfTestedEdges();
        }
        else{
            this.severity = 0;
        }


    }
}
