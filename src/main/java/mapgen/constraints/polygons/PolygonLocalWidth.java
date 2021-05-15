/***********************************************
 * created on 		10.11.2004
 * last modified: 	
 * 
 * author:			sstein
 *   
 *  constrait by AGENT Project and Bader 1997,  
 *  
 *  ensures the readibility (min-distance) of narrow sections within a polygon outline 
 *  (and between the holes as well),
 * using MinWidthParts and the detected VertexEdgeConflicts
 * 
 ***********************************************/
package mapgen.constraints.polygons;

import mapgen.constraints.MicroConstraint;
import mapgen.measures.MinWidthParts;
import mapgen.measures.MinWidthPartsOutline;

import org.locationtech.jts.geom.Geometry;


/**
 * @description
 *  constrait by AGENT Project and Bader 1997,  
 *  
 *  ensures the readibility (min-distance) of narrow sections within a polygon outline 
 *  (and between the holes as well),
 *  using MinWidthParts and the detected VertexEdgeConflicts
 *
 * @author sstein
 *
 */
public class PolygonLocalWidth extends MicroConstraint{
    
    public MinWidthPartsOutline measure;
    private Geometry myGeometry;
    
    public PolygonLocalWidth(Geometry geomObject, double minDistance, double toleranceInPercent){ 
        this.myGeometry = geomObject;
        this.measure = new MinWidthPartsOutline(geomObject, minDistance);
        this.setMeasureName("MinWidthPartsOutline");
        this.setConstraintName("LocalWidth");
        
        this.setGoalValue(minDistance);
        this.setFlexibility(toleranceInPercent);         
        //this.setPriority(1);
        //this.setImportance(1);
        this.computeConstraint();
    }
    
    public void computeConstraint(){
        this.computeCurrentValue();
        this.computeSeverity();
    }
    
    protected void computeCurrentValue(){
        this.setCurrentValue(this.measure.getMinWidth());
    }
    
    /**
     * using MinWidthParts Measure and the found number of vertex-edge Conflicts
     * (here non consecutive points are to close) 
     *  severity is only 0(=good) or 1 (=bad) 
     */
    public void computeSeverity(){
        //-- calc isfullfilled with tolerance value
        double newDistance = this.goalValue - this.goalValue/100*this.flexibility;
        MinWidthParts flexibleMeasure = new MinWidthParts(this.myGeometry,newDistance);
        if (flexibleMeasure.getNrFoundVertexEdgeConflicts() > 0){
            this.setIsfullfilled(false);
        }
        else{
            this.setIsfullfilled(true);
        }
        //-- calc severity : only 1=bad or 0=good
        if (this.measure.getNrFoundVertexEdgeConflicts() > 0){
            this.severity = 1;
        }
        else{
            this.severity = 0;
        }


    }
}
