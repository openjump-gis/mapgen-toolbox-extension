/***********************************************
 * created on 		02.12.2004
 * last modified: 	
 * 
 * @description:
 *  Constraint proposed by the agent project(Delivery A3). 
 * 	Tests the squareness of the Polygon or LineString.
 *  The severity value is equal to the squareness value,
 *  where the squareness is the angle deviation.
 *  The tolorerance value defines a maximal deviation and has
 *  influence on the "isfullfilled" field but not on the constraint
 *  severity. 
 * 
 * author:			sstein
 *
 ***********************************************/
package mapgen.constraints.buildings;

import mapgen.constraints.MicroConstraint;
import mapgen.measures.Squareness;
import org.locationtech.jts.geom.Geometry;

/**
 * @description:
 *  Constraint proposed by the agent project(Delivery A3).
 * 	Tests the squareness of the Polygon or LineString.
 *  The severity value is equal to the squareness value,
 *  where the squareness is the angle deviation.
 *  The tolorerance value defines a maximal deviation and has
 *  influence on the "isfullfilled" field but not on the constraint
 *  severity. 
 * 
 * @author sstein 
 */
public class BuildingSquareness extends MicroConstraint{
    
    public Squareness measure;
    private Geometry myGeometry;
    
    /**
     * sets the fields and calls computeConstraint()
     * @param geomObject either Polygon or LineString
     * @param tolerance : flexibility value (as angle deviation value! in radian), 
     * 		  has influence on isfullfilled field
     */
    public BuildingSquareness(Geometry geomObject, double tolerance){ 
        this.myGeometry = geomObject;
        this.measure = new Squareness(geomObject);
        this.setMeasureName("Squareness");
        this.setConstraintName("BuildSquarness");
        
        this.setGoalValue(0.0);
        this.setFlexibility(tolerance);         
        //this.setPriority(1);
        //this.setImportance(1);
        this.computeConstraint();
    }
    
    public void computeConstraint(){
        this.computeCurrentValue();
        this.computeSeverity();
    }
    
    protected void computeCurrentValue(){
        this.setCurrentValue(this.measure.getSquareness());
    }
   
    public void computeSeverity(){
        //-- calc isfullfilled with tolerance value
        if (this.currentValue <= this.flexibility){
            this.setIsfullfilled(true);
            this.severity = 0;
        }
        else{
            //-- if squareness is bigger flexibility value 
            //    use current value directly
            this.setIsfullfilled(false);
            this.severity = this.currentValue;
        } 
    }
}
