/***********************************************
 * created on 		27.10.2004
 * last modified: 	03.12.2004 (area only for exterior ring)
 * 
 * author:			sstein
 * 
 * description:
 *  Computes severity for area size (of exterrior ring).
 *  The areaSizes of holes can be obtained via measure field. 
 * 
 ***********************************************/
package mapgen.constraints.polygons;

import mapgen.constraints.MicroConstraint;
import mapgen.measures.JtsArea;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;

/**
 * @description:
 *  Computes severity for area size (of exterrior ring).
 *  The areaSizes of holes can be obtained via measure field. 
 * 
 * @author sstein 
 */
public class PolygonMinimalArea extends MicroConstraint{

    public JtsArea measure;
    private Geometry myGeometry;
    
    /**
     * sets the fields and calls computeConstraint()
     * @param geomObject musst be polygon
     * @param valueToReach
     * @param toleranceInPercent flexibility value .. has influence on isfullfilled field
     */
    public PolygonMinimalArea(Geometry geomObject,double valueToReach, double toleranceInPercent){
        this.myGeometry = geomObject;
        this.measure = new JtsArea((Polygon)this.myGeometry);
        this.setMeasureName("JtsArea");
        this.setConstraintName("MinArea");        
        
        this.setGoalValue(valueToReach);
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
        
        double[] area = this.measure.getPolyAreas();
        // use only area of exterior ring => index=0
        this.setCurrentValue(area[0]);
    }
    
    public void computeSeverity(){
        /***********************
         * matlab code: 
         * if (curValue < minValue) 
         *    severity(i) = 1;
         * elseif(curValue>= minValue)
         *    severity(i) = 0;
         * else
         *    severity(i) = abs(minValue-curValue)/(minValue/100*tolerance); 
         * end;
         * graph: 	1 ________
         * 				  	  \
         * 					   \
         * 			0			\_______
         *                   |  |min
         * 					 <tol> 
         ***************************/
        /**
        if (this.currentValue < this.goalValue){
            this.severity=1;
        }
        else if(this.currentValue >= this.goalValue){
            this.severity=0;
        }
        else{
            this.severity = Math.abs(this.goalValue-this.currentValue)/(this.goalValue/100*this.flexibility);             
        }
        **/
        //-- check if it is under tolerance value
        double toleranceValue = this.goalValue - (this.goalValue/100*this.flexibility);
        if (this.currentValue >= toleranceValue){
            this.setIsfullfilled(true);
        }
        else{
            this.setIsfullfilled(false);
        }
        //-- (full) linear gradient to goalValue
        if (this.currentValue >= this.goalValue){
            this.severity=0;
        }
        else {
            this.severity= 1 - (this.currentValue/this.goalValue) ;        	
        	/******
        	//-- or: severity gradient = x^5
            double s=this.currentValue/this.goalValue;
        	this.severity = 1- (s*s*s*s*s);
        	******/
        	}        
    }
}
