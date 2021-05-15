/***********************************************
 * created on 		27.10.2004
 * last modified: 	
 * 
 * author:			sstein
 * 
 * description:
 * 
 * 
 ***********************************************/
package mapgen.constraints;

/**
 * @author sstein
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public abstract class MicroConstraint extends Constraint{

    protected double goalValue;
         
    public double getGoalValue() {
        return this.goalValue;
    }
    
    public void setGoalValue(double goalValue) {
        this.goalValue = goalValue;
    }     
    
}
