package mapgen.constraints;

/**
 * abstract constraint class
 * subclasses: MicroConstraint, MesoConstraint
 *
 * created on 		27.10.2004
 * @author sstein
 */

public abstract class Constraint {
    
    //-------------------------------------
    // fileds    
    protected double currentValue;
    protected double severity;
    protected boolean fullfilled;
    protected double priority;
    protected double weight;
    protected double flexibility;
    protected String measureName="";
    protected String constraintName="";
    
    //-------------------------------------
    // functions
    /**
     * computes Current Value and Severity,
     * called on creation of object, 
     * calls computeSeverity() and computeCurrentValue()  
     */
    public abstract void computeConstraint();
    
    /**
     * computes severity value,
     * is public to recalc if ne flexibility or new goalValue is set, 
     * internaly called by computeConstraint() on creation of object
     */
    public abstract void computeSeverity();
   
    /**
     * computes the current value, 
     * internaly called by computeConstraint() on creation of object, 
     */
    protected abstract void computeCurrentValue();
   
    
    //------------------------------------
    // getters and setters
    /**
     * 
     * @return flexibility value in percent
     */
    public double getFlexibility() {
        return flexibility;
    }
    /**
     * 
     * @param flexibility value in percent
     */
    public void setFlexibility(double flexibility) {
        this.flexibility = flexibility;
    }
    /**
     * 
     * @return priority value for order of execution
     * 		   from 1..5 (1:highest 3: middel  5: lowest)
     */
    public double getPriority() {
        return priority;
    }
    /**
     * 
     * @param priority value for order of execution
     * 		   from 1..5 (1:highest 3: middel  5: lowest)
     */
    public void setPriority(double priority) {
        this.priority = priority;
    }
    public double getCurrentValue() {
        return currentValue;
    }
    public double getSeverity() {
        return severity;
    }    
    protected void setCurrentValue(double currentValue) {
        this.currentValue = currentValue;
    }
    protected void setSeverity(double severity) {
        this.severity = severity;
    }
    public String getMeasureName() {
        return measureName;
    }
    protected void setMeasureName(String measureName) {
        this.measureName = measureName;
    }
    /**
     * 
     * @return weight for calculation of agent satisfaction
     * 		   from 1..5 (1:highest 3: middel  5: lowest)
     */
    public double getWeight() {
        return weight;
    }    
    /**
     * 
     * @param  weight for calculation of agent satisfaction
     * 		   from 1..5 (1:highest 3: middel  5: lowest)
     */
    public void setWeight(double weight) {
        this.weight = weight;
    }
    /**
     * 
     * @return boolean value = true if constraint is fullfilled
     * status depends on the tolerance value 
     */
    public boolean isfullfilled() {
        return fullfilled;
    }
    /**
     * 
     * @param isfullfilled -- set true if constraint is fullfilled
     * status depends on the tolerance value 
     */
    protected void setIsfullfilled(boolean isfullfilled) {
        this.fullfilled = isfullfilled;
    }
    public String getConstraintName() {
        return constraintName;
    }
    public void setConstraintName(String constraintName) {
        this.constraintName = constraintName;
    }
}
