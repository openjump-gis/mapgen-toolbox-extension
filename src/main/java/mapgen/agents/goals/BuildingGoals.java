package mapgen.agents.goals;

/**
 * 
 * description
 *  constraint values for buildings<p>
 * 	- priority<p>
 * 	- importance<p>
 * 	- goal value (no ending  = mm, ending "Real" = in m)<p>
 * 	- flexibility ( in percent, metres, degree, or as deviation)<p>
 *
 * created on 		21.12.2004
 * last modified: 	22.12.2004 (finalized)
 * @author sstein
 *
 */
public class BuildingGoals{

    private int scale;
    private final double factor;//from mm to m;
    
    public BuildingGoals(int scale){
        this.scale = scale;
        this.factor = this.scale/1000.0; //from mm to m;       
    }
    
    //****************************
    // offensive constraints
    //***************************
    
    //=== taken from SGK (swiss society for cartography)
    // minimum object separation
    private int minObjectSeparationPriority = 1; 
    private int minObjectSeparationWeight = 1;
    private double minObjectSeparation = 0.2; //mm
    private int minObjectSeparationFlexibility = 0; //in percent
    
    //=== taken from SGK (swiss society for cartography)
    // minimum distance of facing polygon sides
    private int minWidthPriority = 1; 
    private int minWidthWeight = 1;
    private double minWidth = 0.25; //mm
    private int minWidthFlexibility = 0; //in percent
    
    //=== taken from SGK (swiss society for cartography)
    // minimal Area
    private int minAreaPriority = 1; 
    private int minAreaWeight = 1;
    private double minArea = 0.35*0.35; //mm
    private int minAreaFlexibility = 10; //in percent
    
    //==== value from SGK (swiss society for cartography)
    // minimal polygon edge length of a building
    private int shortestEdgePriority = 1; 
    private int shortestEdgeWeight = 1;
    private double shortestEdge = 0.25; //mm
    private int shortestEdgeFlexibility = 0; //in percent
    
    //==== value from ???
    // minimal squareness of building corner
    private int squarenessPriority = 1; 
    private int squarenessWeight = 1;
    private double squareness = 0.0;     
    private double squarenessFlexibility = 0.1; //as deviation value in rad ???
    //private double squarenessMaxAngle = 15.0; // in degree

    //==== value from SGK 
    // buffer diameter
    private int symbolisePriority = 1; 
    private int symboliseWeight = 1;
    private double symbolise = 0.35; //in mm
    private double symboliseFlexibility = 0.0; //no flexibility    
    
    //****************************
    // defensive constraints
    //***************************
    
    //==== value from Bard 2004
    // defensive constraint
    // polygon elongation (shape value)
    private int elongationPriority = 1; 
    private int elongationWeight = 1;
    //private double elongation = 0.0; 
    //private double elongationReal = elongation;
    private int elongationFlexibility = 20; //in percent

    //==== value from Bard 2004
    // defensive constraint    
    // polygon concavity (shape value)
    private int concavityPriority = 1; 
    private int concavityWeight = 1;
    //private double concavity = 0.6; 	//???? minValue ??? or defending constraint?
    //private double concavityReal = concavity;
    private int concavityFlexibility = 10; //in percent
    
    //===== value from SGK (resolution of eye in 30 cm distance)
    // defensive constraint    
    // positionAccuracy
    private int positionPriority = 1; 
    private int positionWeight = 1;
    private double positionAccuracy = 0.2; //mm    
    
    //===== value from Bard 2004
    // defensive constraint
    // orientation
    private int orientationPriority = 1; 
    private int orientationWeight = 1;
    //private double orientation = 0.0; //degree
    //private double orientationReal = orientation;
    private double orientationFlexibility = 5.0; //in degree
    
    //======
    // defensive constraint 
    // self intersection
    private int selfIntersectionPriority = 1;
    private int selfIntersectionWeight = 1;
    
    /**************** getters and setters ******************/
    
    public int getConcavityFlexibility() {
        return concavityFlexibility;
    }
    public void setConcavityFlexibility(int concavityFlexibility) {
        this.concavityFlexibility = concavityFlexibility;
    }
    public int getConcavityPriority() {
        return concavityPriority;
    }
    public void setConcavityPriority(int concavityPriority) {
        this.concavityPriority = concavityPriority;
    }
    public int getConcavityWeight() {
        return concavityWeight;
    }
    public void setConcavityWeight(int concavityWeight) {
        this.concavityWeight = concavityWeight;
    }
    public int getElongationFlexibility() {
        return elongationFlexibility;
    }
    public void setElongationFlexibility(int elongationFlexibility) {
        this.elongationFlexibility = elongationFlexibility;
    }
    public int getElongationPriority() {
        return elongationPriority;
    }
    public void setElongationPriority(int elongationPriority) {
        this.elongationPriority = elongationPriority;
    }
    public int getElongationWeight() {
        return elongationWeight;
    }
    public void setElongationWeight(int elongationWeight) {
        this.elongationWeight = elongationWeight;
    }
    public double getMinArea() {
        return minArea;
    }
    public void setMinArea(double minArea) {
        this.minArea = minArea;
    }
    public int getMinAreaFlexibility() {
        return minAreaFlexibility;
    }
    public void setMinAreaFlexibility(int minAreaFlexibility) {
        this.minAreaFlexibility = minAreaFlexibility;
    }
    public int getMinAreaPriority() {
        return minAreaPriority;
    }
    public void setMinAreaPriority(int minAreaPriority) {
        this.minAreaPriority = minAreaPriority;
    }
    public int getMinAreaWeight() {
        return minAreaWeight;
    }
    public void setMinAreaWeight(int minAreaWeight) {
        this.minAreaWeight = minAreaWeight;
    }
    public double getMinObjectSeparation() {
        return minObjectSeparation;
    }
    public void setMinObjectSeparation(double minObjectSeparation) {
        this.minObjectSeparation = minObjectSeparation;
    }
    public int getMinObjectSeparationFlexibility() {
        return minObjectSeparationFlexibility;
    }
    public void setMinObjectSeparationFlexibility(
            int minObjectSeparationFlexibility) {
        this.minObjectSeparationFlexibility = minObjectSeparationFlexibility;
    }
    public int getMinObjectSeparationPriority() {
        return minObjectSeparationPriority;
    }
    public void setMinObjectSeparationPriority(int minObjectSeparationPriority) {
        this.minObjectSeparationPriority = minObjectSeparationPriority;
    }
    public int getMinObjectSeparationWeight() {
        return minObjectSeparationWeight;
    }
    public void setMinObjectSeparationWeight(int minObjectSeparationWeight) {
        this.minObjectSeparationWeight = minObjectSeparationWeight;
    }
    public double getMinWidth() {
        return minWidth;
    }
    public void setMinWidth(double minWidth) {
        this.minWidth = minWidth;
    }
    public int getMinWidthFlexibility() {
        return minWidthFlexibility;
    }
    public void setMinWidthFlexibility(int minWidthFlexibility) {
        this.minWidthFlexibility = minWidthFlexibility;
    }
    public int getMinWidthPriority() {
        return minWidthPriority;
    }
    public void setMinWidthPriority(int minWidthPriority) {
        this.minWidthPriority = minWidthPriority;
    }
    public int getMinWidthWeight() {
        return minWidthWeight;
    }
    public void setMinWidthWeight(int minWidthWeight) {
        this.minWidthWeight = minWidthWeight;
    }
    public double getOrientationFlexibility() {
        return orientationFlexibility;
    }
    public void setOrientationFlexibility(double orientationFlexibility) {
        this.orientationFlexibility = orientationFlexibility;
    }
    public int getOrientationPriority() {
        return orientationPriority;
    }
    public void setOrientationPriority(int orientationPriority) {
        this.orientationPriority = orientationPriority;
    }
    public int getOrientationWeight() {
        return orientationWeight;
    }
    public void setOrientationWeight(int orientationWeight) {
        this.orientationWeight = orientationWeight;
    }
    /**
     * 
     * @return accuracy in mm
     */
    public double getPositionAccuracy() {
        return positionAccuracy;
    }
    /**
     * 
     * @param positionAccuracy in mm
     */
    public void setPositionAccuracy(double positionAccuracy) {
        this.positionAccuracy = positionAccuracy;
    }
    /**
     * 
     * @return accuracy in m
     */
    public double getPositionAccuracyReal() {
        return this.factor * this.positionAccuracy;
    }
    public int getPositionAccPriority() {
        return positionPriority;
    }
    public void setPositionAccPriority(int positionPriority) {
        this.positionPriority = positionPriority;
    }
    public int getPositionAccWeight() {
        return positionWeight;
    }
    public void setPositionAccWeight(int positionWeight) {
        this.positionWeight = positionWeight;
    }
    public int getScale() {
        return scale;
    }
    public void setScale(int scale) {
        this.scale = scale;
    }
    public double getShortestEdge() {
        return shortestEdge;
    }
    public void setShortestEdge(double shortestEdge) {
        this.shortestEdge = shortestEdge;
    }
    public int getShortestEdgeFlexibility() {
        return shortestEdgeFlexibility;
    }
    public void setShortestEdgeFlexibility(int shortestEdgeFlexibility) {
        this.shortestEdgeFlexibility = shortestEdgeFlexibility;
    }
    public int getShortestEdgePriority() {
        return shortestEdgePriority;
    }
    public void setShortestEdgePriority(int shortestEdgePriority) {
        this.shortestEdgePriority = shortestEdgePriority;
    }
    public int getShortestEdgeWeight() {
        return shortestEdgeWeight;
    }
    public void setShortestEdgeWeight(int shortestEdgeWeight) {
        this.shortestEdgeWeight = shortestEdgeWeight;
    }
    public double getSquareness() {
        return squareness;
    }
    public void setSquareness(double squareness) {
        this.squareness = squareness;
    }
    public double getSquarenessFlexibility() {
        return squarenessFlexibility;
    }
    public void setSquarenessFlexibility(double squarenessFlexibility) {
        this.squarenessFlexibility = squarenessFlexibility;
    }
    public int getSquarenessPriority() {
        return squarenessPriority;
    }
    public void setSquarenessPriority(int squarenessPriority) {
        this.squarenessPriority = squarenessPriority;
    }
    public int getSquarenessWeight() {
        return squarenessWeight;
    }
    public void setSquarenessWeight(int squarenessWeight) {
        this.squarenessWeight = squarenessWeight;
    }
    public double getSymbolise() {
        return symbolise;
    }
    public void setSymbolise(double symbolise) {
        this.symbolise = symbolise;
    }
    public double getSymboliseFlexibility() {
        return symboliseFlexibility;
    }
    public void setSymboliseFlexibility(double symboliseFlexibility) {
        this.symboliseFlexibility = symboliseFlexibility;
    }
    public int getSymbolisePriority() {
        return symbolisePriority;
    }
    public void setSymbolisePriority(int symbolisePriority) {
        this.symbolisePriority = symbolisePriority;
    }
    public int getSymboliseWeight() {
        return symboliseWeight;
    }
    public void setSymboliseWeight(int symboliseWeight) {
        this.symboliseWeight = symboliseWeight;
    }
    /**
     * 
     * @return minArea in m^2
     */
    public double getMinAreaReal() {
        return (this.factor*this.factor * minArea);
    }
    /**
     * 
     * @return minDistance in m
     */
    public double getMinObjectSeparationReal() {
        return (this.factor * minObjectSeparation);
    }
    /**
     * 
     * @return minWidthParts in m
     */
    public double getMinWidthReal() {
        return (this.factor * minWidth);
    }
    /**
     * 
     * @return minLength in m
     */
    public double getShortestEdgeReal() {
        return (this.factor * this.shortestEdge);
    }
    /**
     * 
     * @return squareness as deviation
     */
    public double getSquarenessReal() {
        return this.squareness;
    }
    /**
     * 
     * @return buffer diameter in m
     */
    public double getSymboliseReal() {
        return (this.factor * this.symbolise);
    }
    
    public int getSelfIntersectionPriority() {
        return selfIntersectionPriority;
    }
    public void setSelfIntersectionPriority(int selfIntersectionPriority) {
        this.selfIntersectionPriority = selfIntersectionPriority;
    }
    public int getSelfIntersectionWeight() {
        return selfIntersectionWeight;
    }
    public void setSelfIntersectionWeight(int selfIntersectionWeight) {
        this.selfIntersectionWeight = selfIntersectionWeight;
    }
	/**
	 * @return Returns the factor.
	 */
	public double getFactor() {
		return factor;
	}
}
