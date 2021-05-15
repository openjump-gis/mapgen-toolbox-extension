/***********************************************
 * created on 		08.12.2004 (structure)
 * last modified: 	17.12.2004 (final)
 * 					19.05.2005 (change: check minPoints for segments and Lines)
 *					21.05.2005 (split long lines)
 *					21.07.2005 (random split position)  					
 * 
 * author:			sstein
 * 
 * description:
 *  Smoothes a given line using a snakes algorithm. 
 *  This can be done segmentwise if break points will be given.
 * 	The class uses class snakesSmoothingSegment
 * 
 * TODO: interpolation of equidistant vertices with distance of maximal point displacement
 ***********************************************/
package mapgen.algorithms.snakes;

import mapgen.geomutilities.SplitLineString;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;

/**
 * @description:
 *  Smoothes a given line using a snakes algorithm. 
 *  This can be done segmentwise if break points will be given.
 * 	The class uses class snakesSmoothingSegment
 * 
 * @author sstein 
 * 
 */
public class SnakesSmoothingLineNew {
    
    private boolean doSegmentation = false;
    private double alpha = 1;
    private double beta = 1; 
    private int iterations = 1;
    private int[] segmentationIndizes = null;
    private LineString inputLine = null;
    private LineString smoothedLine = null;
    private LineString[] segmentList = null;
    private int minPoints = 5;		//--necessary points to calculate a smoothing (at least 5 points are necessary
    								//  for snakes, but since points are mirrored it can be also only 3)
    private int maxPoints = 50;	//used for segmentation, which is necessary to avoid too big matrices
    private int segmentPointIdx = 0; 
    private double maxDistTreshold = 0;
    private boolean couldNotSmooth = false;
    
    // -------------- constructors : begin ---------------------- 
    /**
     * Smoothes a line with the given parameters. Either as complete line
     * or segmentwise if a list of cut point indizes is given.
     * @param line to smooth
     * @param iterations
     * @param alpha snakes parameter (should be one)
     * @param beta snakes parameter (should be one)
     * @param doSegmentation boolean value
     * @param segmentationIndizes list of point indizes there the line has to be cutted
     *                            without first and last line point
     */
    public SnakesSmoothingLineNew(LineString line, int iterations, 
                                    double alpha, double beta, 
                                    boolean doSegmentation, int[] segmentationIndizes){
    	
        this.inputLine = line;
        this.iterations = iterations;
        this.alpha = alpha;
        this.beta = beta;        
        this.doSegmentation = doSegmentation;
        this.segmentationIndizes = segmentationIndizes;
        if ((segmentationIndizes == null) || (segmentationIndizes.length == 0)){
            this.doSegmentation = false;
        }        
        this.solveIter();        
    }

    /**
     * Smoothes a line with the given parameters.    
     * @param line to smooth
     * @param iterations
     * @param alpha snakes parameter (should be one)
     * @param beta snakes parameter (should be one)
     */
    public SnakesSmoothingLineNew(LineString line, int iterations, 
            double alpha, double beta){
        
        this.inputLine = line;
        this.iterations = iterations;
        this.alpha = alpha;
        this.beta = beta;        
        this.doSegmentation = false;
        
        this.solveIter();
}

    /**
     * Smoothes a line with the given parameters. Either as complete line
     * or segmentwise if a list of cut point indizes is given.
     * @param line to smooth
     * @param maxDist distance of point displacement
     * @param doSegmentation boolean value
     * @param segmentationIndizes list of point indizes there the line has to be cutted
     *                            without first and last line point
     */
    public SnakesSmoothingLineNew(LineString line, double maxDist, 
                                    double alpha, double beta, 
                                    boolean doSegmentation, int[] segmentationIndizes){
    	
        this.inputLine = line;         
        this.maxDistTreshold = maxDist;
        this.iterations = 1;
        this.alpha = alpha;
        this.beta = beta;        
        this.doSegmentation = doSegmentation;
        this.segmentationIndizes = segmentationIndizes;
        if ((segmentationIndizes == null) || (segmentationIndizes.length == 0)){
            this.doSegmentation = false;
        }        
        this.solveMaxDisp();        
    }

    /**
     * Smoothes a line with the given parameters.    
     * @param line to smooth
     * @param maxDist maximum distance of point displacement
     */
    public SnakesSmoothingLineNew(LineString line, double maxDist,double alpha, double beta){
        
        this.inputLine = line;
        this.maxDistTreshold = maxDist;        
        this.alpha = alpha;
        this.beta = beta;        
        this.doSegmentation = false;
        
        this.solveMaxDisp();
}
    
    // -------------- constructors : end ----------------------
    
    /**
     * used for iterative solution by a given maximum number of iteration<p> 
     * decides between the cases of segmentation and no segmentation. <p>
     * calls smoothLine() .. which itself calls class SnakesSmoothingSegment 
     */
    private void solveIter(){
        System.out.println("SnakesSmoothingsLine.solveIter: border correction in SnakesSmoothSegment() only for end points");
    	//------------------
    	// with segmentation 
    	//------------------ 
        if(this.doSegmentation == true){
            this.segmentList = SplitLineString.splitInSegments(this.inputLine,this.segmentationIndizes);            
            int n = this.segmentationIndizes.length +1;
            LineString[] smoothedSegments = new LineString[n];
            //-- smooth the single segments
            for (int i = 0; i < n; i++) {             
                LineString lineSegment = this.segmentList[i];
                //--check nr of points first (min. is 3 according to SnakesSmoothingSegment)
                //  and not bigger than maxPoints to avoid big matrices
                if ((lineSegment.getNumPoints() >= this.minPoints)&& 
                		(lineSegment.getNumPoints() <= this.maxPoints)){
                    LineString newSegment = this.smoothLine(lineSegment);
                    smoothedSegments[i] = (LineString)newSegment.copy();
                }
                else if (lineSegment.getNumPoints() < this.minPoints){ 
                	//--if to short, use orginal points
                    smoothedSegments[i] = (LineString)lineSegment.copy();
                }
    	        else if(lineSegment.getNumPoints() > this.maxPoints){
    	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Do further segmentation since segment has to much vertices!!!");
    	        	//-- get segmentation points
    	        	int[] pointidx = this.getSegmentationPointsForLongLines(lineSegment);
    	        	//-- do smoothing by use of the class itself .. use one iteration instaed maxDist threshold 
    	        	SnakesSmoothingLineNew ssl = new SnakesSmoothingLineNew(lineSegment, 1, 
    	        									this.alpha, this.beta, true, pointidx);
    	        	smoothedSegments[i] = ssl.getSmoothedLine();
    	        }
    	        else{
    	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Unhandled case for this number of vertices!!!");
    	        }
            }
            this.smoothedLine = SplitLineString.concatSegements(smoothedSegments);
        }
    	//------------------
    	// no segmentation 
    	//------------------        	
        else{
            System.out.println("no of points: " + this.inputLine.getNumPoints());
            if((this.inputLine.getNumPoints() >= this.minPoints)  &&
    				(this.inputLine.getNumPoints() <= this.maxPoints)){
                this.smoothedLine = this.smoothLine(this.inputLine);
            }
	        else if (this.inputLine.getNumPoints() < this.minPoints){ 
	        	//-- if to short, use orginal points
                this.smoothedLine = this.inputLine;
                System.out.println("SnakesSmoothingLineNew.solveMaxDisp: line to short to smooth!");
	        }
	        else if(this.inputLine.getNumPoints() > this.maxPoints){
	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Do segmentation since line is has to much vertices!!!");
	        	//-- get segmentation points
	        	int[] pointidx = this.getSegmentationPointsForLongLines(this.inputLine);
	        	//-- do smoothing by use of the class itself
	        	SnakesSmoothingLineNew ssl = new SnakesSmoothingLineNew(this.inputLine, 1 , 
	        									this.alpha, this.beta, true, pointidx);
	        	this.smoothedLine = ssl.getSmoothedLine();
	        }
	        else{
	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Unhandled case for this number of vertices!!!");
	        }
        }
    }

    /**
     * used for smoothing with maximum displacement of points..<p> 
     * decides between the cases of segmentation and no segmentation. <p>
     * calls smoothLineMaxDisp() .. if segments are smoothed<p>
     * long lines are segmentated to avoid big matrices (see field maxPoints) 
     */
    private void solveMaxDisp(){
        System.out.println("SnakesSmoothingsLine.solvemaxDisp: border correction in SnakesSmoothSegment() only for end points");
        if(this.doSegmentation == true){
            //---------
            // with segmentation
            //--------
            this.segmentList = SplitLineString.splitInSegments(this.inputLine,this.segmentationIndizes);            
            int n = this.segmentationIndizes.length +1;
            LineString[] smoothedSegments = new LineString[n];  
            LineString[] segments = this.segmentList;          
            //-- smooth the single segments
            for (int i = 0; i < n; i++) {
                System.out.println("==== smoothing segment no: " + (i+1));
                LineString lineSegment = segments[i];
                //--check nr of points first (min. is 3 according to SnakesSmoothingSegment)
                //  and not bigger than maxPoints to avoid big matrices
                if ((lineSegment.getNumPoints() >= this.minPoints)&& 
                		(lineSegment.getNumPoints() <= this.maxPoints)){
                    LineString newSegment = this.smoothLineMaxDisp(lineSegment, this.maxDistTreshold, this.alpha, this.beta);	                    
                    smoothedSegments[i] = (LineString)newSegment.copy();
                }
                else if (lineSegment.getNumPoints() < this.minPoints){ 
                	//--if to short, use orginal points
                    smoothedSegments[i] = (LineString)lineSegment.copy();
                }
    	        else if(lineSegment.getNumPoints() > this.maxPoints){
    	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Do further segmentation since segment has to much vertices!!!");
    	        	//-- get segmentation points
    	        	int[] pointidx = this.getSegmentationPointsForLongLines(lineSegment);
    	        	//-- do smoothing by use of the class itself .. use one iteration instaed maxDist threshold 	    	        	
    	        	SnakesSmoothingLineNew ssl = new SnakesSmoothingLineNew(lineSegment, this.maxDistTreshold, 
    	        									this.alpha, this.beta, true, pointidx);
    	        	smoothedSegments[i] = ssl.getSmoothedLine();
    	        }
    	        else{
    	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Unhandled case for this number of vertices!!!");
    	        }
	         }
	        //-- concat segments and check if smoothed too much
	        this.smoothedLine = SplitLineString.concatSegements(smoothedSegments);
        }
        //---------
        // no segmentation
        //--------        
        else{
            //check nr of points first (min. is 3 according to SnakesSmoothingSegment)
        	System.out.println("no of points: " + this.inputLine.getNumPoints());
            if ((this.inputLine.getNumPoints() >= this.minPoints) &&
            		(this.inputLine.getNumPoints() <= this.maxPoints)){
                this.smoothedLine = this.smoothLineMaxDisp(this.inputLine, this.maxDistTreshold, this.alpha, this.beta);
            }
	        else if (this.inputLine.getNumPoints() < this.minPoints){ 
	        	//-- if to short, use orginal points
	            this.smoothedLine = this.inputLine;
	            System.out.println("SnakesSmoothingLineNew.solveMaxDisp: line to short to smooth!");
	        }
	        else if(this.inputLine.getNumPoints() > this.maxPoints){
	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Do segmentation since line is has to much vertices!!!");
	        	//-- get segmentation points
	        	int[] pointidx = this.getSegmentationPointsForLongLines(this.inputLine);
	        	//-- do smoothing by use of the class itself
	        	SnakesSmoothingLineNew ssl = new SnakesSmoothingLineNew(this.inputLine, this.maxDistTreshold, 
	        									this.alpha, this.beta, true, pointidx);
	        	this.smoothedLine = ssl.getSmoothedLine();
	        }
	        else{
	        	System.out.println("SnakesSmoothingLineNew.maxDisp: Unhandled case for this number of vertices!!!");
	        }
        }//end else segmentation
    }
    
    /**
     * calls SnakesSmoothingSegment class to smooth the line
     * @param line
     */
    private LineString smoothLine(LineString line){        
        //initialize
        SnakesSmoothingSegment mySnake = new SnakesSmoothingSegment();
        mySnake.initialise(line,this.alpha,this.beta);
        //mySnake.setCorrectBorderPoints(false);
        for (int i = 0; i < this.iterations; i++) {
            mySnake.solve();
        }        
        return mySnake.getSmoothedSegment();
    }
    
    /**
     * smooths a line and segment respectively
     * checks if max displacement value has been exceed<p>
     * @param line
     * @param maxDisp
     * @param alpha
     * @param beta
     * @return smoothed segment/line
     */
    private LineString smoothLineMaxDisp(LineString line, double maxDisp, double alpha, double beta){
        LineString inLine = (LineString)line.copy();
        LineString smoothedSegment = null;
        SnakesSmoothingSegment mySnake = new SnakesSmoothingSegment();
        mySnake.initialise(line,alpha,beta);
        //mySnake.setCorrectBorderPoints(false);
        boolean proceed = true; int j=1;
        double prevmaxDisp = 0; double deltamaxDisp; int u = 0;
        LineString prevLine = line;
        while(proceed){
            mySnake.solve();
            smoothedSegment = mySnake.getSmoothedSegment();
            double dist = this.getMaxPointDisplacement(inLine, smoothedSegment);
            System.out.println("SnakesSmoothingLineNew.solveMaxDisp - Loop: " + j + " maxDisp = " 
                    + dist + " treshold = " + maxDisp);               
            if(dist > maxDisp){
                u=u+1;
                if (j == 1){
                    System.out.println("SnakesSmoothingLineNew.solveMaxDisp: reset alpha, beta, segments");
                    alpha = alpha /2;
                    beta = beta/2;
                    System.out.println("alpha new: " + alpha + " beta new: " + beta);
                    line = (LineString)inLine.copy(); //take old line
                    mySnake.initialise(line,alpha,beta);
                    j = 0;   
                    if (u==10){
                        proceed = false; 
                        this.couldNotSmooth = true;
                        smoothedSegment = prevLine;
                        } //notbremse
                }
                else{
                    proceed = false;
                    smoothedSegment = prevLine;
                }
            }
            else{ //dist < maxDisp
                prevLine = smoothedSegment;
                deltamaxDisp = (dist - prevmaxDisp)/dist;
                if (deltamaxDisp < 0.01){ 
                    //--stop if changes are smaller than 1 percent
                    proceed = false;
                    smoothedSegment = prevLine;
                }
                prevmaxDisp = dist; 
            }
            j++;
        } //end while
        return smoothedSegment;
    }
    
    private double getMaxPointDisplacement(LineString lineOriginal, LineString lineNew){
        double maxdist = 0;
        Coordinate[] coordsOrg = lineOriginal.getCoordinates();
        Coordinate[] coordsNew = lineNew.getCoordinates();
        double dx, dy, s;
        for(int i=0; i < coordsOrg.length; i++){
            /**
            //-- point-point distance is not so good, since for snakes lateral movements appear 
            //   (not perpendicular to line direction like for tafus)
            dx= coordsOrg[i].x - coordsNew[i].x;
            dy= coordsOrg[i].y - coordsNew[i].y;
            s = dx*dx + dy*dy;
            **/
            Point pt = new GeometryFactory().createPoint(coordsNew[i]);
            s = pt.distance(lineOriginal);
            if (s > maxdist){
                maxdist = s;
            }
        }
        return maxdist;
    }
    
    /**
     * calculates the segementation points for too long lines
     * @param line
     * @return pointidx
     */
    private int[] getSegmentationPointsForLongLines(LineString line){
    	//-- calculate random split index = maxpoint +/- x	: x=<5;
    	this.segmentPointIdx = this.maxPoints + (int)Math.ceil((Math.random()-0.5)*10);
   	    int parts = 1 + (int)Math.ceil(line.getNumPoints()/this.maxPoints);   	    
       	int[] pointidx = new int[parts-1];
       	int delta = (int)Math.ceil(line.getNumPoints() / parts);
       	System.out.println("delta: "+ delta);
       	System.out.print("Segmentation point indices for to long lines: ");
       	for (int j = 0; j < pointidx.length; j++) {
       	    pointidx[j]=delta*(j+1);
           	System.out.print(pointidx[j] + "  ");
        }
       	System.out.println(" ");
       	return pointidx;
    }
    
    /******************* getters and setters ************/
    
    public LineString getSmoothedLine() {
        return smoothedLine;
    }
    public boolean isCouldNotSmooth() {
        return couldNotSmooth;
    }
	/**
	 * @return Returns the maxPoints.
	 * 		 maxPoints defines a line segmentation criterion
	 * 		 to avoid too big matrices. 
	 * 
	 */
	public int getMaxPoints() {
		return maxPoints;
	}
	/**
	* @param maxPoints The maxPoints to set.
	* 		 maxPoints defines a line segmentation criterion
	* 		 to avoid too big matrices 
	*/
	public void setMaxPoints(int maxPoints) {
		this.maxPoints = maxPoints;
	}
	/**
	 * @return Returns the minPoints.
	 * necessary number of line vertices to do a smoothing
	 * with snakes technique 
	 * 
	 */
	public int getMinPoints() {
		return minPoints;
	}
	/**
	 * necessary number of line vertices to do a smoothing
	 * with snakes technique 
	 * @param minPoints The minPoints to set.
	 */
	public void setMinPoints(int minPoints) {
		this.minPoints = minPoints;
	}
	/**
	 * @return Returns the segmentPointIdx.
	 * 		   It is calculated from Maxpoints and a  
	 *         variation value between +/-5 obtained from a random process. 
	 */
	public int getSegmentPointIdx() {
		return segmentPointIdx;
	}
}
