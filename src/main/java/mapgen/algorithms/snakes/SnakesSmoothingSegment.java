/***********************************************
 * created on 		16.12.2004
 * last modified: 	14.05.2005 (nr of minimal accpted vertices per segment)
 * 
 * author:			sstein
 * 
 * description:
 *   Smooths a segment / LineString without segmentation
 *   in one step. It should be called by SnakesSmoothingLine
 *   class to use segmentation and an iteration process.  
 *   <p> 
 *   initialize() first and then use solve();
 *   get the smoothed line via getSmoothedLine();
 * 
 ***********************************************/
package mapgen.algorithms.snakes;

import org.jmat.MatlabSyntax;
import org.jmat.data.Matrix;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;

/**
 * @description:
 *   Smooths a segment / LineString without segmentation
 *   in one step. It should be called by SnakesSmoothingLine
 *   class to use segmentation and an iteration process.  
 *   <p> 
 *   initialize() first and then use solve();
 *   get the smoothed line via getSmoothedLine(); 
 * 
 * @author sstein
 *
 */
public class SnakesSmoothingSegment{

    private Matrix Apent = null;
    private Matrix ApInv = null;
    private int nrOfMirrorPointsPerSide = 5; //could be changed during calculation, depending on nr of vertices
    private int nrOfVertices = 0; // size of original segment 
    private int size = 0; // size of matrix
    private boolean matWasCalculated = false;
    private boolean correctBorderPoints = true;
    private LineString originalSegment = null;
    private Matrix originalMatrix = null;
    private Matrix xsmooth= null;
    private Matrix ysmooth= null; 
    private Matrix xinit = null;
    private Matrix yinit = null;

    /**
     * extends the line by mirroring of the start and endpoints, 
     * calculates the pentadiagonal matrix
     * @param segment
     * @param alpha
     * @param beta
     */
    public void initialise(LineString segment, 
			 double alpha, double beta){
        
        this.originalSegment = segment;
        //--------
        this.nrOfVertices = segment.getNumPoints();        
        if(nrOfVertices > 5){
            //-- mirror border points
            this.nrOfMirrorPointsPerSide = 5;
            this.createOriginalMatrix(this.originalSegment);
            this.size = this.nrOfVertices + 2* this.nrOfMirrorPointsPerSide;
            //-- calc Apent
	        this.Apent = SnakesMatrixOperations.getPentaDiagConstParamMatrix(alpha,beta,this.size);
	        SnakesMatrixOperations.scaleMatrix(this.Apent);
	        SnakesMatrixOperations.scaleBorderOfMatrix(this.Apent);
	        this.ApInv = (Matrix)this.Apent.inverse();        
	        this.matWasCalculated = true;
        }
        else if ((nrOfVertices >= 3) && (nrOfVertices <6 )){
            //-- mirror border points
            this.nrOfMirrorPointsPerSide = nrOfVertices-1; //-1 since mirroring would not work
            this.createOriginalMatrix(this.originalSegment);
            this.size = this.nrOfVertices + 2* this.nrOfMirrorPointsPerSide;
            //-- calc Apent
	        this.Apent = SnakesMatrixOperations.getPentaDiagConstParamMatrix(alpha,beta,this.size);
	        SnakesMatrixOperations.scaleMatrix(this.Apent);
	        SnakesMatrixOperations.scaleBorderOfMatrix(this.Apent);
	        this.ApInv = (Matrix)this.Apent.inverse();        
	        this.matWasCalculated = true;
	        System.out.println("SnakesSmoothingSegment.initialize : line with 3-5 vertices : problems can appear");
        }        
        else{
            System.out.println("SnakesSmoothingSegment.initialize : line has less than 3 vertices");
        }       
    }

    /**
     * Solves the equation system = smoothing of segment.
     * Uses the last smoothing result which can be recieved via
     * getSmoothedSegment(). This is not necessarily
     * the input line. 
     */
    public void solve(){        
        if (this.matWasCalculated == false){
            System.out.println("SnakesSmoothing: could not calculate, since Apent is missing!");
        }
        else{
            //mirror border points
            Matrix xmirror = SnakesMatrixOperations.mirror1D(xsmooth,this.nrOfVertices,this.nrOfMirrorPointsPerSide);
            Matrix ymirror = SnakesMatrixOperations.mirror1D(ysmooth,this.nrOfVertices,this.nrOfMirrorPointsPerSide);
            //solve
	        Matrix xtemp = MatlabSyntax.times(this.ApInv,xmirror);           
	        Matrix ytemp = MatlabSyntax.times(this.ApInv,ymirror);
	        //extract original length
	        xsmooth = SnakesMatrixOperations.extractOriginalLineLength1D(xtemp,this.nrOfVertices, this.nrOfMirrorPointsPerSide);
	        ysmooth = SnakesMatrixOperations.extractOriginalLineLength1D(ytemp,this.nrOfVertices, this.nrOfMirrorPointsPerSide);
	        //correct border points
	        if(this.correctBorderPoints == true){
		        xsmooth = this.borderPointsCorrect1D(xinit,xsmooth);
		        ysmooth = this.borderPointsCorrect1D(yinit,ysmooth);
	        }
        }
    }
    
    /**
     * Initializes the vectors/matrizes originalMatrix xsmooth and ysmooth 
     * @param segment as LineString
     */
    private void createOriginalMatrix(LineString segment){
       int n = this.nrOfVertices;
       this.originalMatrix = MatlabSyntax.zeros(n,2);
       //-- fill matrices 
       double x,y; 
       for (int i = 0; i < n; i++) {
           x = segment.getCoordinateN(i).x;
           y = segment.getCoordinateN(i).y;
           this.originalMatrix.set(i,0,x);
           this.originalMatrix.set(i,1,y);
       }                       
       //-- init
       this.xsmooth = (Matrix)this.originalMatrix.getColumn(0); 
       this.ysmooth = (Matrix)this.originalMatrix.getColumn(1);
       this.xinit = (Matrix)xsmooth.copy();
       this.yinit = (Matrix)ysmooth.copy();       
    }
    
    /**
     * restores the original line length
     * @param xsmooth
     * @param ysmooth
     * @param nrOfMirroredPoints
     * @return Matrix of format [x y] with length of original line 
     */
    private Matrix extractOriginalLineLength(Matrix xsmooth, Matrix ysmooth, int nrOfMirroredPoints){        
        Matrix mat =  MatlabSyntax.zeros(this.nrOfVertices,2);
        double x,y;
        for (int i = 0; i < this.nrOfVertices; i++) {
            x = xsmooth.get(i+ nrOfMirroredPoints,0);
            y = ysmooth.get(i+ nrOfMirroredPoints,0);
            mat.set(i,0,x);
            mat.set(i,1,y);            
        }            
        return mat;
    }
    
    /**
     * corrects the first and last points to original position
     * @param orgLine
     * @param smoothedLine
     * @return
     */
    private Matrix correctStartAndEndPoint(Matrix orgLine, Matrix smoothedLine){
        Matrix correctLine = (Matrix)smoothedLine.copy();
        int n = this.nrOfVertices-1;
        //set start and endpoint
        correctLine.set(0,0,orgLine.get(0,0)); //set x
        correctLine.set(0,1,orgLine.get(0,1)); //set y
        correctLine.set(n,0,orgLine.get(n,0)); //set x
        correctLine.set(n,1,orgLine.get(n,1)); //set y
       
        return correctLine;
    }

    /**
     * change: function does cotrect now only line end points<p>
     * corrects the first and last three values to avoid to big changes
     * in the coordinates and to ensure that line is still topologic correct.
     * The differences between original and smoothed points are weighted by <p>
     * [0 , 1/3 , 2/3 ,1 ... 1 , 2/3, 1/3, 0] <p>
     * Therefore the line musst have a lenght of at least 6 points. Otherwise
     * only the start and end point are corrected.  
     * @param orgValue original coordinate values for dimension
     * @param smoothValue smoothed  coordinate values for dimension
     * @return
     */
    private Matrix borderPointsCorrect1D(Matrix orgValue, Matrix smoothValue){
        Matrix correctLine = (Matrix)smoothValue.copy();
        int n = this.nrOfVertices-1;
        if (orgValue.getRowDimension() < 6){
            //set start and endpoint
            correctLine.set(0,0,orgValue.get(0,0)); //set x
            correctLine.set(n,0,orgValue.get(n,0)); //set x
           
            //System.out.println("SnakesSmoothingSegment: can't do borderPointCorrection since less than 6 points in line.."); 
            //System.out.println("SnakesSmoothingSegment: ..correct only start and end point");
        }
        else{
            //set start and endpoint
            correctLine.set(0,0,orgValue.get(0,0)); //set x
            correctLine.set(n,0,orgValue.get(n,0)); //set x
            //System.out.println("SnakesSmoothingSegment: ..bodercorrection is off, correct only start and end point");
            /**            
            //second point
            double dx1 = 1/2.0 *(smoothValue.get(1,0) - orgValue.get(1,0)); 
            correctLine.set(1,0,orgValue.get(1,0) + dx1);
            
            double dxn1 = 1/2.0 *(smoothValue.get(n-1,0) - orgValue.get(n-1,0)); 
            correctLine.set(n-1,0,orgValue.get(n-1,0) + dxn1);
            
            //third point
            double dx2 = 3/4.0 *(smoothValue.get(2,0) - orgValue.get(2,0)); 
            correctLine.set(2,0,orgValue.get(2,0) + dx2);
            
            double dxn2 = 3/4.0 *(smoothValue.get(n-2,0) - orgValue.get(n-2,0)); 
            correctLine.set(n-2,0,orgValue.get(n-2,0) + dxn2);
            **/
        }
        
        return correctLine;
    }    
    
    /**
     * obtains a LineString of a given Matrix of Format [x y], where the
     * number of verices is equal to number of matrix rows.
     * @param inMatrix
     * @return line as JTS LineString
     */
    private LineString MatrixToLineString(Matrix inMatrix){
        int n = inMatrix.getRowDimension();
        Coordinate[] coords = new Coordinate[n];
        double x,y;
        for (int i = 0; i < n; i++) {
            x = inMatrix.get(i,0);
            y = inMatrix.get(i,1);
            coords[i] = new Coordinate(x,y);            
        }        
        GeometryFactory gf = new GeometryFactory();
        LineString ls = gf.createLineString(coords);        
        return ls;
    }
    
    /** 
     * returns the smoothed line. But before does 
     * <p>
     * - extract the original line length<p>
     * - corrects the border points<p>
     * - obtains the lineString from the operational matrices  <p>
     * @return the smoothed line
     */
    public LineString getSmoothedSegment() {
        Matrix smoothedMatrix  = MatlabSyntax.zeros(this.nrOfVertices,2);
        smoothedMatrix.setColumn(0,this.xsmooth);
        smoothedMatrix.setColumn(1,this.ysmooth);        
        Matrix outMatrix = smoothedMatrix;
        //-- this has been partly replaced by borderPointsCorrect1D() 
        //    during solve()
        //if (this.correctBorderPoints == true){
        //    outMatrix  = this.correctStartAndEndPoint(this.originalMatrix, smoothedMatrix);
        //}
        //
        LineString smoothedSegment = MatrixToLineString(outMatrix);
        return smoothedSegment;
    }
    
    /****************** getters setters **********************/
    public int getNrOfMirrorPointsPerSide() {
        return nrOfMirrorPointsPerSide;
    }
    public void setNrOfMirrorPointsPerSide(int nrOfMirrorPointsPerSide) {
        this.nrOfMirrorPointsPerSide = nrOfMirrorPointsPerSide;
    }
    
    public boolean isCorrectBorderPoints() {
        return correctBorderPoints;
    }
    public void setCorrectBorderPoints(boolean correctBorderPoints) {
        this.correctBorderPoints = correctBorderPoints;
    }

}
