package mapgen.measures;

import mapgen.algorithms.Rotate;
import mapgen.geomutilities.SecondGeodeticTask2d;

import org.locationtech.jts.algorithm.ConvexHull;
import org.locationtech.jts.algorithm.MinimumDiameter;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;

/**
 * Calculates the orientation for
 * 		a) mathematical correct MBR
 * 		b) statistical orientation
 * 
 * algorithms by mats bader, Uni ZH
 *
 * created on 		24.09.2004
 * last modified: 	02.12.2004
 *
 * @author sstein
 *
 * @TODO : give confidence value for stat. Orientation
 * 		(using max. angle weight for squared building
 * 		 => Duchene et al. 2003, ICA Workshop Paris)
 */
public class OrientationMBR {

    private Geometry myPolygon;
    
    private LineString jtsDiameter;
    private Polygon mbr;
    private Polygon diameterRectangle;
    private double diameterArea=0;
    private double diameterLength=0;
    private double mbrArea=0;
    // diameter-orientation can be angle- Pi/2
    // and can be different from mbr orientation (prefer the last one) 
    private double diameterOrientation=0;
    private double mbrOrientation=0;
    // statistical weight gives two main directions back
    private double statOrientation=0;
    private double mbrWidth=0;
    private double mbrLength=0;
    private int statOuterPolygonEdges=0;

    private boolean mbrOrientationCalculated = false;
    private boolean JtsDiameterCalculated = false;
    private boolean statOrientationCalculated = false;
    private boolean mbrWidthLengthCalculated = false;
    
    private int errori;
   
    /**
     * constructor sets polygon to analyse and calculates stuff
     * @param inPolygon
     */
    public OrientationMBR(Geometry inPolygon){
        this.mbrOrientationCalculated = false;
        this.mbrWidthLengthCalculated = false;
        this.statOrientationCalculated = false;
        this.JtsDiameterCalculated = false;
        
        this.myPolygon = inPolygon;
        /*** now using getx() methodes to start calculation
        this.calcMathCorrMBR();
        this.calcJtsDiameter();
        this.calcWHfromMBR();
        this.calcStatisticMBR();  
        */      
    }
    
    /**
     *  calculates mathematical correct MBR
     */
    private void calcMathCorrMBR(){              
        
        Polygon myMbr = null;        
        Geometry tempGeom = null;
        double mbrAngle = 0;
        
        // calculate the convex hull
        ConvexHull CH = new ConvexHull(this.myPolygon);
        // get outer Ring of convex hull and count verticies
        Geometry myHull = CH.getConvexHull();
        
        try{ 
            Polygon myHullPolygon = (Polygon)myHull;
            // attention: first point = last point for ExteriorRing - Linestring
            LineString outerring = myHullPolygon.getExteriorRing();
            //get Numer of points of outer ring
            int nrPoints = outerring.getNumPoints();

            /* The MBR will have an area of at least the size of the	
            * axis-parallel mbr. Therefore use that	
            * size as 'minArea', so one of the now computed MBR's	
            * will be smaller.	
            */
            Geometry myAxesParallelMBR = this.myPolygon.getEnvelope();
            double minArea = myAxesParallelMBR.getArea();
            this.setMbr((Polygon)myAxesParallelMBR);	//set Initial MBR
            
            /* Retrieve last point of the convex hull.			
            * Together with first point of Convex Hull, this line	
            * builds the first edge to test for minimal mbr.		
            */
            Point lastHullPoint = outerring.getEndPoint();
            double x1 = lastHullPoint.getX();
            double y1 = lastHullPoint.getY();
            
            /* Go through all edges of the Convex-Hull.			
            * The mathematical MBR is parallel to at least one side	
            * of the Convex Hull.				
            */
            
            for (int i = 0; i < nrPoints; i++) {
                this.errori = i;
                /* Compute current edge of convex hull.			
                * Start point: available from last iteration.		
                * End point:	 read from hull-collection.		
                */                
                double x2 = outerring.getPointN(i).getX();
                double y2 = outerring.getPointN(i).getY();
                double dx = x2 - x1;
                double dy = y2 - y1;
                
                // in first step dx=dy=0 since lastpoint = firstpoint for ring
                if ((dx != 0.0) || (dy!=0.0)){
                    
                    double angle = 0;
               	 	/* Compute angle of edge to the horizontal.	
               	 	* Notice that the Convex Hull is oriented counter- 
               	 	* clockwise. => rotate(-1*angle)			
               	 	*/
                    if (dy > 0.0) {angle = Math.acos(dx/Math.sqrt(dx*dx + dy*dy));}
                    if (dy < 0.0) {angle = Math.PI - Math.acos(dx / Math.sqrt(dx*dx + dy*dy));}
                    if (dy == 0.0) {angle = Math.acos(1.0);}
                    if (dx == 0.0) {angle = Math.acos(0.0);}
                    
               	 	/* Copy the building-geometry to a temporary geometry.
               	 	 * otherwise the original geometry will be changed	            
               	 	 */
                    tempGeom = (Geometry)this.myPolygon.copy();
                    
                    /* Rotate the temp-geometry to the horizontal,		
                    * and compute the axis-parallel MBR.				
                    */

                    JtsCentroid myCentroid = new JtsCentroid(this.myPolygon);
                    tempGeom = Rotate.rotate(myCentroid.getGravityCentroid(),-1*angle, (Polygon)tempGeom);                    
                    
                    Geometry myAxesParallelMBR2 = tempGeom.getEnvelope();                    
                    double actualArea = myAxesParallelMBR2.getArea();
                    
                    if( actualArea < minArea){
                        // Store new critical values
                        minArea = actualArea;
                        mbrAngle = angle;
                        
                        //rotate found mbr back
                        myAxesParallelMBR2 = Rotate.rotate(myCentroid.getGravityCentroid(),angle,(Polygon)myAxesParallelMBR2);                        
                        this.setMbr((Polygon)myAxesParallelMBR2);
                    }                                        
                    /* Make the Endpoint of current edge to the Startpoint	
                    * to the new edge.    /    Incremenet iterator.
                    */		
                    x1 = x2;
                    y1 = y2;                    
                } //end if
            }//end for
            
            // store angle as values from 0 to pi
            double div = mbrAngle/Math.PI;
            double full = Math.floor(div);
            double rest = div-full;
            mbrAngle = rest*Math.PI;

            this.setOrientation(mbrAngle);            
            this.setMbrArea(minArea);
            
        }
        catch(Exception e){
            System.out.println("geometry for tempMBR i:" + errori + " isn't a polygon");
        }
        
    }	//end function
    
    /**
     * calculates Diameter and Minimum Bounding Rectangle using 
     * JTS - diameter method
     */
    private void calcJtsDiameter() {
        /*
         * calc the diameter (the smallest hole that contains the geometry)
         * using jts
         */
        MinimumDiameter myDiameter = new MinimumDiameter(this.myPolygon); 
        LineString myHole = myDiameter.getDiameter();
        this.setJtsDiameter(myHole);
        this.setDiameterLength(myHole.getLength());

        
        // calc the corresponding rectangle
        double x1 = myHole.getCoordinateN(0).x;
        double y1 = myHole.getCoordinateN(0).y;
        double x2 = myHole.getCoordinateN(1).x;
        double y2 = myHole.getCoordinateN(1).y;
        double dx = x2 - x1;
        double dy = y2 - y1;
                
   	 	// Compute angle of edge to the horizontal.	
        double angle = 0;
        if ((dx != 0.0) || (dy!=0.0)){        
            angle = SecondGeodeticTask2d.calcAngle(dx,dy);
        }
        
        // copy poygon and rotate
        Geometry tempGeom = (Geometry)this.myPolygon.copy();
        
        JtsCentroid myCentroid = new JtsCentroid(this.myPolygon);
        tempGeom = Rotate.rotate(myCentroid.getGravityCentroid(),-1*angle, (Polygon)tempGeom);                    
        
        Geometry myAxesParallelMBR = tempGeom.getEnvelope();
        double actualArea = myAxesParallelMBR.getArea();
        // rotate back;
        myAxesParallelMBR = Rotate.rotate(myCentroid.getGravityCentroid(),angle, (Polygon)myAxesParallelMBR);        
        
        this.setDiameterArea(actualArea);
        this.setDiameterRectangle((Polygon)myAxesParallelMBR);

        angle = angle + Math.PI;
        // store angle as values from 0 to pi        
        double div = angle/Math.PI;
        double full = Math.floor(div);
        double rest = div-full;
        angle = rest*Math.PI;
        this.setDiameterOrientation(angle);
        
    }

    /**
     * calculates width and Height from MBR
     *
     */    
    public void calcWHfromMBR(){
        
        // rotate object using mbr-orientation
        Geometry myObject = this.myPolygon;
        double angle = this.mbrOrientation;
        Geometry tempGeom = (Geometry)myObject.copy();
        JtsCentroid myCentroid = new JtsCentroid(myObject);
        tempGeom = Rotate.rotate(myCentroid.getGravityCentroid(),angle, (Polygon)tempGeom);
        // calulate envelope(=Bounding rectangle) from now axes parallel object
        Envelope myE = tempGeom.getEnvelopeInternal();
        
        double L1 = myE.getHeight();
        double L2 = myE.getWidth();
        // sort and assign
        if (L1 <= L2){
            this.setMbrLength(L2);
            this.setMbrWidth(L1);
        }
        else{
            this.setMbrLength(L1);
            this.setMbrWidth(L2);            
        }
        
    }
    
    /**
     * calculates orientation using statistical 
     * analysis (histogramm of angles)
     * uses only the outer Ring Elements
     * for description see Duchêne et al. 2003(ICA Workshop, Paris)
     */
    private void calcStatisticMBR(){

        double[] angles;
        double[] lengths;
        double primeAngle = 0;
        
        /* Precision of orientation. The output direction is a multiple of this value.
		* Note that ANGLE_PRECISION is defined in degrees, not radians.		
		*/
        double anglePrecision = 1.0;
        
        /* Parameter to specify weighting. There is one line in this code that	
		* specifies the weighting. Currently a linear inverse distance weighting	
		* is used, where WEIGTHING_WIDTH is its parameter.				
		*/        
        double weightingWidth = Math.PI/12.0;

        /* Iterate through all edges and compute their orientation.	
        * Orientation-values:  0 <= x < Pi				
        * Store the calculated orientations in 'angle_coln' and 		
        * their lengths in 'length_coln'.					
        */
        Polygon myObject = (Polygon)myPolygon;
        LineString outerRing = myObject.getExteriorRing();
        // attention: first and last coordinate are the same (since it is a ring) 
        Coordinate[] myCoordinates = outerRing.getCoordinates();
        int nrPoints = outerRing.getNumPoints();
        //set number of polygon edges of outer ring
        this.setStatOuterPolygonEdges(nrPoints-1);
        
        angles = new double[nrPoints-1];
        lengths = new double[nrPoints-1];
        double x1,y1,x2,y2,dx,dy;
        for (int i = 0; i < myCoordinates.length-1; i++) {
            x1 = myCoordinates[i].x;
            y1 = myCoordinates[i].y;
            x2 = myCoordinates[i+1].x;
            y2 = myCoordinates[i+1].y;
            dx = x2 - x1;
            dy = y2 - y1;
            
            angles[i] = 0;
            if ((dx != 0.0) || (dy!=0.0)){                
                if (dy > 0.0) {angles[i] = Math.acos(dx/Math.sqrt(dx*dx + dy*dy));}
                if (dy < 0.0) {angles[i] = Math.PI - Math.acos(dx / Math.sqrt(dx*dx + dy*dy));}
                if (dy == 0.0) {angles[i] = Math.acos(1.0);}
                if (dx == 0.0) {angles[i] = Math.acos(0.0);}
                
                // store as values from 0 to pi
                double div = angles[i]/Math.PI;
                double full = Math.floor(div);
                double rest = div-full;
                angles[i] = rest*Math.PI;                
            }
            lengths[i] = Math.sqrt(dx*dx + dy*dy);             
        } // end for        
        
        /* for every ANGLE_PRECISION degree, we compute the weighted	
         * sum of the length of the edges.				
         */               
        double anglePrecInRad = Math.PI*anglePrecision/180;
        double steps = Math.floor(Math.PI/anglePrecInRad); //=180
        double[] allAngles = new double[(int)steps];
        double[] found = new double[(int)steps];
        double[] total = new double[(int)steps];
        double maxLength = 0;
        double curAngle = 0;
        // loop from 0 < curAngle <(!) pi : (last value is smaller than pi) 
        // with anglePrecInRad as step width
        for(int i = 0; i < (int)steps; i++){            
            // loop over all edges 
            for(int j=0; j < nrPoints-2; j++){
           	 	/* Compute the angle differences between the        
          	 	 * current edge and the i-th edge.                   
           	 	 * The distance must always be less than PI/2.
           	 	 */                      
                 double delta = Math.abs(curAngle - angles[j]);
                 if(delta > Math.PI/2) {delta = Math.PI - delta;}
                /* If the distance is smaller than WEIGHTING_WIDTH          
                 * the length is weighted (inverse distance weighting).      
                 */
                if (delta < weightingWidth){
                    found[i]=found[i]+1;
                    //weighting in distance to main-rasterised-angle
                    //and using the length of the edge
                    total[i] = total[i] + (weightingWidth - delta) / weightingWidth *lengths[j];
                }   
            }//end for
            // save amplitude and angle for highest "weight" value
            if (total[i] > maxLength){
                 maxLength = total[i];
                 primeAngle = curAngle;                  
             }
            allAngles[i]= curAngle;
            curAngle = curAngle + (anglePrecInRad);                        
        }//end for curAngle
        
        /********************** PLOT ***********************/
        /*
		// Build a 2D data set
		double[][] datas1 = new double [allAngles.length][2];
		double[][] datas2 = new double [allAngles.length][2];		
		for (int i = 0; i < datas1.length; i++) {			
				datas1[i][0] = allAngles[i];
				datas2[i][0] = allAngles[i];
				datas1[i][1] = total[i];
				datas2[i][1] = found[i];
		}
		// Build the 2D scatterplot of the datas in a Panel
		// LINE, SCATTER, BAR, QUANTILE, STAIRCASE, (HISTOGRAMM?)		
		Plot2DPanel plot2d = new Plot2DPanel();
		plot2d.addPlot(datas1,"total","LINE");
		plot2d.setAxeLabel(0,"angle 0..pi");
		plot2d.setAxeLabel(1,"weight");	
		plot2d.addPlot(datas2,"found","SCATTER");		
		// Display a Frame containing the plot panel
		new FrameView(plot2d);
		*/
       /*********************************************/
      
       this.setStatOrientation(primeAngle);
        
    }
    
    /*************** getters and setter **********************/
    
    /**
     * calculates mathematically correct Minimum Bounding Rectangle 
     * and returns the mbr
     */
    public Polygon getMbr() {
        if (this.mbrOrientationCalculated == false){
            this.calcMathCorrMBR();
            this.mbrOrientationCalculated = true;
        }
        return mbr;
    }
    
    private void setMbr(Polygon mbr) {
        this.mbr = mbr;
    }
    
    /**
     * calculates Diameter and Minimum Bounding Rectangle using 
     * JTS - diameter method and returns the diameter
     */
    public LineString getJtsDiameter() {
        if (this.JtsDiameterCalculated == false){
            this.calcJtsDiameter();
            this.JtsDiameterCalculated = true;
        }
        return jtsDiameter;
    }
    
    private void setJtsDiameter(LineString diameter) {
        this.jtsDiameter = diameter;
    }

    /**
     * calculates mathematically correct Minimum Bounding Rectangle 
     * and returns the mbr orientation
     */    
    public double getOrientation() {
        if (this.mbrOrientationCalculated == false){
            this.calcMathCorrMBR();
            this.mbrOrientationCalculated = true;
        }        
        return mbrOrientation;
    }
    
    private void setOrientation(double orientation) {
        this.mbrOrientation = orientation;
    }

    /**
    * calculates Diameter and Minimum Bounding Rectangle using 
    * JTS - diameter method and returns the diameter rectangle area
    */
    public double getDiameterArea() {
        if (this.mbrOrientationCalculated == false){
            this.calcJtsDiameter();
            this.JtsDiameterCalculated = true;
        }
        return diameterArea;
    }
    
    private void setDiameterArea(double diameterArea) {
        this.diameterArea = diameterArea;
    }
    
    /**
     * calculates Diameter and Minimum Bounding Rectangle using 
     * JTS - diameter method and returns the diameter-rectangle
     */    
    public Polygon getDiameterRectangle() {
        if (this.JtsDiameterCalculated == false){
            this.calcJtsDiameter();
            this.JtsDiameterCalculated = true;
        }        
        return diameterRectangle;
    }
    
    private void setDiameterRectangle(Polygon diameterRectangle) {
        this.diameterRectangle = diameterRectangle;
    }
    
    /**
     * calculates mathematically correct Minimum Bounding Rectangle 
     * and returns the mbr area
     */        
    public double getMbrArea() {
        if (this.mbrOrientationCalculated == false){
            this.calcMathCorrMBR();
            this.mbrOrientationCalculated = true;
        }           
        return mbrArea;
        
    }   
    
    private void setMbrArea(double mbrArea) {
        this.mbrArea = mbrArea;
    }

    /**
     * calculates Diameter and Minimum Bounding Rectangle using 
     * JTS - diameter method and returns the diameter orientation
     * (can be different to mbr orientation)
     */        
    public double getDiameterOrientation() {
        if (this.mbrOrientationCalculated == false){
            this.calcJtsDiameter();
            this.JtsDiameterCalculated = true;
        }                
        return diameterOrientation;
    }
    
    private void setDiameterOrientation(double diameterOrientation) {
        this.diameterOrientation = diameterOrientation;
    }

    /**
     * calculates mathematically correct Minimum Bounding Rectangle 
     * and returns the mbr orientation
     */            
    public double getMbrOrientation() {
        if (this.mbrOrientationCalculated == false){
            this.calcMathCorrMBR();
            this.mbrOrientationCalculated = true;
        }                
        return mbrOrientation;
    }
    
    private void setMbrOrientation(double mbrOrientation) {
        this.mbrOrientation = mbrOrientation;
    }
    
    /**
     * 
     * @return length of the polygon mbr
     */
    public double getMbrLength() {
        if (this.mbrOrientationCalculated == false){
            this.calcMathCorrMBR();
            this.mbrOrientationCalculated = true;
        }
        if (this.mbrWidthLengthCalculated == false){
            this.calcWHfromMBR();
            this.mbrWidthLengthCalculated = true;
        }
        return mbrLength;
    }
    
    private void setMbrLength(double mbrLength) {
        this.mbrLength = mbrLength;
    }
    
    /**
     * 
     * @return width of the polygon mbr
     */
    public double getMbrWidth() {
        if (this.mbrOrientationCalculated == false){
            this.calcMathCorrMBR();
            this.mbrOrientationCalculated = true;
        }
        if (this.mbrWidthLengthCalculated == false){
            this.calcWHfromMBR();
            this.mbrWidthLengthCalculated = true;
        }        
        return mbrWidth;
    }
    
    private void setMbrWidth(double mbrWidth) {
        this.mbrWidth = mbrWidth;
    }    
    
    /**
     * calculates and returns a orientation based on 
     * the wall statistics
     */                
    public double getStatOrientation() {
        if (this.statOrientationCalculated == false){
            this.calcStatisticMBR();
            this.statOrientationCalculated = true;
        }
        return statOrientation;
    }
    
    private void setStatOrientation(double statOrientation) {
        this.statOrientation = statOrientation;
    }
    
    /**
     * 
     * @return the number of polygon edges of outerring
     *  used during calculation of wall statistic orientation 
     */
    private int getStatOuterPolygonEdges() {
        if (this.statOrientationCalculated == false){
            this.calcStatisticMBR();
            this.statOrientationCalculated = true;
        }        
        return statOuterPolygonEdges;
    }
    
    private void setStatOuterPolygonEdges(int statOuterPolygonSides) {
        this.statOuterPolygonEdges = statOuterPolygonSides;
    }
    
    /**
     * calculates Diameter and Minimum Bounding Rectangle using 
     * JTS - diameter method and returns the diameter length
     */        
    public double getDiameterLength() {
        if (this.mbrOrientationCalculated == false){
            this.calcJtsDiameter();
            this.JtsDiameterCalculated = true;
        }                
        return diameterLength;
    }
    
    private void setDiameterLength(double diameterLength) {
        this.diameterLength = diameterLength;
    }
 }
