/***********************************************
 * created on 		08.12.2004
 * last modified: 	01.08.2005 new internal Energy calculations 
 * 								#calcIntEnergyDiff()
 * 								#calcInternalEnergy()
 * 								#calcInternalEnergy() change: exclude matrix scaling
 * 					05.08.2005  new matrix scaling
 * 					16.04.2006  #calcSumPointDisplacement()	
 * 					23.04.2006  changed	#calcIntEnergyDiff(): must be E_new-E_old
 * 								which will influence the signs  			
 *  
 * author:			sstein
 * 
 * description:
 *  Calculates internal energy for the points of one line from the
 *  given Lists of conflict matrices with other lines. 
 *  The conflict matrices can be obtained from the measure
 *  and contain the point wise conflicts with other lines.
 * 
 *  For external energy the point wise calculation is done
 *  among the original and the displaced line.  
 *  
 * 
 ***********************************************/
package mapgen.algorithms.snakes;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;

import org.jmat.MatlabSyntax;
import org.jmat.data.AbstractMatrix;
import org.jmat.data.Matrix;

import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;

/**
 * @description:
 *  Calculates internal energy for the points of one line from a
 *  given List of conflict matrices with other lines.
 *  The conflict matrices can be obtained from the measure 
 *  The energy equals metric distance (= is not normalized)
 * 
 *  For external energy the point wise calculation is done
 *  among the original and the displaced line.
 *   
 * @author sstein
 *
 */
public class SnakesEnergyDisplacement {
           
    /**
     *  Calculates energy for the points of one line from the
     *  given Lists of conflict matrices with other lines.
     *  The conflict matrices can be obtained from the measure.
     *
     * @param dxConflictMatrixList contains the point wise conflicts with 
     *                other lines obtained from distance measure
     * @param dyConflictMatrixList contains the point wise conflicts with 
     *                other lines obtained from distance measure
     * 
     * @return a two column matrix containing the
     * 		   components [dx,dy] of the displacement
     *         vector for a line point  
     */
    public static Matrix calcExtEnergyOverAllLines(List dxConflictMatrixList, List dyConflictMatrixList, int nrLinePoints){

        Matrix sumdisp = MatlabSyntax.zeros(nrLinePoints,2);
        Matrix sumNonZero = MatlabSyntax.zeros(nrLinePoints,2);
        Matrix mat = null;
        Matrix nonzero = null;
        int listLength = dxConflictMatrixList.size();
        // iterate over all lines = conflict matrices
        for (int i = 0; i < listLength; i++) {
             Matrix dxMat = (Matrix)dxConflictMatrixList.get(i);
             Matrix dyMat = (Matrix)dyConflictMatrixList.get(i);
             mat = calcPointEnergyForOneLine(dxMat, dyMat);            
             nonzero = findNonZeroRows(mat);             
             if (i == 0){
                 sumdisp = (Matrix)mat.copy(); 
                 sumNonZero = (Matrix)nonzero.copy(); 
             }
             else{
                 sumdisp = MatlabSyntax.plus(sumdisp,mat);
                 sumNonZero = MatlabSyntax.plus(sumNonZero,nonzero);
             }
        } 
        //dived sum of all dx (dy) movements by number of dx (dy) movements
        Matrix displacementxy = matrixDivide(sumdisp, sumNonZero);
        return displacementxy;
    }
    
    /**
     * calculates energy for the points of one line
     * energy equals metric distance (= is not normalized) 
     * @param dxMat
     * @param dyMat
     * @return matrix with displacement components [dx,dy] 
     *         for every point of line
     */
    private static Matrix calcPointEnergyForOneLine(Matrix dxMat, Matrix dyMat){ 
        int nrPointsLineA = dxMat.getRowDimension();
        int nrPointsLineB = dxMat.getColumnDimension();
        Matrix mat = MatlabSyntax.zeros(nrPointsLineA,2);
        for (int i = 0; i < nrPointsLineA; i++) {
            double sumdx = 0;
            double sumdy = 0;
            int count = 0;
            for (int j = 0; j < nrPointsLineB; j++) {
                //-- get already calculated displacement vector elements 
                double dx = dxMat.get(i,j); 
                double dy = dyMat.get(i,j);
                //-- if displacement conflict between point and edge then sum up
                if( (dx !=0) || (dy!=0)){
                    sumdx = sumdx + dx;
                    sumdy = sumdy + dy;
                    count = count +1;
                }
              double movedx = 0; double movedy = 0;
             if (count > 0){
	            movedx = sumdx/count;
	            movedy = sumdy/count;
             }
            mat.set(i,0,movedx);
            mat.set(i,1,movedy);
            }            
        }
        
        return mat;
    }        
    
    /**
     * 
     * @return a one dimensional vector/matrix there row element is 0 
     * if all elements of a row a zero and it is 1 if at least one
     * element of a row is not zero  
     */
    private static Matrix findNonZeroRows(Matrix inMat){
        int nrPoints = inMat.getRowDimension();
        int nrDim = inMat.getColumnDimension();
        boolean nonzero = false;
        Matrix outMat = MatlabSyntax.zeros(nrPoints,1);
        for (int i = 0; i < nrPoints; i++) {
            nonzero = false;
            for (int j = 0; j < nrDim; j++) {
                if (inMat.get(i,j) != 0){
                    nonzero = true;
                }
            }
            if (nonzero == true){
                outMat.set(i,0,1);
            }
            else{
                outMat.set(i,0,0);
            }
        }         
        return outMat;
    }
    
    /**
     * divides the values of the input matrix row wise using the divisor 
     * value from the (one dimensional=list) divisor matrix.   
     * @param inMat
     * @param divisorMat one dim matrix / vector
     * @return
     */
    private static Matrix matrixDivide(Matrix inMat, Matrix divisorMat){
        int nrPoints = inMat.getRowDimension();
        int nrDim = inMat.getColumnDimension();
        Matrix resultMat = MatlabSyntax.zeros(nrPoints, nrDim);
        double value = 0;
        double divisor =0;
        for (int i = 0; i < nrPoints; i++) {            
            for (int j = 0; j < nrDim; j++) {
                divisor = divisorMat.get(i,0);
                if (divisor != 0){
                    value = inMat.get(i,j) / divisor;
                    resultMat.set(i,j, value);
                 }
                else{
                    resultMat.set(i,j,0);
                }                 
            }
        }                         
        return resultMat;
    }
        
    /**
     * Attention: this calculation of E_int is wrong!!! [sstein] 01.08.2005 <p>
     * calculates the internal energy for snakes displacement
     * @param originalLine
     * @param displacedLine
     * @return matrix of point(old-new) differences [dx_i,dy_i] 
     */
    public static Matrix calcInternalEnergy(LineString originalLine, LineString displacedLine){
        int n =originalLine.getNumPoints(); 
        Matrix mat = MatlabSyntax.zeros(n,2);
        double dx, dy;
        for (int i = 0; i < n; i++) {
            Point pold = originalLine.getPointN(i);
            Point pnew = displacedLine.getPointN(i);
            dx = pold.getX() - pnew.getX();
            dy = pold.getY() - pnew.getY();
            mat.set(i,0,dx);
            mat.set(i,1,dy);
        }        
        return mat;
    }

    /**
     * calculates the internal energy for snakes displacement
     * @param currentLine : the actual line as jts LineString
     * @param alpha : Snakes parameter
     * @param beta : Snakes parameter
     * @return matrix of internal energy per point [E_int_x,E_int_y] 
     */
    public static Matrix calcInternalEnergy(LineString currentLine, double alpha, double beta){
    	int n =currentLine.getNumPoints();
    	Matrix mat = MatlabSyntax.zeros(n,2);
    	Matrix aPent = SnakesMatrixOperations.getPentaDiagConstParamMatrix(alpha,beta,n);
    	/**
    	* Skaling can not be used for calculation of E_int since in scaleMatrix() 
    	*   is 1 added to the diagonal elements if a matrix-row is equal to zero 
    	*   further scaleBorderOfMatrix changes the border colums which is not 
    	*   necessary since we like to use only changes of internal energy for displacement
    	**/ 
    	//   SnakesMatrixOperations.scaleMatrix(aPent);  [sstein] 04.08.2005
        //   SnakesMatrixOperations.scaleBorderOfMatrix(aPent);  
    	//-- new scaling funtion
    	SnakesMatrixOperations.scaleApentForIntNRG(aPent);
    	//--
    	Matrix xcoord = MatlabSyntax.zeros(n,1);
    	Matrix ycoord = MatlabSyntax.zeros(n,1);
    	for (int i = 0; i < n; i++) {
            Point p = currentLine.getPointN(i);
            xcoord.set(i,0,p.getX());
            ycoord.set(i,0,p.getY());
        } 
    	Matrix iex = MatlabSyntax.times(aPent,xcoord); 
    	Matrix iey = MatlabSyntax.times(aPent,ycoord);
        for (int i = 0; i < n; i++) {
            mat.set(i,0,iex.get(i,0));
            mat.set(i,1,iey.get(i,0));
        }      
        //-- show what happens
        //ShowSmoothingDx ss = new ShowSmoothingDx(xcoord,iex);
    	return mat;
    }    
    
    /**
     * calculates the over all energy for snakes displacement,
     * uses the summed absolute values of positive and negative energies. 
     * Does not take into account the used weigths according 
     * to network connections.
     * @param intEnergy
     * @param extEnergy
     * @return
     */
    public static double calcSumEnergy(Matrix intEnergy, Matrix extEnergy){
        double energy = 0;
        double t1, t2, t3, t4;
        Matrix iex = MatlabSyntax.abs((Matrix)intEnergy.getColumn(0));
        Matrix iey = MatlabSyntax.abs((Matrix)intEnergy.getColumn(1));
        Matrix eex = MatlabSyntax.abs((Matrix)extEnergy.getColumn(0));
        Matrix eey = MatlabSyntax.abs((Matrix)extEnergy.getColumn(1));
        t1 = iex.sum().toDouble();
        t2 = iey.sum().toDouble();
        t3 = eex.sum().toDouble();
        t4 = eey.sum().toDouble();
        energy = t1+t2+t3+t4;
        return energy;
    }
    
    /**
     * calculates the over all energy for snakes displacement,
     * uses the sum of the (positive and negative energies). 
     * Does not take into account the used weigths according 
     * to network connections.
     * @param intEnergy
     * @param extEnergy
     * @return
     */
    public static double calcIntegralEnergy(Matrix intEnergy, Matrix extEnergy){
        double energy = 0;
        double t1, t2, t3, t4;
        Matrix iex = MatlabSyntax.abs((Matrix)intEnergy.getColumn(0));
        Matrix iey = MatlabSyntax.abs((Matrix)intEnergy.getColumn(1));
        Matrix eex = MatlabSyntax.abs((Matrix)extEnergy.getColumn(0));
        Matrix eey = MatlabSyntax.abs((Matrix)extEnergy.getColumn(1));
        AbstractMatrix dex = iex.minus(eex);
        AbstractMatrix dey = iey.minus(eey);
        t1 = dex.sum().toDouble();
        t2 = dey.sum().toDouble();
        //energy = Math.abs(t1)+ Math.abs(t2);
        energy = Math.sqrt(t1*t1 + t2*t2);
        return energy;
    }
    
    /**
     * calculates the difference (old-new) of internal energy for snakes displacement
     * Since E_int should be larger with stronger deformation, the resulting dE_int
     * will be (negativ).
     * @param intEnergyOld : the internal energy of the previuos state in format [E_int_x,E_int_y]
     * @param intEnergyNew : the actual internal energy in format [E_int_x,E_int_y]
     * @return matrix of internal energy  difference per point [dE_int_x,dE_int_y] 
     */
    public static Matrix calcIntEnergyDiff(Matrix intEnergyOld, Matrix intEnergyNew){
    	//[sstein 23.04.2006] this is wrong according to the publications 
    	AbstractMatrix mat = intEnergyOld.minus(intEnergyNew); 
    	//AbstractMatrix mat = intEnergyNew.minus(intEnergyOld);
    	return (Matrix)mat;
    }
    
    /**
     * Calculates for a given list of displacement vector components for a given point
     * the displacement average/sum of the components. This method is usefull to get the
     * vector components for displacement of a building.<p>
     * Note: It is assumed that the ArrayList are of same size.  
     * @param dxDisp
     * @param dyDisp
     * @return the average displacement for a point [dx, dy]
     */
    public static double[] calcSumPointDisplacement(ArrayList dxDisp, ArrayList dyDisp){
        double[] dispComp = {0,0};
        int noVectors = dxDisp.size();
        if (noVectors > 0){
	        double sumdx=0, sumdy=0;
	        for (int i = 0; i < noVectors; i++) {
	            sumdx = sumdx + ((Double)dxDisp.get(i)).doubleValue();
	            sumdy = sumdy + ((Double)dyDisp.get(i)).doubleValue();
	        }
	        dispComp[0]=sumdx/((double)noVectors); //added type cast to be sure that result is double val
	        dispComp[1]=sumdy/((double)noVectors);
        }
        return dispComp;
    }
        
}

/*
final class ShowSmoothingDx extends JFrame{
    
	public ShowSmoothingDx(Matrix dxFirst, Matrix dxAfter){		
	
	//==============================	
	double[][] dataDxF = new double [dxFirst.getRowDimension()][2];
	double[][] dataDxA = new double [dxAfter.getRowDimension()][2];
	
	for (int j = 0; j < dataDxF.length; j++) {			
	dataDxF[j][0] = j;
	dataDxF[j][1] = dxFirst.get(j,0);
	dataDxA[j][0] = j;
	dataDxA[j][1] = dxAfter.get(j,0);
	}		
	
	//================================
	// Build the 2D scatterplot of the datas in a Panel
	// LINE, SCATTER, BAR, QUANTILE, STAIRCASE, (HISTOGRAMM?)		
	Plot2DPanel plot2dA = new Plot2DPanel();
	//====================
	plot2dA.addLinePlot("dxInit",dataDxF);
	plot2dA.addLinePlot("dxAfter",dataDxA);		
	//====================
	plot2dA.setAxisLabel(0,"pos vertex");
	plot2dA.setAxisLabel(1,"displacement");
	// Display a Frame containing the plot panel
	new FrameView(plot2dA);			      
	}   
}            
*/