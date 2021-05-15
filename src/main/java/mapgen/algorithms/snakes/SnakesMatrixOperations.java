/***********************************************
 * created on 		09.12.2004
 * last modified: 	14.12.2004
 * 					05.08.2005 scaleBorderOfMatrix for calculation of E_Int
 * 
 * author:			sstein
 * 
 * description:
 * class contains static functions which are necessary 
 * for the snakes algorithm
 * 
 * sources: 
 *  - Kass et al. 1987,
 *  - Meier and Burghardt 1997,
 *  - Borkwoski, Burghardt and Meier, 1999
 *  - Steiniger 2003 (german diploma thesis)
 *  - Steiniger and Meier 2004    
 * 
 * @TODO: implement tafusRestrictions 
 ***********************************************/
package mapgen.algorithms.snakes;

import org.jmat.MatlabSyntax;
import org.jmat.data.AbstractMatrix;
import org.jmat.data.Matrix;

/**
 * @TODO: implement tafusRestrictions
 *  
 * @description: 
 *  class contains static functions which are necessary 
 *  for the snakes algorithm:
 *  - Kass et al. 1987,
 *  - Meier and Burghardt 1997,
 *  - Borkwoski, Burghardt and Meier, 1999
 *  - Steiniger 2003 (german diploma thesis)
 *  - Steiniger and Meier 2004   
 *   
 * @author sstein
 */
public class SnakesMatrixOperations{

    /**
     * calculates the pentadiagonal snakes matrix 
     * @param alpha snakes parameter (should ne one)
     * @param beta snakes parameter (should be >= 1)
     * @param size equals number of line vertices
     * @return the matrix of dimension [size,size]
     */
    public static Matrix getPentaDiagConstParamMatrix(double alpha, double beta, int size){
        
        Matrix pentMat = MatlabSyntax.zeros(size, size);
        //-- set diagonal element values
        double a=2*alpha+6*beta; 
        double b=-1*alpha-4*beta;
        double c=beta;
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                //-- set main diagonal
                if(j == i ){pentMat.set(i,j,a);}
                //-- set main left inner diagonal
                if(j == i - 1){pentMat.set(i,j,b);}                
                //-- set main right inner diagonal
                if(j == i +1){pentMat.set(i,j,b);}
                //-- set main left inner+1 diagonal
                if(j == i - 2){pentMat.set(i,j,c);}                
                //-- set main right inner+1 diagonal
                if(j == i + 2){pentMat.set(i,j,c);}                
            }
        }
        //System.out.println("-- pentadiagonal Mat with const coeffs ---- ");
	    //printMatrix(pentMat);
        
        return pentMat;
    }
    
    /**
     * calculates a pentadiagonal snakes matrix with row varying elements
     * (discretisation after Kass et al. 1087; alpha beta might be 
     *  curvature dependend curvature)   
     * @param alpha const snakes parameter (should ne one)
     * @param beta varying snakes parameter  
     * @param size equals number of line vertices
     * @return pentadiagonal matrix
     * 
     * @TODO: test this
     */
    public static Matrix getPentaDiagVarParamMatrix(double alpha, double[] beta, int size){
        
        /*
        size=10;
        double[] al ={1}; 
        double[] be={10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
        alpha = al;
        beta = be;
        */
        
        Matrix pentMat = MatlabSyntax.zeros(size, size);
        //-- calc coefficients from varying parameters
        // a(1) laesst sich nicht berechnen, genauso wie a(n)
        double[] a = new double[size];
        double[] bL = new double[size]; double[] bR = new double[size];
        double[] cL = new double[size]; double[] cR = new double[size];
        //-- source  s. steiniger and s. meier (2004)
        //----- main diagonal ----
        for(int i = 1; i < size-1; i++){ 
            a[i]=2*alpha+ beta[i-1]+ 4*beta[i]+beta[i+1];
        }
        // replacement, since not able to calculate
        a[1]=(double)a[2]; a[size-1]=a[size-2];
        //----- inner diagonals ----
        for(int i = 1; i < size; i++){
            bL[i-1]=-1*alpha-2*beta[i-1]-2*beta[i];
        }
        for(int i = 0; i < size-1; i++){
            bR[i]=-1*alpha-2*beta[i]-2*beta[i+1];
        }        
        //----- inner+1 diagonals ----        
        for(int i = 2; i < size; i++){            
            cL[i-2]=beta[i-1];
        }
        for(int i = 0; i < size-2; i++){            
            cR[i-2]=beta[i+1];
        }        
        //---- set diagonal element values -----        
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                //-- set main diagonal
                if(j == i ){pentMat.set(i,j,a[i]);}
                //-- set main left inner diagonal
                if(j == i - 1){pentMat.set(i,j,bL[i]);}                
                //-- set main right inner diagonal
                if(j == i +1){pentMat.set(i,j,bR[i]);}
                //-- set main left inner+1 diagonal
                if(j == i - 2){pentMat.set(i,j,cL[i]);}                
                //-- set main right inner+1 diagonal
                if(j == i + 2){pentMat.set(i,j,cR[i]);}                
            }
        }
        //System.out.println("-- pentadiagonal Mat with variable coeffs ---- ");
	    //printMatrix(pentMat);
        
        return pentMat;
    }

    /**
     * calculates the tridiagonal tafus matrix from Borkowski, Burghardt & Meier(1999)
     * with constant parameters
     * @param alpha tafus parameter (should ne one)
     * @param beta tafus parameter (should be >= 1)
     * @param size equals number of line vertices (-1 ?? since TAF function )
     * @return tridiagonal tafus matrix
     * 
     * @TODO: test this 
     */
    public static Matrix getConstTriDiagMatrix(double alpha, double beta, int size){
        
        Matrix triMat = MatlabSyntax.zeros(size, size);
        
        double a=alpha+2*beta; 
        double b=-1*beta;
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                //-- set main diagonal
                if(j == i ){triMat.set(i,j,a);}
                //-- set main left inner diagonal
                if(j == i - 1){triMat.set(i,j,b);}                
                //-- set main right inner diagonal
                if(j == i +1){triMat.set(i,j,b);}
            }
        }
        //System.out.println("-- tridiagonal Mat with const coeffs ---- ");
	    //printMatrix(triMat);

        return triMat;
    }

    /**
     * calculates the tridiagonal tafus matrix for forward(!) differences,
     * source: Sigfried Meier (TU Dresden)
     * 
     * @param alpha varying tafus parameter (should ne one) 
     * @param beta varying tafus parameter
     * @param size of matrix equal to number of vertices (-1 ?? since TAF function)
     * @return tridiagonal varying tafus matrix
     * 
     * @TODO: test this
     */
    public static Matrix getVarTriDiagMatrix(double[] alpha, double[] beta, int size){

        /*
        size=10;
        double[] al ={1, 2, 3, 4, 5, 6, 7, 8, 9, 10}; 
        double[] be={10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
        alpha = al;
        beta = be;
        */
        
        //-- restriction to avoid nummerical instabilities
        System.out.println("Snakes build ATri: Restriction beta < 0.5 is working");
        for (int i=0; i < size; i++){
            if(beta[i] < 0.5){beta[i]=0.5;}
            System.out.println("Snakes build ATri: Restriction beta < 0.5 reset of beta");
        }

        Matrix triMat = MatlabSyntax.zeros(size, size);
        
        //-- calc coefficients from varying parameters
        double[] a = new double[size];
        double[] bL = new double[size]; double[] bR = new double[size];
        //-- main diagonal
        for (int i = 0; i < size-1; i++){
            a[i]=alpha[i]+beta[i]+beta[i+1];
        }
        //since last element can not be calculated
        a[size-1]=alpha[size-1]+2*beta[size-1];
        //-- inner diagonals
        for (int i = 0; i < size-1; i++){
            bR[i]=-1*beta[i+1];
        }
        // since last element can not be calculated
        bR[size-1] = -1*beta[size-1];        
        for(int i = 1; i < size; i++){
            bL[i-1]=-1*beta[i];
        }        
        //build matrix
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                //-- set main diagonal
                if(j == i ){triMat.set(i,j,a[i]);}
                //-- set main left inner diagonal
                if(j == i - 1){triMat.set(i,j,bL[i]);}                
                //-- set main right inner diagonal
                if(j == i + 1){triMat.set(i,j,bR[i]);}
            }
        }
        //System.out.println("-- tridiagonal Mat with varying coeffs ---- ");
	    //printMatrix(triMat);

        return triMat;
    }
    
    /**
     * scales the element sum of the input matrix row to One. <p>
     * this is particular equal to a regularisation.
     * For tafus set alpha = 1 instead of using this scaling function.   
     * 
     * @param myMat snakes matrix to scale
     */
    public static void scaleMatrix(Matrix myMat){

	     //System.out.println("----- inputMat---- ");
	     //printMatrix(myMat);
	     
        double lambda = 1.0; //regularisation value
        
	     int length = myMat.getRowDimension();
	     double rowsum = 0;
	     double newValue = 0;
	     for (int i = 0; i < length; i++) {
	         // get matrix row 
	         AbstractMatrix row = myMat.getRow(i);
	         // transpose to vector [n,1] (is only temporary)and	         
	         // get sum of one row (now colum)
	         rowsum = row.transpose().sum().toDouble();
	         // make regularisation of main diagonal element if necessary 
	         if(Math.abs(rowsum) < 0.001){
	             newValue = myMat.get(i,i) + lambda;
	             myMat.set(i,i,newValue);	             
	         }
	         else{ //if no regularisation necessary then do scaling
	             AbstractMatrix rowD = myMat.getRow(i).divide(rowsum);
	             myMat.setRow(i,rowD);
	         }
	     }
	     
	     //System.out.println("----- changed rows---- ");
	     //printMatrix(myMat);
        
	     
    }
    
    /**
     * Scales the edges (first and last both rows) for
     * the pentadiagonal snakes matrix. Sum of row elements will be one.
     * Function is intend for matrix const parameters.
     * @param D matrix to scale the edge elements
     */
    public static void scaleBorderOfMatrix(Matrix D){
        
        //System.out.println("----- inputMat---- ");
	    //printMatrix(D);
        
        int n =D.getColumnDimension()-1;
        
        double s1=(1-D.get(2,2)) / (D.get(0,1)+D.get(0,2));
        double s2=(1-D.get(2,2)) / (D.get(1,0)+D.get(1,2)+D.get(1,3));
        //first row
        D.set(0,0,D.get(2,2));
        D.set(0,1,D.get(0,1)*s1);
        D.set(0,2,D.get(0,2)*s1);
        //second row
        D.set(1,1,D.get(2,2));
        D.set(1,0,D.get(1,0)*s2);
        D.set(1,2,D.get(1,2)*s2);
        D.set(1,3,D.get(1,3)*s2);
        
        double s3=(1-D.get(n-2,n-2)) / (D.get(n,n-1)+D.get(n,n-2));
        double s4=(1-D.get(n-2,n-2)) / (D.get(n-1,n)+D.get(n-1,n-2)+D.get(n-1,n-3));
        //last row diag element
        D.set(n,n,D.get(n-2,n-2));
        //last row -1 diag element
        D.set(n-1,n-1,D.get(n-2,n-2));
        //last row
        D.set(n,n-1,D.get(n,n-1)*s3);
        D.set(n,n-2,D.get(n,n-2)*s3);
        //last row -1
        D.set(n-1,n,D.get(n-1,n)*s4);
        D.set(n-1,n-2,D.get(n-1,n-2)*s4);
        D.set(n-1,n-3,D.get(n-1,n-3)*s4);
                
        //System.out.println("-- border scaled Matrix  ---- ");
	    //printMatrix(D);
    }

    /**
     * Scales the edges (first and last both rows) of
     * the pentadiagonal snakes matrix for calculation of internal Energy.
     * The sum of the matrix row elements will be Zero.  
     * Function is intend for matrix const parameters.
     * @param D matrix to scale the edge elements
     */
    public static void scaleApentForIntNRG(Matrix D){
        
        //System.out.println("----- inputMat---- ");
	    //printMatrix(D);
        
        int n =D.getColumnDimension()-1;
        
        //first row
        D.set(0,1,D.get(0,1)*2);
        D.set(0,2,D.get(0,2)*2);
        //second row
        D.set(1,3,D.get(1,3)*2);
        
        //last row
        D.set(n,n-1,D.get(n,n-1)*2);
        D.set(n,n-2,D.get(n,n-2)*2);
        //last row -1
        D.set(n-1,n-3,D.get(n-1,n-3)*2);
                
        //System.out.println("-- border scaled Matrix  ---- ");
	    //printMatrix(D);
    }

    public static void tafusRestrictions(){
        
    }
    
    /**
     * prints the input matrix to console
     * @param myMat
     */
    public static void printMatrix(Matrix myMat){
        
	     for (int i = 0; i < myMat.getRowDimension(); i++) {
	         for (int j = 0; j < myMat.getColumnDimension(); j++) {
	               System.out.print(myMat.get(i,j) + "  ");                
	         }          
	            System.out.println(" ");
	     }
	     System.out.println("-----");
    }

    public static Matrix mirror1D(Matrix coords1d, int nrOfOrigVertices, int pointsPerSide){
        int n = nrOfOrigVertices;
        int newSize = (int)(n + 2* pointsPerSide);
        Matrix matMirror = MatlabSyntax.zeros(newSize,1);
        //-- fill matrices 
        double x,y; 
        for (int i = 0; i < n; i++) {
            x = coords1d.get(i,0);
            matMirror.set(i+pointsPerSide,0,x);           
        }          
        //-- mirror first points
        for (int i = 0; i < pointsPerSide; i++) {
            x= coords1d.get(0,0) + coords1d.get(0,0) - coords1d.get(pointsPerSide-i,0);
            matMirror.set(i,0,x);
        }        
        //-- mirror last points
        for (int i = 0; i < pointsPerSide; i++) {
            x= coords1d.get(n-1,0) + coords1d.get(n-1,0) - coords1d.get(n-1-pointsPerSide+i,0);
            matMirror.set(newSize-1-i,0,x);           
        }                              
        return matMirror;
     }
    
    public static Matrix extractOriginalLineLength1D(Matrix coords1d, int nrOfOrigVertices, int nrOfMirroredPoints){        
        Matrix mat =  MatlabSyntax.zeros(nrOfOrigVertices,1);
        double x;
        for (int i = 0; i < nrOfOrigVertices; i++) {
            x = coords1d.get(i+ nrOfMirroredPoints,0);
            mat.set(i,0,x);            
        }            
        return mat;
    }
    
    public static Matrix extendWithZeros1D(Matrix coords1d, int nrOfOrigVertices, int pointsPerSide){
        int n = nrOfOrigVertices;
        int newSize = (int)(n + 2* pointsPerSide);
        Matrix matMirror = MatlabSyntax.zeros(newSize,1);
        //-- fill matrices 
        double x,y; 
        for (int i = 0; i < n; i++) {
            x = coords1d.get(i,0);
            matMirror.set(i+pointsPerSide,0,x);           
        }          
        /*
        //-- mirror first points
        for (int i = 0; i < pointsPerSide; i++) {
            x= coords1d.get(0,0) + coords1d.get(0,0) - coords1d.get(pointsPerSide-i,0);
            matMirror.set(i,0,x);
        }        
        //-- mirror last points
        for (int i = 0; i < pointsPerSide; i++) {
            x= coords1d.get(n-1,0) + coords1d.get(n-1,0) - coords1d.get(n-1-pointsPerSide+i,0);
            matMirror.set(newSize-1-i,0,x);           
        } 
        */                             
        return matMirror;
     }
    
    /**
     * Calculates the signum of the matrix elements. If the value is zero
     * the signum will be +1.
     * @param inMatrix
     * @return matrix containing values of -1 : e < 0 and +1 : e >= 0
     */
    public static Matrix calcSignum(Matrix inMatrix){
    	int n=inMatrix.getRowDimension();
    	int m=inMatrix.getColumnDimension();
    	Matrix outMatrix = MatlabSyntax.zeros(n,m);
    	for(int i=0; i < n; i++){
    		for(int j=0; j < m; j++){
    			if (inMatrix.get(i,j) >= 0){
    				outMatrix.set(i,j,1);
    			}    		    
    			if (inMatrix.get(i,j) < 0){
    				outMatrix.set(i,j,-1);
    			}
    		}
    	}
    	return outMatrix;
    }
    
}
