/***********************************************
 * created on 		08.12.2004
 * last modified: 	
 * 
 * author:			sstein
 * 
 * description:
 *  Displacement of a line using a snakes algorihm.
 *  For use call first setParamsAndCalcMat() and call afterwards
 *  solve();
 *  A minimum displacement threshold of 0.05m per coordinate direction 
 *  is set, if disp. value is smaller  no point displacement will be done.
 * 
 * 
 * 
 ***********************************************/
package mapgen.algorithms.snakes;

import org.jmat.MatlabSyntax;
import org.jmat.data.AbstractMatrix;
import org.jmat.data.Matrix;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;

/**
 * @description
 *  Displacement of a line using a snakes algorihm.
 *  For use call first setParamsAndCalcMat() and call afterwards
 *  solve(); 
 *  A minimum displacement threshold of 0.05m per coordinate direction 
 *  is set, if disp. value is smaller  no point displacement will be done.
 * 
 * @author sstein
 *
 */
public class SnakesDisplacementLines{
    
    private Matrix Apent = null;
    private Matrix ApInv = null;
    private int networkstate = 0;
    private boolean matWasCalculated = false;
    
    /**
     * initialize Snakes Calculation Object 
     *
     */
    public SnakesDisplacementLines(){
        this.matWasCalculated = false; 
    }

    /**
     * set necessary values and calculate the pentadiagonal matrix
     * @param numberOfVertices of the Line to displace
     * @param alpha snakes parameter (should be one)
     * @param beta snakes parameter
     * @param networkState ( 0 = free line, 1 = fix startpoint, 
     *                       2= fix end point, 3 = fix start and endpoint)
     */
    public void setParamsAndCalcMat(int numberOfVertices, 
             				 double alpha, double beta, 
                             int networkState){
        if(numberOfVertices > 5){
	        this.networkstate = networkState;
	        this.Apent = SnakesMatrixOperations.getPentaDiagConstParamMatrix(alpha,beta,numberOfVertices);
	        SnakesMatrixOperations.scaleMatrix(this.Apent);
	        SnakesMatrixOperations.scaleBorderOfMatrix(this.Apent);
	        this.ApInv = (Matrix)this.Apent.inverse();        
	        this.matWasCalculated = true;
        }
        else{
            System.out.println("SnakesDisplacement.setParams : can't calc A_pent, since line has less than 6 verticies");
        }
    }
    
    /**
     * calculates the displacement
     * @param line to displace 
     * @param extEnergy is equal to the displacement vector, 
     *         get the two column matrix [dx_i,dy_i] from 
     *         SnakesEnergyDisplacement class
     * @param intEnergy is equal to the already displaced point
     *         distances, get the two column matrix [dx_i,dy_i] from 
     *         SnakesEnergyDisplacement class
     * @return displaced line
     */
    public LineString solve(LineString line, Matrix extEnergy, Matrix intEnergy){
        
        LineString newLine = (LineString)line.copy();
        
        if(this.matWasCalculated == false){
            System.out.println("SnakesDisplacement: could not calculate, since Apent is missing!");
        }
        else{
	        int n = line.getNumPoints();
	        Matrix weights = MatlabSyntax.ones(n,1);
/**
	        if(this.networkstate == 0){ // no network
	            // all weights are 1
	        }
	        else if(this.networkstate == 1){ // network on line start
	            weights.set(0,0,0);
	            //weights.set(1,0,1.0/3.0);
	            //weights.set(2,0,2.0/3.0);
	        }
	        else if(this.networkstate == 2){ // network on line end
	            weights.set(n-1,0,0);
	            //weights.set(n-2,0,1.0/3.0);
	            //weights.set(n-3,0,2.0/3.0);            
	        }
	        else if(this.networkstate == 3){ // network on line start and end
	            weights.set(0,0,0);
	            //weights.set(1,0,1.0/3.0);
	            //weights.set(2,0,2.0/3.0);
	            weights.set(n-1,0,0);
	            //weights.set(n-2,0,1.0/3.0);
	            //weights.set(n-3,0,2.0/3.0);                        
	        }
**/	         
	        //System.out.println("SnakesDisplacement.solve(): for test reason all weights set to one => networkstate not considered!!!");
	        
	        AbstractMatrix iex = intEnergy.getColumn(0);
	        AbstractMatrix iey = intEnergy.getColumn(1);
	        AbstractMatrix eex = extEnergy.getColumn(0);	        
	        AbstractMatrix eey = extEnergy.getColumn(1);	        
	        AbstractMatrix dex = iex.minus(eex);
	        AbstractMatrix dey = iey.minus(eey);
	        Matrix dx = MatlabSyntax.times(this.ApInv,(Matrix)dex);           
	        Matrix dy = MatlabSyntax.times(this.ApInv,(Matrix)dey);	        
	                
	        Coordinate[] coord= newLine.getCoordinates();
	        double pdx, pdy;
	        for (int i = 0; i < n; i++) {
	            pdx = dx.get(i,0)*weights.get(i,0);
	            pdy = dy.get(i,0)*weights.get(i,0);
	            // check to avoid small unnecessary changes
	            if (Math.abs(pdx) > 0.05){
	                //System.out.println("SnakesDisplacement class: verbessere coordinate x von punkt " + i);
	                coord[i].x=coord[i].x+pdx;
	            }
	            if (Math.abs(pdy) > 0.05){
	                //System.out.println("SnakesDisplacement class: verbessere coordinate y von punkt " + i);	                
	                coord[i].y=coord[i].y+pdy;
	            }
	        }        
        }
        return newLine;
    }
    
}
