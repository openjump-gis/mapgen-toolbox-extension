
/*****************************************************
 * created:  		23.11.2005
 * last modified:  	
 * 
 * @author sstein
 * 
 * description:
 *   the class is used to work with indexes from vertices
 *   of a closed polygon.
 *****************************************************/
package mapgen.algorithms;

/**
 * @author sstein
 *
 */
public class PolyRingIndex {

	public int i=0;
	private int noOfVertices = 0;
	
	public PolyRingIndex(int index, int noOfRingPoints){
		this.i = index;
		this.noOfVertices =noOfRingPoints;
	}
	
	public int getNext(){
		int v = this.i+1;
		if (this.i == this.noOfVertices-1){
			v = 1;
		}
		return v;
	}

	public int getNN(){
		int v = this.i+2;
		if (this.i == this.noOfVertices-1){
			v = 2;
		}
		if (this.i== this.noOfVertices-2){
			v = 1;
		}		
		return v;
	}

	public int getPrevious(){
		int v = this.i-1;
		if (this.i == 0){
			v = this.noOfVertices-2;
		}
		return v;
	}
	
	public int getPP(){
		int v = this.i-2;
		if (this.i == 0){
			v = this.noOfVertices-3;
		}
		if (this.i == 1){
			v = this.noOfVertices-2;
		}		
		return v;
	}
}
