package mapgen.geomutilities;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.Polygon;

/**
 *
 * created on 		13.01.2005
 * @author sstein
 */
public class ModifyPolygonPoints {

    /**
     * Deletes a point in a polygon. The point can be part of a hole.
     * @param inPoly input Polygon
     * @param ringIndex no of ring which contains the point (ExteriorRing has index=0
     * 		  and interiorRings have index=InteriorIndex + 1)
     * @param pointIndex (index of ring coordinates)
     * @return the input Polygon without the point of index pointIndex in ringIndex
     */
    public static Polygon deletePoint(Polygon inPoly, int ringIndex, int pointIndex){
        //-------------------
        // copy Geometry parts
        //--------------------
        GeometryFactory gf = new GeometryFactory();
        //-- exteriorRing
        Coordinate[] coordsE = inPoly.getExteriorRing().getCoordinates();
        LinearRing exteriorRing = gf.createLinearRing(coordsE);
        //-- interiorRings
        LinearRing[] interiorRings = null;        
        int nrRings = inPoly.getNumInteriorRing();
        if (nrRings > 0){
            interiorRings = new LinearRing[nrRings];
            for (int i = 0; i < nrRings; i++) {
                Coordinate[] coordsI = inPoly.getInteriorRingN(i).getCoordinates();
                interiorRings[i] = gf.createLinearRing(coordsI);
            }
        }                                      
        //-------------------------------
        // modify Geometrie - delete Point
        //------------------------------
        //-- modify exterior Ring 
        if (ringIndex == 0){ 
            Coordinate[] newCoords = new Coordinate[coordsE.length -1];
            //-- check if at more than three points
            if(coordsE.length < 4){
            	System.out.println("ModifyPolygonPoint.delete: cant delete Point!!! Otherwise Polygon disappears");
            }
            else{
	            //-- check if starting point or last point should be modified
	            if ((pointIndex == 0) || (pointIndex == (coordsE.length -1))){
	            	//delete first
	            	if (pointIndex == 0){
	    	            //int idx=0;
	    	            for (int i = 0; i < newCoords.length-1; i++) {	                
	    	                newCoords[i] = coordsE[i+1];
	    	            }
	    	            newCoords[newCoords.length-1] = newCoords[0];            		
	            	}
	            	//delete last
	            	if (pointIndex == (coordsE.length -1)){
	    	            //int idx=0;
	    	            for (int i = 1; i < newCoords.length; i++) {	                
	    	                newCoords[i-1] = coordsE[i];    	                
	    	            }
	    	            newCoords[newCoords.length-1] = newCoords[0];            		
	            	}            	
	            }
	            else{
		            int idx=0;
		            for (int i = 0; i < coordsE.length; i++) {
		                if (i != pointIndex){
		                    newCoords[idx] = coordsE[i];
		                    idx=idx+1;
		                }
		            }
	            }
	            exteriorRing = gf.createLinearRing(newCoords);
            }
        }
        //-- modify interior Ring
        if (ringIndex > 0){
            if(interiorRings[ringIndex-1].getNumPoints() < 5){ //limit is 5 since first=last = 2 points in list 
            	System.out.println("ModifyPolygonPoint.delete: delete 3rd Point!!! inner Ring vanishs");
            	//-- delete inner ring
            	if (interiorRings.length > 1){
	                interiorRings = new LinearRing[nrRings-1];
	                int ridx=0;
	                for (int i = 0; i < nrRings; i++) {
	                	if (i != (ringIndex-1)){
	                		Coordinate[] coordsI = inPoly.getInteriorRingN(i).getCoordinates();
	                		interiorRings[ridx] = gf.createLinearRing(coordsI);
	                		ridx = ridx+1;
	                	}
	                }
            	}
            	else{//only one inner ring
            		interiorRings = null; 
            	}
            }
            else{        	
	            Coordinate[] coordsOld = interiorRings[ringIndex-1].getCoordinates();
	            Coordinate[] newCoords = new Coordinate[coordsOld.length -1];
	            //-- check if starting point should be modified
	            if ((pointIndex == 0) || (pointIndex == (coordsOld.length -1))){
		            int idx=0;
		            for (int i = 1; i < coordsOld.length-1; i++) {	                
		                newCoords[idx] = coordsOld[i];
		                idx=idx+1;
		            }
		            newCoords[newCoords.length-1] = coordsOld[1];
	            }
	            else{            
	                int idx=0;
	                for (int i = 0; i < coordsOld.length; i++) {
	                    if (i != pointIndex){
	                        newCoords[idx] = coordsOld[i];
	                        idx=idx+1;
	                    }
	                }
	            }
	            LinearRing newIntRing = gf.createLinearRing(newCoords);
	            interiorRings[ringIndex-1]=newIntRing;
            }
        }
        //-- new OutputPolygon
			return gf.createPolygon(exteriorRing,interiorRings);
    }
}
