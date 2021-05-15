/***********************************************
 * created on 		17.12.2004
 * last modified: 	
 * 
 * author:			sstein
 * 
 * description:
 * 
 * 
 ***********************************************/
package mapgen.geomutilities;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;

/**
 * @author sstein
 *
 */
public class SplitLineString {
    
    /**
     * splits a linestring in parts and returns the parts as LineStrings
     * @param originalLine input LineString
     * @param pointIndizes index of points where to split 
     * 		  (without first and last point index)
     * @return Array of LineStrings
     */
    public static LineString[] splitInSegments(LineString originalLine, int[] pointIndizes){                
        int parts = pointIndizes.length + 1;
        int[] indizes = new int[parts+1];
        indizes[0]=0;
        indizes[parts]= originalLine.getNumPoints()-1;
        for (int i = 1; i < indizes.length-1; i++) {
            indizes[i]=pointIndizes[i-1];            
        }        
        GeometryFactory gf = new GeometryFactory();
        LineString[] segments = new LineString[parts];
        //double x,y;
        for (int i = 0; i < parts; i++) {            
            int idx=0;
            int s= indizes[i+1] - indizes[i] +1;
            Coordinate[] coords = new Coordinate[s];
            for (int j = indizes[i]; j < indizes[i+1]+1; j++) {                
                coords[idx] = originalLine.getCoordinateN(j);                
                idx= idx+1;
            }                     
            LineString ls = gf.createLineString(coords);
            segments[i] = (LineString)ls.copy();
        }                
        return segments;
    }
    
    public static LineString concatSegements(LineString[] segments){        
        int parts = segments.length;
        int slength = 0;
        for (int i = 0; i < parts; i++) {
            LineString temp = segments[i];
            slength = slength + (temp.getNumPoints() -1);
        }        
        slength = slength+1; //for start point
        Coordinate[] coords = new Coordinate[slength];
        //starts not with first point
        int k=1;
        for (int i = 0; i < parts; i++) {
            LineString temp = segments[i];
            for (int j = 1; j < temp.getNumPoints(); j++) {
                coords[k] = temp.getCoordinateN(j);                 
                k=k+1;
            }
        }
        //-- set first Point
        LineString temp = segments[0];
        coords[0] = temp.getCoordinateN(0);        
       
        GeometryFactory gf = new GeometryFactory();
        return gf.createLineString(coords);
    }

}
