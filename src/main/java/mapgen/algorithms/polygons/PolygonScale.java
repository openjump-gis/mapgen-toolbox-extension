package mapgen.algorithms.polygons;


import org.locationtech.jts.geom.Polygon;

/**
 *  Scales a polygon along x and y axis
 *  with the polygon centroid as center. <p>
 *  Uses StretchPolygon.stretchPolygon() and StretchPolygon.rotate() methods  
 *
 * created on 		09.01.2005
 * @author sstein
 *
 */
public class PolygonScale {
       
    /**
     *  Scales a polygon along x and y axis
     *  with the polygon centroid as center. 
     *
     * @param geom
     * @param scale factor (1 = no scaling)
     */
	public static void scalePolygon(Polygon geom, double scale){
	    //-- scale on horizontal axis
	    StretchPolygon.stretchPolygon(geom,0,scale);
	    //-- scale on vertical axis
	    StretchPolygon.stretchPolygon(geom,Math.PI/2.0,scale);	    
	}
}
