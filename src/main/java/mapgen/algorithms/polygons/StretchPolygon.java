package mapgen.algorithms.polygons;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Polygon;

/**
 * description:
 *  stretches a polygon or Multi polygon;
 *  stretch does allow rotations,
 *
 * created on 		22 oct. 2004
 * @author marius (IGN)
 *
 */
public class StretchPolygon {
    
	/**
	 *  Stretch the Polygon along a direction with a given scale from a point.
	 *
	 *@param  geom the Polygon to stretch.
	 *@param  x0 the X coordinate of the point from where to stretch.
	 *@param  y0 the Y coordinate of the point from where to stretch.
	 *@param  angle the angle of the direction to stretch. (angle from the growing X axis)
	 *@param  scale the stretching scale.
	 */
	public static void stretchPolygon(Polygon geom, double x0, double y0, double angle, double scale){
		//rotate the polygon
		rotate(geom, -1.0*angle);
		//stretch the shell of the polygon along the X axis
		Coordinate[] coord=geom.getExteriorRing().getCoordinates();
		for(int i=0;i<coord.length;i++){			
			coord[i].x=x0+scale*(coord[i].x-x0);
		}
		//stretch the holes of the polygon along the X axis
		for(int j=0;j<geom.getNumInteriorRing();j++){
			Coordinate[] hole_coord=geom.getInteriorRingN(j).getCoordinates();
			for(int i=0;i<hole_coord.length;i++){
				hole_coord[i].x=x0+scale*(hole_coord[i].x-x0);
			}
		}
		//rotate back the polygon
		rotate(geom, angle);
	}
	
	/**
	 *  Stretch the MultiPolygon along a direction with a given scale from a point.
	 *
	 *@param  geom the MultiPolygon to stretch.
	 *@param  x0 the X coordinate of the point from where to stretch.
	 *@param  y0 the Y coordinate of the point from where to stretch.
	 *@param  angle the angle of the direction to stretch. (angle from the growing X axis)
	 *@param  scale the stretching scale.
	 */
	public static void stretchMultiPolygon(MultiPolygon geom, double x0, double y0, double angle, double scale){
		for(int i=0;i<geom.getNumGeometries();i++){
			stretchPolygon((Polygon)geom.getGeometryN(i), x0, y0, angle, scale);
		}
	}

	/**
	 *  Stretch the Polygon along a direction with a given scale from the center point.
	 *
	 *@param  geom the Polygon to stretch.
	 *@param  angle the angle of the direction to stretch. (angle from the growing X axis)
	 *@param  scale the stretching scale.
	 */
	public static void stretchPolygon(Polygon geom, double angle, double scale){
		stretchPolygon(geom, geom.getCentroid().getX(), geom.getCentroid().getY(), angle, scale);
	}

	/**
	 *  Stretch the MultiPolygon along a direction with a given scale from the center point.
	 *
	 *@param  geom the MultiPolygon to stretch.
	 *@param  angle the angle of the direction to stretch. (angle from the growing X axis)
	 *@param  scale the stretching scale.
	 */
	public static void stretchMultiPolygon(MultiPolygon geom, double angle, double scale){
		stretchMultiPolygon(geom, geom.getCentroid().getX(), geom.getCentroid().getY(), angle, scale);
	}

	public static void rotate(Polygon geom, double x0, double y0, double angle){
		//calculate it only one time
		double c=Math.cos(angle), s=Math.sin(angle);
		
		//rotate the shell of the polygon
		Coordinate[] coord=geom.getExteriorRing().getCoordinates();
		for(int i=0;i<coord.length;i++){
			double x=coord[i].x, y=coord[i].y;
			coord[i].x=x0+c*(x-x0)-s*(y-y0);
			coord[i].y=y0+s*(x-x0)+c*(y-y0);
		}
		
		//rotate the holes of the polygon
		for(int j=0;j<geom.getNumInteriorRing();j++){
			Coordinate[] coord2=geom.getInteriorRingN(j).getCoordinates();
			for(int i=0;i<coord2.length;i++){
				double x=coord2[i].x, y=coord2[i].y;
				coord2[i].x=x0+c*(x-x0)-s*(y-y0);
				coord2[i].y=y0+s*(x-x0)+c*(y-y0);
			}
		}
	}
	
	public static void rotate(MultiPolygon geom, double x0, double y0, double angle){
		for(int i=0;i<geom.getNumGeometries();i++){
			rotate((Polygon)geom.getGeometryN(i), x0, y0, angle);
		}
	}
	
	public static void rotate(Polygon geom, double angle){
		rotate(geom, geom.getCentroid().getX(), geom.getCentroid().getY(), angle);
	}
	
	public static void rotate(MultiPolygon geom, double angle){
		rotate(geom, geom.getCentroid().getX(), geom.getCentroid().getY(), angle);
	}

}
