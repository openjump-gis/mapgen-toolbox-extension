package mapgen.measures;

import java.util.Collection;
import java.util.Iterator;

import org.locationtech.jts.algorithm.ConvexHull;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryCollection;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;

/**
 * Calculate the centroids (convex hull and gravity) for one or more objects
 * (if more objects are given, then the centroid for conevex hull is created
 *
 * created on 		20.09.2004
 * last modified: 	21.09.2004
 * @author sstein 
 */
public class JtsCentroid {

    private Point gravityCentroid;
    private Point hullCentroid;
    private double nrObjects;
    
    /**
     * Calculate centroids for the convex hull and gravity of one ore
     * more (selected) (JTS) geometry Objects
     * in a collection using JTS lib
     *	
     * @param geometries
     */
    public JtsCentroid(Collection geometries) {
    
        int counter = 0;        
        
        // create array of geometries
        int arraylength = geometries.size();
        Geometry[] geomObjects = new Geometry[arraylength];       
        for (Iterator i = geometries.iterator(); i.hasNext();) {            
            // get single geometrie from envelope
            geomObjects[counter] = (Geometry) i.next();
            counter = counter+1;
        }                
        GeometryFactory geometryFactory = new GeometryFactory();
        // create GeometrieCollection from array to process union polygon
        GeometryCollection geomCollection = geometryFactory.createGeometryCollection(geomObjects);
        
        // join polygons (are one object, but still x areas => no aggregation) 
        Geometry union = geomCollection.buffer(0);        
        // calculate convex hull
        ConvexHull myConvexHull = new ConvexHull(union);
        // return convex hull
        Geometry myHullGeometry = myConvexHull.getConvexHull(); 
        // calculate centroid for convex hull
        Point centroid1 = myHullGeometry.getCentroid();
        this.setHullCentroid(centroid1);
        
        //centroid method for the collection => center of gravity 
        Point centroid2 = geomCollection.getCentroid(); 
        this.setGravityCentroid(centroid2);
        
    }

    /**
     * calculate the centroids of one geometry object
     * @param myGeometry
     */
    public JtsCentroid(Geometry myGeometry) {
        
        // calculate  gravity centroid
        Point centroid1 = myGeometry.getCentroid();
        this.setNrObjects(1);
        this.setGravityCentroid(centroid1);
        
        // calculate convex hull
        ConvexHull myConvexHull = new ConvexHull(myGeometry);        
        Geometry myHullGeometry = myConvexHull.getConvexHull();
        // calculate centroid for convex hull
        Point centroid2 = myHullGeometry.getCentroid();
        this.setHullCentroid(centroid2);
        
    }

    
    /**
     * @return Returns the centroid.
     */
    public Point getGravityCentroid() {
        return this.gravityCentroid;        
    }
    /**
     * @param centroid The centroid to set.
     */
    private void setGravityCentroid(Point centroid) {
        this.gravityCentroid = centroid;
    }
    /**
     * @return Returns the nrObjects.
     */
    public double getNrObjects() {
        return nrObjects;
    }
    /**
     * @param nrObjects The nrObjects to set.
     */
    private void setNrObjects(double nrObjects) {
        this.nrObjects = nrObjects;
    }
    /**
     * @return Returns the hullCentroid.
     */
    public Point getHullCentroid() {
        return hullCentroid;
    }
    /**
     * @param hullCentroid The hullCentroid to set.
     */
    private void setHullCentroid(Point hullCentroid) {
        this.hullCentroid = hullCentroid;
    }
}
