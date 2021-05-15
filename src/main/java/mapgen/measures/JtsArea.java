package mapgen.measures;

import java.util.Collection;
import java.util.Iterator;

import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.Polygon;

/**
 *
 * description:
 * calculate the area for one or more objects.
 * For polygons area of outerring (complete without holes)
 * and holes are calculated, recieved with getPolyAreas().
 *
 * created on 		20.09.2004
 * last modified: 	07.10.2004
 * @author sstein
 */
public class JtsArea {

    private double area;
    private double nrObjects;
    private double[] polyAreas;
    private boolean PolyHasHoles = false;
    
    /**
     * calculate areas for one ore more (selected) (JTS) geometry Objects
     * in a collection using JTS lib
     * @param geometries input geometries
     */
    public JtsArea(Collection<Geometry> geometries) {

        double overallArea = 0.0;

        for (Geometry geometry : geometries) {
            overallArea = overallArea + geometry.getArea();
        }

        this.setNrObjects(geometries.size());
        this.setArea(overallArea);                
    }

    /**
     * calculate the area of one geometry object,
     * value as given back by JTS. Value can be 
     * recieved by getArea();
     * @param myGeometry
     */
    public JtsArea(Geometry myGeometry) {

        double area = myGeometry.getArea();        
        this.setNrObjects(1);
        this.setArea(area);       
    }

    /**
     * calculates separate areas for exterior ring 
     * and holes, which can be recived by getPolyAreas(); 
     * @param myPolygon
     */
    public JtsArea(Polygon myPolygon) {

        this.setNrObjects(1);
        int nrOfHoles = myPolygon.getNumInteriorRing();
        if ( nrOfHoles > 0){ 
            //--with holes
            this.PolyHasHoles = true;
            this.polyAreas = new double[nrOfHoles + 1];
            //-- calc area for exterior ring            
            LineString outerring = myPolygon.getExteriorRing();            
            GeometryFactory myGF = new GeometryFactory();
            LinearRing lr = myGF.createLinearRing(outerring.getCoordinates());
            Polygon helpy = myGF.createPolygon(lr, null);
            this.polyAreas[0] = helpy.getArea();
            //-- calc area for holes
            for (int i=0; i < nrOfHoles; i++){                
                LineString iring = myPolygon.getInteriorRingN(i);
                lr = myGF.createLinearRing(iring.getCoordinates());
                helpy = myGF.createPolygon(lr, null);
                this.polyAreas[i+1] = helpy.getArea();
            }
        System.gc();
                
        }
        else{ 
            //--no holes
            this.polyAreas = new double[1];
            this.polyAreas[0] = myPolygon.getArea();
            this.area = this.polyAreas[0];
        }
        
        
    }
    
    /**
     * @return Returns the area.
     */
    public double getArea() {
        return area;        
    }
    /**
     * @param area The area to set.
     */
    private void setArea(double area) {
        this.area = area;
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
     * gives back the polygon areas,
     * the value for the exterior ring is saved on index[0]  
     * 
     * @return one dim double array with area value 
     */
    public double[] getPolyAreas() {
        return polyAreas;
    }
    
    /**
     * 
     * @return true if single polygon (constructor: area(polygon))
     * was analysed and it has holes 
     * 
     */
    public boolean hasPolyHoles() {
        return PolyHasHoles;
    }
}
