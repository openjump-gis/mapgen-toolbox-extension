/*
 * The Unified Mapping Platform (JUMP) is an extensible, interactive GUI 
 * for visualizing and manipulating spatial features with geometry and attributes.
 *
 * JUMP is Copyright (C) 2003 Vivid Solutions
 *
 * This program implements extensions to JUMP and is
 * Copyright (C) 2004 Integrated Systems Analysts, Inc.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 * For more information, contact:
 *
 * Integrated Systems Analysts, Inc.
 * 630C Anchors St., Suite 101
 * Fort Walton Beach, Florida
 * USA
 *
 * (850)862-7321
 */
 
package com.isa.jump.plugin;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.CoordinateList;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

public class Arc {

    protected Coordinate center;
    protected Coordinate start;
    protected double radius;
    protected double angle;
    protected double arcTolerance = 0.1;
    final protected int minNumPts = 16;
    
    public Arc(Coordinate center, Coordinate start, double angle)
    {
        this.center = center;
        this.start = start;
        this.angle = angle;
        radius = center.distance(start);
    }
    
    public void setArcTolerance(double arcTolerance)
    {
        this.arcTolerance = arcTolerance;
    }
    
    public Polygon getPoly()
    {
        if (angle == 360.0)
        {
            CoordinateList polyCoords = arcAnglePts(angle, start, center);
            return new GeometryFactory().createPolygon( new GeometryFactory().createLinearRing(polyCoords.toCoordinateArray()),null);
        }
        else
        {
            CoordinateList polyCoords = new CoordinateList();
            polyCoords.add(center);
            CoordinateList coordinates = arcAnglePts(angle, start, center);
            polyCoords.add(coordinates.toCoordinateArray(), true);
            polyCoords.add(center);
            return new GeometryFactory().createPolygon( new GeometryFactory().createLinearRing(polyCoords.toCoordinateArray()),null);
        }
    }
    
    public LineString getLineString()
    {
        CoordinateList coordinates = arcAnglePts(angle, start, center);
        return new GeometryFactory().createLineString(coordinates.toCoordinateArray());
    }
    
    public CoordinateList getCoordinates()
    {
        return arcAnglePts(angle, start, center);
    }
    
    protected CoordinateList arcAnglePts(double angle, Coordinate pt, Coordinate center)
    {
        CoordinateList coordinates = new CoordinateList();
        long n = getPtsFromTolerance(center.distance(pt), angle, arcTolerance);
        long minPts = Math.abs(Math.round(angle/360.0 * minNumPts));
        if (n < minPts) n = minPts;
        double ai = angle / n;
        coordinates.add(new Coordinate(pt));
        
        for (int i = 1; i < n; i++) //add all but the last one
        {
            //Coordinate p2 = GeoUtils.rotPt(pt, center, ai*i);
            coordinates.add(GeoUtils.rotPt(pt, center, ai*i));
        }
        
        if (angle == 360.0)
        {
            coordinates.add(new Coordinate(pt)); //close the circle
        }
        else
        {
            coordinates.add(GeoUtils.rotPt(pt, center, angle)); //add the last point
        }
        return coordinates;
    }
    
    protected int getPtsFromTolerance(double radius, double angle, double tolerance)
    {
        //Tolerance is the distance from the center of a chord to the arc
        //For the given arc, this function will return the number of points
        //that is needed to draw the arc with the given tolerance
        
        final double epsilon = 0.00001;
        int n;
        
        if (radius < epsilon)
        {
            n = 1;
        }
        else
        {
            if ((tolerance / radius) > 0.333)
            {
                n = 1;
            }
            else
            {
                double theta = Math.toDegrees(2 * Math.acos((radius - tolerance) / radius));
                if (theta < epsilon)
                {
                    n = 0;
                }
                else
                {
                    n = (int) Math.floor(Math.abs(angle) / theta) + 1;
                }
            }
        }
        return n;
    }
}