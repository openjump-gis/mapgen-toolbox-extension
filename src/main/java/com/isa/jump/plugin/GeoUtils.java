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

import java.lang.reflect.Method;
import java.util.*;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.CoordinateList;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryCollection;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.MultiPoint;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.util.UniqueCoordinateArrayFilter;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.feature.IndexedFeatureCollection;
import com.vividsolutions.jump.workbench.WorkbenchContext;
import com.vividsolutions.jump.workbench.model.Task;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;


public class GeoUtils {
  public static final int emptyBit = 0;
  public static final int pointBit = 1;
  public static final int lineBit = 2;
  public static final int polyBit = 3;

  public GeoUtils() {
  }

  public static double mag(Coordinate q) {
    return Math.sqrt(q.x * q.x + q.y * q.y);
  }

  public static Coordinate unitVec(Coordinate q) {
    double m = mag(q);
    if (m == 0) m = 1;
    return new Coordinate(q.x / m, q.y / m);
  }

  public static Coordinate vectorAdd(Coordinate q, Coordinate r) {   //return the Coordinate by vector adding r to q
    return new Coordinate(q.x + r.x, q.y + r.y);
  }

  public static Coordinate vectorBetween(Coordinate q, Coordinate r) {   //return the Coordinate by vector subtracting q from r
    return new Coordinate(r.x - q.x, r.y - q.y);
  }

  public static Coordinate vectorTimesScalar(Coordinate q, double m) {   //return the Coordinate by vector subracting r from q
    return new Coordinate(q.x * m, q.y * m);
  }

  public static double dot(Coordinate p, Coordinate q) {
    return p.x * q.x + p.y * q.y;
  }

  public static Coordinate rotPt(Coordinate inpt, Coordinate rpt, double theta) {   //rotate inpt about rpt by theta degrees (+ clockwise)
    double tr = Math.toRadians(theta);
    double ct = Math.cos(tr);
    double st = Math.sin(tr);
    double x = inpt.x - rpt.x;
    double y = inpt.y - rpt.y;
    double xout = rpt.x + x * ct + y * st;
    double yout = rpt.y + y * ct - st * x;
    return new Coordinate(xout, yout);
  }

  public static boolean pointToRight(Coordinate pt, Coordinate p1, Coordinate p2) {   //true if pt is to the right of the line from p1 to p2
    double a = p2.x - p1.x;
    double b = p2.y - p1.y;
    double c = p1.y * a - p1.x * b;
    double fpt = a * pt.y - b * pt.x - c; //Ay - Bx - C = 0
    return (fpt < 0.0);
  }

  public static Coordinate perpendicularVector(Coordinate v1, Coordinate v2, double dist, boolean toLeft) {
    //return perpendicular Coordinate vector from v1 of dist specified to left of v1-v2}
    Coordinate v3 = vectorBetween(v1, v2);
    Coordinate v4 = new Coordinate();
    if (toLeft) {
      v4.x = -v3.y;
      v4.y = v3.x;
    } else {
      v4.x = v3.y;
      v4.y = -v3.x;
    }
    return vectorAdd(v1, vectorTimesScalar(unitVec(v4), dist));
  }


  public static double getBearing180(Coordinate startPt, Coordinate endPt) {   //return Bearing in degrees (-180 to +180) from startPt to endPt
    Coordinate r = new Coordinate(endPt.x - startPt.x, endPt.y - startPt.y);
    double rMag = Math.sqrt(r.x * r.x + r.y * r.y);
    if (rMag == 0.0) {
      return 0.0;
    } else {
      double rCos = r.x / rMag;
      double rAng = Math.acos(rCos);

      if (r.y < 0.0)
        rAng = -rAng;
      return rAng * 360.0 / (2 * Math.PI);
    }
  }

  public static double getBearingRadians(Coordinate startPt, Coordinate endPt) {   //return Bearing in radians (-PI to +PI) from startPt to endPt
    Coordinate r = new Coordinate(endPt.x - startPt.x, endPt.y - startPt.y);
    double rMag = Math.sqrt(r.x * r.x + r.y * r.y);
    if (rMag == 0.0) {
      return 0.0;
    } else {
      double rCos = r.x / rMag;
      double rAng = Math.acos(rCos);
      if (r.y < 0.0)
        rAng = -rAng;
      return rAng;
    }
  }

  public static double getBearing360(Coordinate startPt, Coordinate endPt) {  //return Bearing in degrees (0 - 360) from startPt to endPt
    double bearing = getBearing180(startPt, endPt);
    if (bearing < 0) {
      bearing = 360 + bearing;
    }
    return bearing;
  }

  public static double theta(Coordinate p1, Coordinate p2) {   //this function returns the order of the angle from p1 to p2
    //special use in ConvexHullWrap
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double ax = Math.abs(dx);
    double ay = Math.abs(dy);
    double t = ax + ay;
    if (t != 0.0)
      t = dy / t;
    if (dx < 0.0)
      t = 2.0 - t;
    else {
      if (dy < 0.0)
        t = 4.0 + t;
    }
    return (t * 90.0);
  }

  public static CoordinateList ConvexHullWrap(CoordinateList coords) {
    //The convex hull is the linestring made by the points on the outside of a cloud of points.
    //Thmin = 0, e package wrapping algorithm - see  Algorithms by Sedgewick
    //modified to handle colinear points 28 Jan 2005 by LDB and RFL @ ISA
    //this version removes colinear points on the hull except for the corners
    CoordinateList newcoords = new CoordinateList();
    int n = coords.size();
    int i, m;
    double t, minAngle, dist, distMax, v, vdist;
    Coordinate[] p = new Coordinate[n + 1];
    for (i = 0; i < n; i++) {
      p[i] = coords.getCoordinate(i);
    }
    int min = 0;
    for (i = 1; i < n; i++) {
      if (p[i].y < p[min].y)
        min = i;
    }
    p[n] = coords.getCoordinate(min);
    minAngle = 0.0;
    distMax = 0.0;
    for (m = 0; m < n; m++) {
      //swap(p, m, min);
      Coordinate temp = p[m];
      p[m] = p[min];
      p[min] = temp;
      min = n;
      v = minAngle;
      vdist = distMax;
      minAngle = 360.0;
      for (i = m + 1; i <= n; i++) {
        t = theta(p[m], p[i]);
        dist = p[m].distance(p[i]);
        if ((t > v) || ((t == v) && (dist > vdist))) {
          if ((t < minAngle) || ((t == minAngle) && (dist > distMax))) {
            min = i;
            minAngle = t;
            distMax = dist;
          }
        }
      }
      if (min == n) { //sentinal found
        for (int j = 0; j <= m; j++)
          newcoords.add(p[j], true);
        if (!(p[0].equals2D(p[m]))) {
          newcoords.add(p[0], true);
        }

        LinearRing lr = new GeometryFactory().createLinearRing(newcoords.toCoordinateArray());
        if (!clockwise(lr)) {
          CoordinateList newcoordsCW = new CoordinateList();
          for (int j = newcoords.size() - 1; j >= 0; j--)
            newcoordsCW.add(newcoords.getCoordinate(j));
          return newcoordsCW;
        } else {
          return newcoords;
        }
      }
    }
    return newcoords; //should never get here
  }


  /**
   * @return - the distance from r to the line segment p0-p1
   */
  public static double getDistance(Coordinate r, Coordinate p0, Coordinate p1) {
    //return r.distance(getClosestPointOnSegment(pt, p0, p1));
    return Math.sqrt(getDistanceSqd(r, p0, p1, new Coordinate(0, 0)));//,  new int[1]));
  }

  /**
   * @return - the coordinate on the line segment p0-p1 which is closest to r
   */
  public static Coordinate getClosestPointOnSegment(Coordinate r, Coordinate p0, Coordinate p1) {
    Coordinate coordOut = new Coordinate(0, 0);
    getDistanceSqd(r, p0, p1, coordOut);
    return coordOut;
  }


  /**
   * Find the perpendicular distance between Point R and the Line from P0 to P1.
   * Based on the parametric equation of a line: P(t) = P0 + tV.
   * First find the value of t such that R - P(t) is perpendicular to V where V
   * is the vector P1 - P0.  Given that the dot product of two vectors is zero
   * when they are perpendicular:	  ( * is read as dot )
   * (R-P(t)) * V = 0			  substituting P0 + tV for P(t) gives
   * (R - P0 - tV) * V = 0		  collecting term gives:
   * (R-P0) * V = tV * V			  solving for t gives:
   * t = ((R-P0) * V) / (V * V)	  If t is in the interval 0 to 1 then
   * the intersection point is between P0 and P1, otherwise use the distance
   * formula.	Plugging in the value of t to the original equation gives the
   * vector from the line to R and we need only take the magnitude of it to
   * find the distance.
   *
   * @param r        - an arbitrary Coordinate
   * @param p0       - start point of line segment
   * @param p1       - end point of line segment
   * @param coordOut - pass pre-allocated. Returns point on p0-p1 closest to r
   *                 (constrained to segment).
   * @return the distance squared from r to segment p0-p1.
   */
  public static double getDistanceSqd(final Coordinate r,
                                      final Coordinate p0, final Coordinate p1,
                                      final Coordinate coordOut) //, int[] which)
  // @param which pass with which = new int[1] - returns with which[0] == 0 (P0), 1 (P0-P1), 2 (P1)
  {
    double Xv = p1.x - p0.x;
    double Yv = p1.y - p0.y;
    double VdotV = Xv * Xv + Yv * Yv;
    double Xp0r = r.x - p0.x;
    double Yp0r = r.y - p0.y;
    //which[0] = 0;
    if (VdotV == 0.0) { //degenerate line (p0, p1 the same)
      coordOut.x = p0.x;
      coordOut.y = p0.y;
      //which[0] = 1;
      return Xp0r * Xp0r + Yp0r * Yp0r;
    }
    double t = (Xp0r * Xv + Yp0r * Yv) / VdotV; //Dot(VectorBetween(P0, R), V) / VdotV
    if (t <= 0.0) { //return r.distance(p0);
      coordOut.x = p0.x;
      coordOut.y = p0.y;
      //which[0] = 0;
      return Xp0r * Xp0r + Yp0r * Yp0r;
    } else if (t >= 1.0) { //return r.distance(p1);
      coordOut.x = p1.x;
      coordOut.y = p1.y;
      double Xp1r = r.x - p1.x;
      double Yp1r = r.y - p1.y;
      //which[0] = 2;
      return Xp1r * Xp1r + Yp1r * Yp1r;
    } else { //if ((t > 0.0) && (t < 1.0)) { //P(t) is between P0 and P1
      double Xp = (p0.x + t * Xv) - r.x; //VectorBetween(R, VectorAdd(P0, VectorTimesScalar(V, t)))}
      double Yp = (p0.y + t * Yv) - r.y;
      coordOut.x = r.x + Xp;
      coordOut.y = r.y + Yp;
      //which[0] = 1;
      return Xp * Xp + Yp * Yp;
    }
  }

  public static double distanceSqd(Coordinate r, Coordinate p) {
    double dx = r.x - p.x;
    double dy = r.y - p.y;
    return dx * dx + dy * dy;
  }

  public static Coordinate getClosestPointOnLine(Coordinate pt, Coordinate p0, Coordinate p1) {   //returns the nearest point from pt to the infinite line defined by p0-p1
    MathVector vpt = new MathVector(pt);
    MathVector vp0 = new MathVector(p0);
    MathVector vp1 = new MathVector(p1);
    MathVector v = vp0.vectorBetween(vp1);
    double vdotv = v.dot(v);

    if (vdotv == 0.0) //degenerate line (ie: P0 = P1)
    {
      return p0;
    } else {
      double t = vp0.vectorBetween(vpt).dot(v) / vdotv;
      MathVector vt = v.scale(t);
      vpt = vp0.add(vt);
      return vpt.getCoord();
    }
  }

  public static Coordinate along(double d, Coordinate q, Coordinate r) {   //return the point at distance d along vector from q to r
    double ux, uy, m;
    Coordinate n = (Coordinate) r.clone();
    ux = r.x - q.x;
    uy = r.y - q.y;
    m = Math.sqrt(ux * ux + uy * uy);
    if (m != 0) {
      ux = d * ux / m;
      uy = d * uy / m;
      n.x = q.x + ux;
      n.y = q.y + uy;
    }
    return n;
  }

  public static double interiorAngle(Coordinate p1, Coordinate p2, Coordinate p3) {
    //return the angle between vectors p2-p1 and p2-p3  from 0 to 180
    //NOTE: this routine returns POSITIVE angles only
    Coordinate p = vectorBetween(p1, p2);  //relativize the position vectors
    Coordinate q = vectorBetween(p3, p2);
    double arg = dot(p, q) / (mag(p) * mag(q));
    if (arg < -1.0) arg = -1.0;
    else if (arg > 1.0) arg = 1.0;
    return Math.toDegrees(Math.acos(arg));
  }

  /**
   * @param ring - LinearRing represented as LineString to analyze
   * @return - Coordinate[] with first point of passed LineString in [0]
   * followed by [1 to length] with x as distance and y as angle.
   * The angle will be the absolute bearing in the range 0-360.
   * The original LineString and Coordinate points are unmodified.
   */
  public static Coordinate[] getDistanceBearingArray(LineString ring) {
    Coordinate[] coords = new Coordinate[ring.getNumPoints()];
    Coordinate p1 = new Coordinate(ring.getCoordinateN(0));
    coords[0] = p1;
    for (int i = 1; i < coords.length; i++) {
      coords[i] = new Coordinate(ring.getCoordinateN(i));
      Coordinate p2 = coords[i];
      double angle = getBearing360(p1, p2);
      double distance = p1.distance(p2);
      p1.x = p2.x;
      p1.y = p2.y;
      coords[i].x = distance;
      coords[i].y = angle;
    }
    return coords;
  }

  /**
   * @param ring - LinearRing represented as LineString to analyze
   * @return - Coordinate[] array of with x as distance and y as
   * interior angles in degrees 0 to +180.
   * The angles at each index in the array are the interior angles at the
   * vertex position in the (closed polygon) ring.  Every array position is filled.
   * The distances are the distance at a vertex to the following point.  For [n-2]
   * the distance is computed to the [n-1] position assuming the ring is closed.
   */
  public static Coordinate[] getDistanceAngleArray(final LineString ring) {
    int n = ring.getNumPoints();
    Coordinate[] coords = new Coordinate[n];
    for (int i = 0; i < coords.length; i++) {
      Coordinate pb = ring.getCoordinateN(  //previous Index
          (i == 0) ? n - 2 : i - 1);
      Coordinate p = ring.getCoordinateN(i);
      Coordinate pn = ring.getCoordinateN(  //next Index(
          (i == n - 1) ? 1 : i + 1);
      double angle = interiorAngle(pb, p, pn);
      double distance = p.distance(pn);
      coords[i] = new Coordinate(distance, angle, Double.NaN);
    }
    return coords;
  }

  /**
   * @param ring - a LineString representing a linear ring
   * @return - an array of Coordinate points with colinear points removed.
   * The original LineString and Coordinate points are unmodified.
   */
  public static LinearRing removeRedundantPoints(final LineString ring) {
    //final double epsilon = 1E-6; //probably too coarse for lat/long maps
    Coordinate[] coords = new Coordinate[ring.getNumPoints()];
    int n = coords.length;
    boolean[] remove = new boolean[n];
    for (int i = 0; i < n; i++) {
      coords[i] = new Coordinate(ring.getCoordinateN(i));
      remove[i] = false;
    }
    Coordinate p2 = null;
    Coordinate p3 = null;
    for (int i = 0; i < coords.length; i++) {
      Coordinate p1 = coords[i];
      if (i > 1) {
        //double dist = getDistance( p2, p1, p3); //distance from p2 to segment p1-p3
        //boolean colinear = (dist <= epsilon);
        double angle = interiorAngle(p1, p2, p3); //angle between vectors 0 - 180
        boolean colinear = angle > 179;
        if (colinear) {
          remove[i - 1] = colinear;
          n--;
        }
      }
      p3 = p2;
      p2 = p1;
    }
    Coordinate[] newCoords = new Coordinate[n];
    int j = 0;
    for (int i = 0; i < coords.length; i++) {
      if (!remove[i])
        newCoords[j++] = new Coordinate(coords[i]);
    }
    return ring.getFactory().createLinearRing(newCoords);
  }

  public static Geometry reducePoints(final Geometry geo, double tolerance) { //uses Douglas-Peucker algorithm
    CoordinateList coords = new CoordinateList();
    UniqueCoordinateArrayFilter filter = new UniqueCoordinateArrayFilter();
    geo.apply(filter);
    coords.add(filter.getCoordinates(), false);

    //need to do this since UniqueCoordinateArrayFilter keeps the poly from being closed
    if ((geo instanceof Polygon) || (geo instanceof LinearRing)) {
      coords.add(coords.getCoordinate(0));
    }

    int maxIndex = coords.size() - 1;
    int temp;
    do {
      temp = maxIndex;
      int i = 0;
      do //generate every possible corridor
      {
        Coordinate anchor = coords.getCoordinate(i);
        boolean pointDeleted = false;
        int k = maxIndex;
        do {
          Coordinate floater = coords.getCoordinate(k);
          double dmax = -1.0;
          int j = k;

          while (j > (i + 1)) {
            j--;
            Coordinate pt = coords.getCoordinate(j);
            Coordinate cp = getClosestPointOnLine(pt, anchor, floater);
            double d = pt.distance(cp);

            if (d > dmax) {
              dmax = d;
              k = j;
            }
          }

          if ((dmax < tolerance) && (dmax > -1.0) && (maxIndex > 1)) {
            pointDeleted = true;
            coords.remove(k);
            maxIndex--;
            k = maxIndex;
          }

        } while (!(pointDeleted || (k <= (i + 1)))); //until PointDeleted or (k<=(i+1))
        i++;
      } while (i <= (maxIndex - 2));
    } while (temp != maxIndex);
    if (geo instanceof LinearRing) {
      return new GeometryFactory().createLinearRing(coords.toCoordinateArray());
    } else if (geo instanceof LineString) {
      return new GeometryFactory().createLineString(coords.toCoordinateArray());
    } else if (geo instanceof Polygon) {
      return new GeometryFactory().createPolygon(
          new GeometryFactory().createLinearRing(coords.toCoordinateArray()),
          null);
    } else {
      return geo;
    }
  }

  public static boolean clockwise(final Geometry geo) {
    if ((geo instanceof Polygon) || (geo instanceof LinearRing)) {   //calculates the area; neg means clockwise
      //from CRC 25th Edition Page 284
      double t1, t2;
      double geoArea;
      Coordinate[] geoCoords = geo.getCoordinates();
      int maxIndex = geoCoords.length - 1;
      t1 = geoCoords[maxIndex].x * geoCoords[0].y;
      t2 = -geoCoords[0].x * geoCoords[maxIndex].y;

      for (int i = 0; i < maxIndex; i++) {
        t1 += (geoCoords[i].x * geoCoords[i + 1].y);
        t2 -= (geoCoords[i + 1].x * geoCoords[i].y);
      }

      geoArea = 0.5 * (t1 + t2);
      return (geoArea < 0);
    } else {
      return true;
    }
  }

  /**
   * Compute a partial area of a polygon represented by a
   *
   * @param start  - start index of partial polygon.
   * @param num    - number of vertices of partial polygon.
   * @param coords Coordinate[] representing polygon.
   * @return signed area
   */
  public static double area(int start, int num, final Coordinate[] coords) {
    //from CRC 25th Edition Page 284
    int n = coords.length - 1;
    int maxIndex = moduloAccess(start, num, n);
    double t1 = coords[maxIndex].x * coords[0].y;
    double t2 = -coords[0].x * coords[maxIndex].y;
    for (int i = 0; i < num; i++) {
      int m = moduloAccess(start, i, n);
      int m1 = (m == n) ? 0 : m + 1;
      t1 += (coords[m].x * coords[m1].y);
      t2 -= (coords[m1].x * coords[m].y);
    }
    return 0.5 * (t1 + t2);
  }

  /**
   * Compute Modulo index to a circular array starting at i adding offset to wrap around at n.
   *
   * @param i      - can be any value, even negative
   * @param offset - can be any value, even negative
   * @param n      - the last allowable index
   * @return int position of (i + offset) modulo n
   */
  public static int moduloAccess(int i, int offset, int n) {
    int modulo = (i + offset) % (n + 1);
    if (modulo < 0)
      modulo = -modulo;
    return modulo;
  }

  public static Coordinate intersect(Coordinate P1, Coordinate P2, Coordinate P3, Coordinate P4) //find intersection of two lines
  {
    Coordinate V = new Coordinate((P2.x - P1.x), (P2.y - P1.y));
    Coordinate W = new Coordinate((P4.x - P3.x), (P4.y - P3.y));
    double n = W.y * (P3.x - P1.x) - W.x * (P3.y - P1.y);
    double d = W.y * V.x - W.x * V.y;

    if (d != 0.0) {
      double t1 = n / d;
      Coordinate E = new Coordinate((P1.x + V.x * t1), (P1.y + V.y * t1));
      return E;
    } else //lines are parallel; no intersection
    {
      return null;
    }
  }

  /**
   * Warning: this method adds an Epsilon tolerance of .001 to the end of both segments.
   * This eliminates any doubt that abutting segments might not intersect (i.e. T intersections)
   *
   * @param p0, p1 segment 1
   * @param p2, p3 segment 2
   * @return Coordinate  intersection point that lies on both line segments or null
   */
  public static Coordinate intersectSegments(Coordinate p0, Coordinate p1, Coordinate p2, Coordinate p3) {
    //find intersection of two line segment that meet the criteria expressed by onp0p1 & onp2p3
    double Vx = (p1.x - p0.x);
    double Vy = (p1.y - p0.y);
    double Wx = (p3.x - p2.x);
    double Wy = (p3.y - p2.y);
    double d = Wy * Vx - Wx * Vy;

    if (d != 0.0) {
      double n1 = Wy * (p2.x - p0.x) - Wx * (p2.y - p0.y);
      double n2 = Vy * (p2.x - p0.x) - Vx * (p2.y - p0.y);
      double t1 = n1 / d;
      double t2 = n2 / d;
      double epsilon = 0.001;
      double lowbound = 0.0 - epsilon;
      double hibound = 1.0 + epsilon;
      boolean onp0p1 = (t1 >= lowbound) && (t1 <= hibound);
      boolean onp2p3 = (t2 >= lowbound) && (t2 <= hibound);
      if (onp0p1 && onp2p3) {
        return new Coordinate((p0.x + Vx * t1), (p0.y + Vy * t1)); //intersection point
      } else
        return null; //the intersection point does not lie on both segments
    } else //lines are parallel; no intersection
    {
      return null;
    }
  }

  public static Coordinate getCenter(Coordinate p1, Coordinate p2, Coordinate p3) {
    double x = p1.x + ((p2.x - p1.x) / 2.0);
    double y = p1.y + ((p2.y - p1.y) / 2.0);
    Coordinate p12 = new Coordinate(x, y);

    if (pointToRight(p3, p1, p2))
      p1 = rotPt(p1, p12, -90.0);
    else
      p1 = rotPt(p1, p12, 90.0);

    x = p2.x + ((p3.x - p2.x) / 2.0);
    y = p2.y + ((p3.y - p2.y) / 2.0);
    Coordinate p23 = new Coordinate(x, y);

    if (pointToRight(p1, p3, p2))
      p3 = rotPt(p3, p23, -90.0);
    else
      p3 = rotPt(p3, p23, 90.0);

    Coordinate center = intersect(p1, p12, p3, p23);

    if (center == null) //no intersection; lines parallel
      return p2;
    else
      return center;
  }

  public static BitSet setBit(BitSet bitSet, Geometry geometry) {
    BitSet newBitSet = (BitSet) bitSet.clone();
    if (geometry.isEmpty()) newBitSet.set(emptyBit);
    else if (geometry instanceof Point) newBitSet.set(pointBit);
    else if (geometry instanceof MultiPoint) newBitSet.set(pointBit);
    else if (geometry instanceof LinearRing) newBitSet.set(lineBit);
    else if (geometry instanceof LineString) newBitSet.set(lineBit);
    else if (geometry instanceof MultiLineString) newBitSet.set(lineBit);
    else if (geometry instanceof Polygon) newBitSet.set(polyBit);
    else if (geometry instanceof MultiPolygon) newBitSet.set(polyBit);
    else if (geometry instanceof GeometryCollection) {
      GeometryCollection geometryCollection = (GeometryCollection) geometry;
      for (int i = 0; i < geometryCollection.getNumGeometries(); i++)
        newBitSet = setBit(newBitSet, geometryCollection.getGeometryN(i));
    }
    return newBitSet;
  }

  public static LineString MakeRoundCorner(Coordinate A, Coordinate B, Coordinate C, Coordinate D, double r, boolean arcOnly) {
    MathVector Gv = new MathVector();
    MathVector Hv;
    MathVector Fv;
    Coordinate E = intersect(A, B, C, D);  //vector solution

    if (E != null) //non-parallel lines
    {
      MathVector Ev = new MathVector(E);

      if (E.distance(B) > E.distance(A)) //find longest distance from intersection
      {   //these equations assume B and D are closest to the intersection
        //reverse points
        Coordinate temp = A;
        A = B;
        B = temp;
      }

      if (E.distance(D) > E.distance(C)) //find longest distance from intersection
      {   //these equations assume B and D are closest to the intersection
        //reverse points
        Coordinate temp = C;
        C = D;
        D = temp;
      }

      MathVector Av = new MathVector(A);
      MathVector Cv = new MathVector(C);
      double alpha = Ev.vectorBetween(Av).angleRad(Ev.vectorBetween(Cv)) / 2.0; //we only need the half angle
      double h1 = Math.abs(r / Math.sin(alpha));  //from definition of sine solved for h

      if ((h1 * h1 - r * r) >= 0) {
        double d1 = Math.sqrt(h1 * h1 - r * r);  //pythagorean theorem}
        double theta = Math.PI / 2.0 - alpha; //sum of triangle interior angles = 180 degrees
        theta = theta * 2.0;          //we only need the double angle}
        //we now have the angles and distances needed for a vector solution:
        //we must find the points G and H by vector addition.
        //Gv = Ev.add(Av.vectorBetween(Ev).unit().scale(d1));
        //Hv = Ev.add(Cv.vectorBetween(Ev).unit().scale(d1));
        //Fv = Ev.add(Gv.vectorBetween(Ev).rotateRad(alpha).unit().scale(h1));
        Gv = Ev.add(Ev.vectorBetween(Av).unit().scale(d1));
        Hv = Ev.add(Ev.vectorBetween(Cv).unit().scale(d1));
        Fv = Ev.add(Ev.vectorBetween(Gv).rotateRad(alpha).unit().scale(h1));

        if (Math.abs(Fv.distance(Hv) - Fv.distance(Gv)) > 1.0) //rotated the wrong dirction
        {
          Fv = Ev.add(Ev.vectorBetween(Gv).rotateRad(-alpha).unit().scale(h1));
          theta = -theta;
        }

        CoordinateList coordinates = new CoordinateList();
        if (!arcOnly) coordinates.add(C);
        Arc arc = new Arc(Fv.getCoord(), Hv.getCoord(), Math.toDegrees(theta));
        LineString lineString = arc.getLineString();
        coordinates.add(lineString.getCoordinates(), false);
        if (!arcOnly) coordinates.add(A);
        return new GeometryFactory().createLineString(coordinates.toCoordinateArray());
      }
    }
    return null;
  }

  public static boolean geometriesEqual(Geometry geo1, Geometry geo2) {
    if ((!(geo1 instanceof GeometryCollection)) &&
        (!(geo2 instanceof GeometryCollection)))
      return geo1.equals(geo2);

    if ((!(geo1 instanceof GeometryCollection)) &&
        ((geo2 instanceof GeometryCollection)))
      return false;

    if (((geo1 instanceof GeometryCollection)) &&
        (!(geo2 instanceof GeometryCollection)))
      return false;

    //at this point both are instanceof GeometryCollection
    int numGeos1 = (geo1).getNumGeometries();
    int numGeos2 = (geo2).getNumGeometries();
    if (numGeos1 != numGeos2) return false;

    for (int index = 0; index < numGeos1; index++) {
      Geometry internalGeo1 = (geo1).getGeometryN(index);
      Geometry internalGeo2 = (geo2).getGeometryN(index);
      if (!geometriesEqual(internalGeo1, internalGeo2))
        return false;
    }

    return true;
  }

  ;

  public static double getDistanceFromPointToGeometry(Coordinate coord, Geometry geo) {
    //will return distance to nearest edge of closed polys or GeometryCollections including holes
    //unlike jts which returns zero for any point inside a poly
    double closestDist = Double.MAX_VALUE;

    for (int i = 0; i < geo.getNumGeometries(); i++) {
      double newDist;
      Geometry internalGeo = geo.getGeometryN(i);

      if (internalGeo instanceof Point) {
        newDist = coord.distance(internalGeo.getCoordinate());
        if (newDist < closestDist) closestDist = newDist;
      } else if (internalGeo instanceof LineString) {
        Coordinate[] coords = internalGeo.getCoordinates();
        for (int j = 0; j < coords.length - 1; j++) {
          newDist = GeoUtils.getDistance(coord, coords[j], coords[j + 1]);
          if (newDist < closestDist) closestDist = newDist;
        }
      } else if (internalGeo instanceof Polygon) {
        Geometry newGeo = internalGeo.getBoundary();
        newDist = getDistanceFromPointToGeometry(coord, newGeo);
        if (newDist < closestDist) closestDist = newDist;
      } else if (internalGeo instanceof MultiPoint) {
        Coordinate[] coords = internalGeo.getCoordinates();
        for (int k = 0; k < coords.length; k++) {
          newDist = coord.distance(coords[k]);
          if (newDist < closestDist) closestDist = newDist;
        }
      } else //remaining geometry types are multi or collections
      {
        for (int m = 0; m < internalGeo.getNumGeometries(); m++) {
          newDist = getDistanceFromPointToGeometry(coord, internalGeo.getGeometryN(m));
          if (newDist < closestDist) closestDist = newDist;
        }
      }
    }

    return closestDist;
  }

  public static boolean geometryIsSegmentOf(Geometry geo1, Geometry geo2) {
    //true if geo1 matches with a segment of geo2

    if (geo1.getNumPoints() > geo2.getNumPoints())
      return false;

    int numGeos1 = geo1.getNumGeometries();
    int numGeos2 = geo2.getNumGeometries();

    if ((numGeos1 == 1) && (numGeos2 == 1)) {
      Coordinate[] coords1 = geo1.getCoordinates();
      Coordinate[] coords2 = geo2.getCoordinates();
      int i1 = 0;
      int i2 = 0;

      while (i2 < coords2.length) {
        if (coords1[0].equals2D(coords2[i2])) break;
        i2++;
      }

      if (i2 == coords2.length) return false;

      while ((i1 < coords1.length) && (i2 < coords2.length)) {
        if (!coords1[i1].equals2D(coords2[i2])) return false;
        i1++;
        i2++;
      }

      return (i1 == coords1.length);
    } else {
      boolean foundMatch = false;

      for (int i = 0; i < numGeos1; i++) {
        foundMatch = false;

        for (int j = 0; j < numGeos2; j++) {
          if (geometryIsSegmentOf(geo1.getGeometryN(i), geo2.getGeometryN(j))) {
            foundMatch = true;
            break;
          }
        }

        if (!foundMatch) return false;
      }

      return foundMatch;
    }
  }

  ;

  /**
   * @param startPt,         endPt - coordinates of line to construct buffer arc from
   * @param bufferStartAngle - angle in degrees from perpendicular CCW around startPt
   * @param bufferEndAngle   - angle in degrees from perpendicular CW around endPt
   * @param bufferDistance   - radius of arc buffer
   * @param arcTolerance
   * @return Polygon representing the arc buffer
   */
  public static Geometry bufferArc(Coordinate startPt, Coordinate endPt,
                                   double bufferStartAngle, double bufferEndAngle,
                                   double bufferDistance, double arcTolerance) {
    Coordinate perp1 = perpendicularVector(startPt, endPt, bufferDistance, true);
    Coordinate startBuf = rotPt(perp1, startPt, -bufferStartAngle);
    Coordinate perp2 = perpendicularVector(endPt, startPt, bufferDistance, false);
    Arc arc1 = new Arc(startPt, startBuf, bufferStartAngle);
    Arc arc2 = new Arc(endPt, perp2, bufferEndAngle);
    arc1.setArcTolerance(arcTolerance);
    arc2.setArcTolerance(arcTolerance);
    CoordinateList polyCoords = new CoordinateList();
    polyCoords.add(startPt, true);
    polyCoords.add(arc1.getCoordinates().toCoordinateArray(), true);
    polyCoords.add(arc2.getCoordinates().toCoordinateArray(), true);
    polyCoords.add(endPt, true);
    polyCoords.add(startPt, true);
    return new GeometryFactory().createPolygon(new GeometryFactory()
        .createLinearRing(polyCoords.toCoordinateArray()), null);
  }

  /**
   * Construct a perfectly orthogonal rectangle using the input's first two points,
   * and the distance between the second and third points as the length.
   *
   * @param rectangle - rectangular polygon.
   * @param sideOne   - the index of the side (numbered from 1 to 4) that is the front.
   * @return - a rectangle represented by a Coordinate[5];
   */
  public static Coordinate[] rectangleFromGeometry(Geometry rectangle, int sideOne) {
    if ((rectangle.getNumGeometries() > 1) || (rectangle instanceof MultiPolygon))
      rectangle = rectangle.getGeometryN(0);
    Coordinate[] p;
    if (rectangle instanceof Polygon) {
      p = ((Polygon) rectangle).getExteriorRing().getCoordinates();
    } else
      p = rectangle.getCoordinates();
    if (!(p.length == 5))
      return null;
    if (sideOne != 1) {
      sideOne = Math.max(1, Math.min(4, sideOne)) - 1; //force 0 to 3
      for (int j = 0; j < sideOne; j++) {
        //rotate vertex down by one
        int n = p.length - 2;
        Coordinate t = p[0];
        for (int i = 0; i < n; i++) {
          p[i] = p[i + 1];
        }
        p[n] = t;
      }
      p[p.length - 1] = p[0];
    }
    Coordinate p2 = perpendicularVector(p[1], p[0], p[1].distance(p[2]), true);
    Coordinate p3 = perpendicularVector(p2, p[1], p[0].distance(p[1]), true);
    return new Coordinate[]{p[0], p[1], p2, p3, p[0]};
  }

  /**
   * Create buffer arc polygons from the four sides of a rectangular polygon.  Note that
   * the method does not depend on the polygon being perfectly rectangular, but will
   * produce buffer arcs around an orthogalized version of the polygon using side one
   * and the length of side two.
   *
   * @param rectangle  - rectangular polygon.
   * @param frontAngle - angle in degrees of arc segments from front (side 1)
   * @param rearAngle  - angle in degrees of arc segments from rear (side 3)
   * @param distances  - array of 4 arc radii for front, right, rear, and left distances
   * @param sideOne    - the index of the side (numbered from 1 to 4) that is the front.
   * @return - a MultiPolygon with the non-zero buffer arc Polygons.  If only one distance is
   * non-zero, a Polygon will be returned.
   */
  public static Geometry rectangleBufferArcs(Geometry rectangle,
                                             double frontAngle, double rearAngle,
                                             double[] distances, int sideOne, double arcTolerance) {
    Coordinate[] p = rectangleFromGeometry(rectangle, sideOne);
    if (p == null)
      return null;
    int count = 0;
    for (int i = 0; i < distances.length; i++) {
      if (distances[i] != 0.0) count++;
    }
    Polygon[] arcs = new Polygon[count];
    count = 0;
    if (distances[0] != 0.0)
      arcs[count++] = (Polygon) bufferArc(p[0], p[1], frontAngle, frontAngle,
          distances[0], arcTolerance);
    if (distances[1] != 0.0)
      arcs[count++] = (Polygon) bufferArc(p[1], p[2], 90 - frontAngle, 90 - rearAngle,
          distances[1], arcTolerance);
    if (distances[2] != 0.0)
      arcs[count++] = (Polygon) bufferArc(p[2], p[3], rearAngle, rearAngle,
          distances[2], arcTolerance);
    if (distances[3] != 0.0)
      arcs[count++] = (Polygon) bufferArc(p[3], p[0], 90 - rearAngle, 90 - frontAngle,
          distances[3], arcTolerance);
    if (count == 1)
      return arcs[0];
    else
      return new GeometryFactory().createMultiPolygon(arcs);
  }

  /**
   * SkyJUMP has modified core Task class to add getUnitsName() method.
   * The following code allows this plugin to be used with other core branches
   * without having to implement SkyJUMP core changes.
   *
   * @return "Meters", "Feet", or "Undefined"
   */
  public static String getTaskUnits(WorkbenchContext workbenchContext) {
    try {
      Task task = workbenchContext.getTask();
      Class<? extends Task> c = task.getClass();
      Method m = c.getMethod("getUnitsName", (Class[]) null);
      return (String) m.invoke(task, (Object) null);
    } catch (Exception ex) {
      // NoSuchMethodException, IllegalAccessException, InvocationTargetException
    }
    return "Undefined";
  }

  public static void setTaskUnits(PlugInContext context, String units) {
    try {
      //SkyJUMP has modified core Task class to add getUnitsName() method.
      //The following code allows this plugin to be used with other core branches
      //without having to implement SkyJUMP core changes.
      Task task = context.getWorkbenchContext().getTask();
      Class<?> c = context.getWorkbenchContext().getTask().getClass();
      Method m = c.getMethod("setUnitsName", new Class[]{String.class});
      Object[] parameters = new Object[]{units};
      m.invoke(task, parameters);
    } catch (Exception ex) {
    }
  }

  public final static String ASHS_ID = "ASHS_ID";

  public static Geometry clipToPolygon(Polygon poly, Geometry b) {
    Geometry intersection = null;
    try {
      intersection = poly.intersection(b);
    } catch (Exception ex) {
      System.out.println(poly);
      System.out.println(b);
    }
    return intersection;
  }

  /**
   * @param poly       - the Polygon to be used for clipping.
   * @param ifc        - a FeatureCollection that has been indexed.
   *                   You can just pass: new IndexedFeatureCollection(fc) if you like.
   * @param copySchema - if true the output will have the same Schema as the input.
   *                   If false it will have GEOMETRY and ASHS_ID only.
   * @return a FeatureCollection with the input clipped to poly.
   */
  public static FeatureCollection clipToPolygon(Polygon poly, IndexedFeatureCollection ifc,
                                                boolean copySchema) {
    //IndexedFeatureCollection ifc = new IndexedFeatureCollection(b);
    FeatureSchema featureSchema = new FeatureSchema();
    if (copySchema) {
      FeatureSchema ifcSchema = ifc.getFeatureSchema();
      for (int i = 0; i < ifcSchema.getAttributeCount(); i++) {
        featureSchema.addAttribute(ifcSchema.getAttributeName(i),
            ifcSchema.getAttributeType(i));
      }
    } else {
      featureSchema.addAttribute("GEOMETRY", AttributeType.GEOMETRY);
      featureSchema.addAttribute(ASHS_ID, AttributeType.STRING);
    }
    FeatureDataset overlay = new FeatureDataset(featureSchema);
    Envelope polyEnvelope = poly.getEnvelopeInternal();
    Geometry polyGeomEnvelope = poly.getEnvelope();
    for (Feature b : ifc.query(polyEnvelope)) {
      //addIntersection(a, b, mapping, overlay);
      if (!polyGeomEnvelope.intersects(b.getGeometry().getEnvelope())) continue;
      Geometry intersection = clipToPolygon(poly, b.getGeometry());
      if ((intersection == null) || intersection.isEmpty()) continue;
      addFeature(intersection, overlay, b, copySchema);
    }
    return overlay;
  }

  public static void addFeature(Geometry intersection, FeatureDataset overlay,
                                Feature b, boolean copySchema) {
    if (intersection instanceof GeometryCollection) {
      GeometryCollection gc = (GeometryCollection) intersection;
      for (int i = 0; i < gc.getNumGeometries(); i++) {
        addFeature(gc.getGeometryN(i), overlay, b, copySchema);
      }
      return;
    }
    Feature feature = new BasicFeature(overlay.getFeatureSchema());
    if (copySchema) {
      for (int i = 0; i < b.getSchema().getAttributeCount(); i++) {
        feature.setAttribute(i, b.getAttribute(i));
      }
    } else {
      feature.setAttribute(ASHS_ID, b.getAttribute(ASHS_ID));
    }
    feature.setGeometry(intersection);
    overlay.add(feature);
  }

  /**
   * @param geom Geometry to check
   * @return true if Geometry has multiple components.
   * This includes MultiPolygon, MultiLineString, etc. and simple Polygons with holes.
   * Use with getNumComponents() and getComponentN().
   */
  public static boolean isMultiGeometry(Geometry geom) {
    return (geom instanceof Polygon)
        ? ((Polygon) geom).getNumInteriorRing() > 0
        : geom.getNumGeometries() > 1;
  }

  /**
   * The purpose of this method is (along with getCompentN()) is to treat
   * Polygons with holes the same as a Multi-Geometry to make processsing them easier.
   *
   * @param geom - Geometry used to determine the number of components.
   * @return - number of geometry components.
   */
  public static int getNumComponents(Geometry geom) {
    return (geom instanceof Polygon)
        ? ((Polygon) geom).getNumInteriorRing() + 1
        : geom.getNumGeometries();
  }

  /**
   * The purpose of this method is (along with getNumComponents()) is to treat
   * Polygons with holes the same as a Multi-Geometry to make processsing them easier.
   *
   * @param geom - Geometry from which to extract the component.
   * @param n    - index from 0 to number of components.
   * @return - the Nth component of passed Geometry.
   */
  public static Geometry getComponentN(Geometry geom, int n) {
    if (geom instanceof Polygon) {
      if (n == 0) {
        return ((Polygon) geom).getExteriorRing();
      } else {
        return ((Polygon) geom).getInteriorRingN(n - 1);
      }
    } else {
      return geom.getGeometryN(n);
    }
  }

  /**
   * @param geom1,geom2   - Geometry (supports all Geometry types including GeometryCollection,
   *                      MultiPolygon, MultiPoint, and MultiLineString).
   * @param limitDistance - Distance at which search terminates. If you have no limit then call
   *                      with limitDistance = Double.MAX_VALUE.  Method will check Envelope.distance(Envelope) to
   *                      determine if it is greater than limitDistance, and if so will return immediately.
   * @return - shortest distance between the two Geometry objects.  Will return distance
   * between nearest edge of closed polys or GeometryCollections unlike JTS which returns zero
   * for any point inside a poly.  Also, it is more efficient than JTS.
   */
  public static double distance(Geometry geom1, Geometry geom2, double limitDistance,
                                final Coordinate coord1, final Coordinate coord2) {
    if (geom1.getEnvelopeInternal().distance(geom2.getEnvelopeInternal()) >= limitDistance) {
      return limitDistance; //if limitDistance less than distance between envelopes we're done
    }
    if (isMultiGeometry(geom1)) {
      Coordinate newcoord1 = new Coordinate();
      Coordinate newcoord2 = new Coordinate();
      for (int i = 0; i < getNumComponents(geom1); i++) {
        double newDist;
        Geometry internalGeo = getComponentN(geom1, i);
        newDist = distance(internalGeo, geom2, limitDistance, newcoord1, newcoord2);
        if (newDist < limitDistance) {
          coord1.setCoordinate(newcoord1);
          coord2.setCoordinate(newcoord2);
          limitDistance = newDist;
        }
      }
      return limitDistance;
    } else if (!isMultiGeometry(geom2)) { //this simple case occurs 99% of the time
      return getShortestDistance(geom1, geom2, coord1, coord2);
    }
    //geom1 is not MultiGeometry
    for (int i = 0; i < getNumComponents(geom2); i++) {
      Geometry internalGeo = getComponentN(geom2, i);
      Coordinate newcoord1 = new Coordinate();
      Coordinate newcoord2 = new Coordinate();
      double newDist;
      if ((internalGeo instanceof Point) || (internalGeo instanceof LineString)) {
        newDist = getShortestDistance(internalGeo, geom1, newcoord2, newcoord1);
        if (newDist < limitDistance) {
          coord1.setCoordinate(newcoord1);
          coord2.setCoordinate(newcoord2);
          limitDistance = newDist;
        }
      } else if (internalGeo instanceof Polygon) {
        if (isMultiGeometry(internalGeo)) {
          newDist = distance(internalGeo, geom1, limitDistance, newcoord2, newcoord1);
        } else {
          newDist = getShortestDistance(internalGeo, geom1, newcoord2, newcoord1);
        }
        if (newDist < limitDistance) {
          coord1.setCoordinate(newcoord1);
          coord2.setCoordinate(newcoord2);
          limitDistance = newDist;
        }
      } else { //if (isMultiGeometry(internalGeo)) {
        newDist = distance(geom1, internalGeo, limitDistance, newcoord1, newcoord2);
        if (newDist < limitDistance) {
          coord1.setCoordinate(newcoord1);
          coord2.setCoordinate(newcoord2);
          limitDistance = newDist;
        }
      }
    }
    return limitDistance;
  }

//    private static double distance(Polygon poly1, Geometry geom2, double limitDistance,
//    		final Coordinate coord1, final Coordinate coord2)
//    {
//    	Coordinate newcoord1 = new Coordinate();
//    	Coordinate newcoord2 = new Coordinate();
//    	double newDist;
//    	Geometry shell = poly1.getExteriorRing();
//    	if (isMultiGeometry(geom2)) {
//    		newDist = distance(geom2, poly1, limitDistance, newcoord2, newcoord1);
//    	} else {
//    		newDist = getShortestDistance(shell, geom2, newcoord1, newcoord2);
//    	}
//    	if (newDist < limitDistance) {
//    		coord1.setCoordinate(newcoord1);
//    		coord2.setCoordinate(newcoord2);
//    		limitDistance = newDist; 
//    	}
//    	int n = poly1.getNumInteriorRing();
//    	for (int j=0; j<n; j++) {
//    		Geometry hole = ((Polygon) poly1).getInteriorRingN(j);
//    		if (isMultiGeometry(geom2)) {
//    			newDist = distance(geom2, hole, limitDistance, newcoord2, newcoord1);
//    		} else {
//    			newDist = getShortestDistance(hole, geom2, newcoord1, newcoord2);
//    		}
//    		if (newDist < limitDistance) {
//    			coord1.setCoordinate(newcoord1);
//    			coord2.setCoordinate(newcoord2);
//    			limitDistance = newDist; 
//    		}					    			
//    	}   
//        return limitDistance;
//    }


  /**
   * @param geom1,geom2   - Geometry objects to compute distance between.
   * @param coord1,coord2 - preallocated Coordinate points that will return
   *                      with the values of x and y that yielded the measured distance.
   * @return - shortest distance between the two inputs, or Double.NaN if either is null,
   * or Double.NaN if either geometry has multiple geometries.
   */
  public static double getShortestDistance(Geometry geom1, Geometry geom2,
                                           final Coordinate coord1, final Coordinate coord2) {
    //TODO: figure out if we really need 0.00001 flag value
    //a distance of exactly zero is used to flag that there are no exposures
    //a distance of 0.00001 is used to flag that we want to output zero distance with exposure
    //WriteDistance will convert float .00001 to string '0'
    if (geom1 == null || geom2 == null) {
      return Double.NaN;
    }
    if (isMultiGeometry(geom1) || isMultiGeometry(geom2))
      return Double.NaN; //flag this as an error - Multi not supported

    if (geom1.getEnvelopeInternal().intersects(geom2.getEnvelopeInternal())) {
      Coordinate[] coordsOne = geom1.getCoordinates();
      Coordinate[] coordsTwo = geom2.getCoordinates();
      for (int j = 0; j < coordsOne.length - 1; j++) {
        Coordinate p0 = coordsOne[j];
        Coordinate p1 = coordsOne[j + 1];
        for (int k = 0; k < coordsTwo.length - 1; k++) {
          Coordinate p2 = coordsTwo[k];
          Coordinate p3 = coordsTwo[k + 1];
          Coordinate coord = GeoUtils.intersectSegments(p0, p1, p2, p3);
          if (coord != null) {  //a segment of 1 crossed 2
            coord1.x = coord.x;
            coord1.y = coord.y;
            coord2.x = coord.x;
            coord2.y = coord.y;  //both points are the same
            return 0; //if their borders intersect then the distance is zero
          }
        }
      }
    }
    return distanceBetweenTwo(geom1, geom2, coord1, coord2);
  }

  /**
   * @param geom1,geom2 - Geometry objects to compute the distance between.
   *                    Objects with a dimension greater than one should be broken down before
   *                    calling this method (i.e. MultiPolygon, MultiLineString, GeometryCollection).
   * @param coord1,     coord2 -  preallocated Coordinate points that will return
   *                    with the values of x and y that yielded the measured distance.
   * @return - the distance between the two Geometry objects.
   */
  public static double distanceBetweenTwo(Geometry geom1, Geometry geom2,
                                          final Coordinate coord1, final Coordinate coord2) {
    Coordinate[] coordsOne = geom1.getCoordinates();
    Coordinate[] coordsTwo = geom2.getCoordinates();
    coord1.setCoordinate(coordsOne[0]);
    coord2.setCoordinate(coordsTwo[0]);
    if ((coordsOne.length == 1) && (coordsTwo.length == 1)) {
      return coord1.distance(coord2);  //both are points
    }
    Coordinate coord = new Coordinate(0, 0);  //need to pre-allocate before passing
    double distSqd = Double.MAX_VALUE;

    for (int j = 0; j < coordsOne.length; j++) {
      Coordinate pt0 = coordsOne[j];
      for (int k = 0; k < coordsTwo.length - 1; k++) {
        Coordinate pt1 = coordsTwo[k];
        Coordinate pt2 = coordsTwo[k + 1];
        double dist = GeoUtils.getDistanceSqd(pt0, pt1, pt2, coord); //,  which);
        if (dist < distSqd) {
          distSqd = dist;
          coord1.x = pt0.x;
          coord1.y = pt0.y;   //points from geom1 go to coord1
          coord2.x = coord.x;
          coord2.y = coord.y; //points from geom2 go to coord2
        }
      }
    }
    //run the same code with the roles reversed
    for (int j = 0; j < coordsTwo.length; j++) {
      Coordinate pt0 = coordsTwo[j];
      for (int k = 0; k < coordsOne.length - 1; k++) {
        Coordinate pt1 = coordsOne[k];
        Coordinate pt2 = coordsOne[k + 1];
        double dist = GeoUtils.getDistanceSqd(pt0, pt1, pt2, coord);  //,  which);
        if (dist < distSqd) {
          distSqd = dist;
          coord1.x = coord.x;
          coord1.y = coord.y; //points from geom1 go to coord1
          coord2.x = pt0.x;
          coord2.y = pt0.y;   //points from geom2 go to coord2
        }
      }
    }
    return Math.sqrt(distSqd);
  }

  /**
   * @param features      - input list of Feature items (overlaps will be combined).
   * @param featureSchema - FeatureSchema of returned Feature Collection.
   * @return <Feature> Collection of medial axis LineString Features with featureSchema.
   */
  public static Collection<Feature> findMedialAxisCollection(Collection<Feature> features,
                                                             FeatureSchema featureSchema) {
    List<Feature> medialAxisLineStringFeatures = new ArrayList<>();
    Collection<Feature> combinedFeatures = GeoUtils.combineOverlappingFeatures(features, featureSchema);
    for (Feature f : combinedFeatures) {
      CoordinateList coordinateList = new CoordinateList();
      Geometry geo = f.getGeometry();
      coordinateList.add(geo.getCoordinates(), true); //allow repeat coordinates for closed
      CoordinateList coordList = GeoUtils.ConvexHullWrap(coordinateList);
      if (coordList.size() == coordinateList.size()) //convex already?
        coordList = coordinateList; //use original list
      else {
        int sideOne = coordList.indexOf(coordinateList.getCoordinate(0));
        if (sideOne > 0) { //convex hull reordered coordinates?
          Coordinate[] p = coordList.toCoordinateArray();
          for (int k = 0; k < sideOne; k++) {
            //rotate vertex down by one
            int n = p.length - 2;
            Coordinate t = p[0];
            for (int i = 0; i < n; i++)
              p[i] = p[i + 1];
            p[n] = t;
          }
          p[p.length - 1] = p[0];
          coordList = new CoordinateList();
          for (int k = 0; k < p.length; k++)
            coordList.add(p[k], true);
        }
      }

      LineString medialAxis = GeoUtils.findMedialAxis(coordList.toCoordinateArray());
      Feature newFeature = new BasicFeature(featureSchema);
      if (featureSchema.hasAttribute("ASHS_ID") && geo.getUserData() != null) {
        String id = (String) geo.getUserData();
        newFeature.setAttribute("ASHS_ID", id);
        geo.setUserData(null);
      }
      newFeature.setGeometry(medialAxis);
      medialAxisLineStringFeatures.add(newFeature);
    }
    return medialAxisLineStringFeatures;
  }

  /**
   * Find the medial axis of the array coordinates that form a convex hull.
   * The medial axis in this context means the longest bisecting line that falls
   * on the axis of maximal symmetry.
   * See ConvexHullWrap() in this file.
   *
   * @param coords Coordinate[] of a convex hull
   * @return LineString representing the medial axis.
   */
  public static LineString findMedialAxis(Coordinate[] coords) {
    Coordinate p1;
    int n = coords.length;
    n -= 1; //last point is the same as the first
    //for example: could be triangle - n would be 3, or quadralateral - n would be 4
    int n2 = n / 2;
    Coordinate pn2 = coords[n2];
    Coordinate pn0 = coords[0];
    double minSymmetry = Double.MAX_VALUE;
    double symmetry;
    if (n == 3) { //triangle
      for (int i = 0; i < n; i++) {
        int m = GeoUtils.moduloAccess(i, n2, n - 1);
        int m1 = GeoUtils.moduloAccess(m, 1, n - 1);
        p1 = new Coordinate((coords[m].x + coords[m1].x) / 2, (coords[m].y + coords[m1].y) / 2);
        symmetry = measureSymmetry(coords[i], p1, coords);
        if (symmetry < minSymmetry) {
          minSymmetry = symmetry;
          pn0 = coords[i];
          pn2 = p1;
        }
//  			addDebugLine(coords[i], p1, Integer.toString(i), Double.toString(symmetry));
      }
    } else if (n == 4) { //quadralateral, possibly a rectangle
      for (int i = 0; i < n; i++) {
        int m = GeoUtils.moduloAccess(i, n2, n - 1);
        int m1 = GeoUtils.moduloAccess(m, 1, n - 1);
        int i1 = GeoUtils.moduloAccess(i, 1, n - 1);
        Coordinate p0 = new Coordinate((coords[i].x + coords[i1].x) / 2, (coords[i].y + coords[i1].y) / 2);
        p1 = new Coordinate((coords[m].x + coords[m1].x) / 2, (coords[m].y + coords[m1].y) / 2);
        symmetry = measureSymmetry(p0, p1, coords);
        symmetry -= p0.distance(p1) * .001;
        if (symmetry < minSymmetry) {
          minSymmetry = symmetry;
          pn0 = p0;
          pn2 = p1;
        }
//  			addDebugLine(p0, p1, Integer.toString(i), Double.toString(symmetry));
      }
    } else {
      for (int i = 0; i <= n; i++) {
        p1 = getPolarOpposite(i, true, coords);
        symmetry = measureSymmetry(coords[i], p1, coords);
        if (i == 0) //give a slight bonus to the line from the first point
          symmetry -= coords[i].distance(p1) * .001;
        if (symmetry < minSymmetry) {
          minSymmetry = symmetry;
          pn0 = coords[i];
          pn2 = p1;
        }
//  			addDebugLine(coords[i], p1, Integer.toString(i), Double.toString(symmetry));
        p1 = getPolarOpposite(i, false, coords);
        symmetry = measureSymmetry(coords[i], p1, coords);
        if (i == 0) //give a slight bonus to the line from the first point
          symmetry -= coords[i].distance(p1) * .001;
        if (symmetry < minSymmetry) {
          minSymmetry = symmetry;
          pn0 = coords[i];
          pn2 = p1;
        }
//  			addDebugLine(coords[i], p1, Integer.toString(i), Double.toString(symmetry));
        int m = GeoUtils.moduloAccess(i, n2, n - 1);
        p1 = coords[m];
        symmetry = measureSymmetry(coords[i], p1, coords);
        if (i == 0) //give a slight bonus to the line from the first point
          symmetry -= coords[i].distance(p1) * .001;
        if (symmetry < minSymmetry) {
          minSymmetry = symmetry;
          pn0 = coords[i];
          pn2 = p1;
        }
//  			addDebugLine(coords[i], p1, Integer.toString(i), Double.toString(symmetry));
        m = GeoUtils.moduloAccess(i, n2, n - 1);
        int m1 = GeoUtils.moduloAccess(m, 1, n - 1);
        int i1 = GeoUtils.moduloAccess(i, 1, n - 1);
        Coordinate p0 = new Coordinate((coords[i].x + coords[i1].x) / 2, (coords[i].y + coords[i1].y) / 2);
        p1 = new Coordinate((coords[m].x + coords[m1].x) / 2, (coords[m].y + coords[m1].y) / 2);
        symmetry = measureSymmetry(p0, p1, coords);
        if (i == 0) //give a slight bonus to the line from the first point
          symmetry -= coords[i].distance(p1) * .001;
        if (symmetry < minSymmetry) {
          minSymmetry = symmetry;
          pn0 = p0;
          pn2 = p1;
        }
//  			addDebugLine(p0, p1, Integer.toString(i), Double.toString(symmetry));
      }
    }
    Coordinate[] line = new Coordinate[2];
    line[0] = pn0;
    line[1] = pn2;
    return new GeometryFactory().createLineString(line);
  }

  /**
   * Evaluate the symmetry about the line from p0 to p1.
   *
   * @return a double proportional to the amount of symmetry about the p0-p1 axis.
   * Perfect symmetry scores 0.  Less perfect symmetry scores increase by the distance
   * difference between the left and right perpendicular distances to the axis
   * starting from Pi and working in both directions.
   */
  private static double measureSymmetry(Coordinate p0, Coordinate p1, Coordinate[] coords) {
    //score the symmetry based on a series of intersecting lines perpendicular to the medial axis
    if (p0.equals(p1))
      return 1e9;
    double symmetryTotal = 0;
    //Coordinate p0 = coords[i];
    int n = 20; //set this suitable large but not too large for efficiency
    double di = p0.distance(p1) / n;
    double dist = 1e9;  //suitably far out
    double symmetry = 0;
    for (int k = 1; k < n; k++) {
      Coordinate pt = GeoUtils.along(di * k, p0, p1); //find the next point along the axis
      Coordinate p2 = GeoUtils.perpendicularVector(pt, p0, dist, true);  //left
      Coordinate p3 = GeoUtils.perpendicularVector(pt, p0, dist, false); //right
      Coordinate[] pts = intersections(p2, p3, coords); //find the intersection points
      if (pts[0] != null && pts[1] != null)
        symmetry = Math.abs(pt.distance(pts[0]) - pt.distance(pts[1]));
      symmetryTotal += symmetry;
    }
    return Math.round(symmetryTotal * 1000.0) / 1000.0; //round to nearest .001
  }

  private static Coordinate[] intersections(Coordinate p0, Coordinate p1, Coordinate[] coords) {
    Coordinate[] line = new Coordinate[4]; //4 just in case we happen to hit a vertex
    int lineCount = 0;
    int n = coords.length - 1; //max index of coords
    for (int k = 0; k < n; k++) {
      Coordinate p2 = coords[k];
      Coordinate p3 = coords[k + 1];
      Coordinate coord = intersectSegmentsExact(p0, p1, p2, p3);
      if (coord != null) {  //a segment crossed
        line[lineCount] = coord;
        lineCount++;
      }
    }
    return line;
  }

  public static Coordinate intersectSegmentsExact(Coordinate p0,
                                                  Coordinate p1, Coordinate p2, Coordinate p3) {
    //find intersection of two line segment that meet the criteria expressed by onp0p1 & onp2p3
    double Vx = (p1.x - p0.x);
    double Vy = (p1.y - p0.y);
    double Wx = (p3.x - p2.x);
    double Wy = (p3.y - p2.y);
    double d = Wy * Vx - Wx * Vy;

    if (d != 0.0) {
      double n1 = Wy * (p2.x - p0.x) - Wx * (p2.y - p0.y);
      double n2 = Vy * (p2.x - p0.x) - Vx * (p2.y - p0.y);
      double t1 = n1 / d;
      double t2 = n2 / d;
      boolean onp0p1 = (t1 >= 0.0) && (t1 <= 1.0);
      boolean onp2p3 = (t2 >= 0.0) && (t2 <= 1.0);
      if (onp0p1 && onp2p3) {
        return new Coordinate((p0.x + Vx * t1), (p0.y + Vy * t1)); //intersection point
      } else
        return null; //the intersection point does not lie on both segments
    } else //lines are parallel; no intersection
    {
      return null;
    }
  }

  /**
   * @param i      - coords[i] is the point to find the polar opposite of
   * @param coords - the Coordinate array of the convex hull to evaluate
   * @return - the Coordinate of the point that is the polar opposite of coords[i]
   * (snapped to nearest point or center)
   */
  public static Coordinate getPolarOpposite(int i, boolean center, Coordinate[] coords) {
    double perimiter = 0;
    int n = coords.length - 1;
    for (int k = 0; k < n; k++) {
      Coordinate p0 = coords[k];
      Coordinate p1 = coords[k + 1];
      double distance = p0.distance(p1);
      perimiter += distance;
    }
    double perimHalf = perimiter / 2;
    perimiter = 0;
    int m = 0;
    int m1 = 0;
    for (int k = 0; perimiter < perimHalf; k++) {
      m = GeoUtils.moduloAccess(i, k, n);
      m1 = GeoUtils.moduloAccess(i, k + 1, n);
      perimiter += coords[m].distance(coords[m1]);
    }
    //the polar opposite is at distance d between coords[m] and coords[m1]
    Coordinate snap;
    if (center) {
      snap = new Coordinate((coords[m].x + coords[m1].x) / 2, (coords[m].y + coords[m1].y) / 2);
    } else {
      double d = perimiter - perimHalf;
      Coordinate p = GeoUtils.along(d, coords[m], coords[m1]);
      if (p.distance(coords[m]) < p.distance(coords[m1]))
        snap = coords[m];
      else
        snap = coords[m1];
    }
    return snap;
  }

  /**
   * @param features      - Collection of Feature items to combine
   * @param featureSchema - the Schema for returned Feature items (blank Attributes)
   * @return a Collection of new Feature items with Geometry intersecting items
   * combined into GeometryCollections.  Open LineStrings will be discarded, but closed
   * LineStrings will be considered Polygons.
   */
  public static Collection<Feature> combineOverlappingFeatures(Collection<Feature> features, FeatureSchema featureSchema) {
    //discard open LineString elements
    List<Geometry> noLineStrings = new ArrayList<>();
    boolean hasASHS_ID = featureSchema.hasAttribute("ASHS_ID");
    for (Feature f1 : features) {
      Geometry geo1 = makeClosedLineStringsPolygons(f1.getGeometry());
      if (!(geo1 instanceof LineString)) {
        if (hasASHS_ID) {
          String id = f1.getString("ASHS_ID");
          if (id != null && !id.isEmpty())
            geo1.setUserData(id);
        }
        noLineStrings.add(geo1);
      }
    }
    GeometryFactory geometryFactory = new GeometryFactory();
    List<Geometry> alreadyProcessed = new ArrayList<>();
    List<Feature> combinedFeatures = new ArrayList<>();
    for (Geometry geo1 : noLineStrings) {
      if (alreadyProcessed.contains(geo1)) continue;
      List<Geometry> overlappingGeometries = new ArrayList<>();
      overlappingGeometries.add(geo1);
      for (Geometry geo2 : noLineStrings) {
        if (geo1 == geo2 || alreadyProcessed.contains(geo2)) continue;
        if (geo1.intersects(geo2)) {
          overlappingGeometries.add(geo2);
          alreadyProcessed.add(geo2);
        }
      }
      //create GeometryCollection out of the overlappingGeometries with geo1
      Geometry geo3 = geometryFactory.buildGeometry(overlappingGeometries);
      String id = "";
      if (hasASHS_ID) {
        for (Geometry geo : overlappingGeometries) {
          String id2 = (String) geo.getUserData();
          if (id2 != null && !id2.isEmpty()) {
            id = id2;
            break;
          }
        }
      }
      Feature newFeature = new BasicFeature(featureSchema);
      newFeature.setGeometry(geo3);
      if (hasASHS_ID && !id.isEmpty()) {
        newFeature.setAttribute("ASHS_ID", id);
      }
      combinedFeatures.add(newFeature);
    }
    return combinedFeatures;
  }

  public static Geometry makeClosedLineStringsPolygons(Geometry geometry) {
    if (geometry instanceof LineString) {
      GeometryFactory gf = geometry.getFactory();
      Coordinate[] coords = geometry.getCoordinates();
      int n = coords.length;
      if (n > 2 && coords[0].equals(coords[n - 1])) {
        geometry = gf.createPolygon(gf.createLinearRing(coords));
      }
    }
    return geometry;
  }

}