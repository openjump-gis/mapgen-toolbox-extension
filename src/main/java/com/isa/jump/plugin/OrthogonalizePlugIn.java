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

import java.util.ArrayList;
import java.util.Collection;

import javax.swing.ImageIcon;
import javax.swing.JComponent;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.impl.CoordinateArraySequence;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.WorkbenchContext;
import com.vividsolutions.jump.workbench.model.Layer;
import com.vividsolutions.jump.workbench.plugin.AbstractPlugIn;
import com.vividsolutions.jump.workbench.plugin.EnableCheck;
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.plugin.ThreadedPlugIn;
import com.vividsolutions.jump.workbench.ui.EditTransaction;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.MultiInputDialog;
import com.vividsolutions.jump.workbench.ui.SelectionManagerProxy;


public class OrthogonalizePlugIn extends AbstractPlugIn implements ThreadedPlugIn{

	  private final static String SELECTONLYSUPPORTED =
		    "Select only Polygons, MultiPolygons, or closed LineStrings";
    private final static String NEWLAYER = "Copy to new layer";
    private final static String NEWLAYERHINT = "Modify a copy instead of original.";
    private final static String UNSUPPORTEDGEOMETRY="Skipped unsupported geometry";
    private final static String SIDEBARDESCRIPTION = "Orthogonalize Selected Features";
    private final static String PROGRESSMESSAGE="Orthogonalizing";
    private final static String ANGLETOLERANCE = "Angle Tolerance";
    private final static String ANGLETOLERANCEHINT = "Angle in degrees";
    private boolean newLayer = false;
    private boolean disableNewLayer = false;
    private double alignTolerance;
    private int anchor = 0;  //anchor point index
    private int move = 1;    //move point index
    private LinearRing shellRing = null;
    private LinearRing[] holes = null;
    private double angleTolerance = 12d; //ortho angle errors > this should not be fixed 
   
    public void initialize(PlugInContext context) throws Exception {
   		/*
   		JPopupMenu popupMenu = LayerViewPanel.popupMenu();

      featureInstaller.addPopupMenuItem(popupMenu,
            this, "Orthogonalize",
            false, null,  //to do: add icon
            this.createEnableCheck(workbenchContext));
      */
    	context.getFeatureInstaller().addMainMenuPlugin(
    	        this,								//exe
                new String[] {MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Not Scale Dependent Algorithms", "Buildings"}, 	//menu path
                "Orthogonalize", //name methode .getName recieved by AbstractPlugIn 
                false,			//checkbox
                null,			//icon
                getEnableCheck(context)); //enable check
    }
    
    public boolean execute(final PlugInContext context) throws Exception {
        reportNothingToUndoYet(context);
        WorkbenchContext workbenchContext = context.getWorkbenchContext();
        Collection<Layer> layers =
						workbenchContext.getLayerViewPanel().getSelectionManager().getLayersWithSelectedItems();
        if ((layers.size() > 1) || !(layers.iterator().next()).isEditable()) {
        	newLayer = true;
        	disableNewLayer = true;
        } else {
        	newLayer = false;
        	disableNewLayer = false;
        }
        MultiInputDialog dialog = new MultiInputDialog(
            context.getWorkbenchFrame(), getName(), true);
        setDialogValues(dialog, context);
        dialog.setVisible(true);
        if (! dialog.wasOKPressed()) { return false; }
        getDialogValues(dialog);
        
        if (newLayer)
        {
        	CopySelectedItemsToNewLayerPlugIn copySelectedItemsToNewLayer = 
        		new CopySelectedItemsToNewLayerPlugIn();
        	copySelectedItemsToNewLayer.execute(context);
        	Layer layer = copySelectedItemsToNewLayer.getNewLayer();
        	workbenchContext.getLayerViewPanel().getSelectionManager().clear();
 					FeatureCollection fc = layer.getFeatureCollectionWrapper().getWrappee();
					Collection<Feature> features = new ArrayList<>(fc.getFeatures());
					context.getLayerViewPanel().getSelectionManager().getFeatureSelection().selectItems(layer, features);
        }
        return true;
    }
    
      private void setDialogValues(MultiInputDialog dialog, PlugInContext context)
      {
        dialog.setSideBarImage(new ImageIcon(getClass().getResource("ortho.png"))); 
        dialog.setSideBarDescription(SIDEBARDESCRIPTION);
        dialog.addCheckBox(NEWLAYER,newLayer,NEWLAYERHINT);
        dialog.getCheckBox(NEWLAYER).setEnabled(!disableNewLayer);
        dialog.addDoubleField(ANGLETOLERANCE, angleTolerance, 4, ANGLETOLERANCEHINT);
        }

      private void getDialogValues(MultiInputDialog dialog) {
    	 newLayer = dialog.getCheckBox(NEWLAYER).isSelected();
    	 angleTolerance = dialog.getDouble(ANGLETOLERANCE);
      }
    
    public MultiEnableCheck getEnableCheck(final PlugInContext context) {
        EnableCheckFactory checkFactory = context.getCheckFactory();
        return new MultiEnableCheck()
            .add(checkFactory.createWindowWithLayerViewPanelMustBeActiveCheck())
            .add(checkFactory.createAtLeastNFeaturesMustHaveSelectedItemsCheck(1))
            .add(onlySupportedGeometriesMayBeSelectedCheck(context.getWorkbenchContext()));
    }
   
    public EnableCheck onlySupportedGeometriesMayBeSelectedCheck(final WorkbenchContext workbenchContext) {
        return new EnableCheck() {
            public String check(JComponent component) {
	           Collection<Geometry> selectedItems = ((SelectionManagerProxy) workbenchContext
                            .getWorkbench()
                            .getFrame()
                            .getActiveInternalFrame())
                            .getSelectionManager()
                            .getSelectedItems();
	            Geometry selectedGeo = selectedItems.iterator().next();
	            selectedGeo = GeoUtils.makeClosedLineStringsPolygons(selectedGeo);
            
                return    ((selectedGeo instanceof Polygon) 
                		|| (selectedGeo instanceof MultiPolygon))
                    ?  null
                    : (SELECTONLYSUPPORTED);
               }
        };
    }

    public void run(TaskMonitor monitor, PlugInContext context) throws Exception{
		monitor.allowCancellationRequests();
	    orthogonalize(context, monitor);
	}

    
    /**
     * Top level method that iterates thorough selected geometries, handles differing 
     * geometry types, and decomposes MultiPolygons into Polygons.  Supports the 
     * EditTransaction feature that allows a single undo to support multiple edits.
     */
    private boolean orthogonalize(PlugInContext context, TaskMonitor monitor) {
	    final Collection<Feature> features =
					context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();
	    Collection<Layer> layers =
					context.getWorkbenchContext().getLayerViewPanel().getSelectionManager().getLayersWithSelectedItems();
			EditTransaction transaction = new EditTransaction(features, this.getName(), layers.iterator().next(),
				this.isRollingBackInvalidEdits(context), false, context.getWorkbenchFrame());
	    int count=0; 
	    int numItems = features.size(); 
	    Geometry resultgeom;
	    for (Feature f : features) {
	    	Geometry geom = GeoUtils.makeClosedLineStringsPolygons (f.getGeometry());
	    	resultgeom = geom;
	    	shellRing = null;
	    	holes = null;
	    	if (geom instanceof MultiPolygon) {
	    		MultiPolygon multiPolygon = (MultiPolygon) geom;
	    		if (!multiPolygon.isEmpty() && multiPolygon.isValid()) {
	    			int n = multiPolygon.getNumGeometries();
	    			Polygon[] polygons = new Polygon[n];
	    			Polygon poly = null;
	    			for (int i = 0; i < n; i++) {
	    				poly = (Polygon) multiPolygon.getGeometryN(i);
	    				alignPoly(poly);
	    				polygons[i] = new Polygon(shellRing, holes, poly.getFactory()); 
	    			}
			        resultgeom = new MultiPolygon(polygons,poly.getFactory());
	    		}
	    	} else if(geom instanceof Polygon) {
	    		Polygon poly = (Polygon) geom;
	    		alignPoly(poly);
		        resultgeom = new Polygon(shellRing, holes, poly.getFactory());        	
	    	}
	    	else{
	    		context.getWorkbenchFrame().warnUser(UNSUPPORTEDGEOMETRY);
	    	}
	    	String mytext = PROGRESSMESSAGE + ": " + count+1 + " / " + numItems;
	    	monitor.report(mytext);		    	
	    	transaction.setGeometry(f, resultgeom); //commit changes to undo history
	    	count++;
	    } //end for loop over selected objects 
	    transaction.commit();
	    return true;        
	}

    /**
     * Decomposes Polygons into shell and hole LinearRings for alignment.  Clones and 
     * cleans the rings before aligning them.
     * @param poly a Polygon
     */
    private void alignPoly(Polygon poly) {
        LinearRing shell = GeoUtils.removeRedundantPoints( //does a deep clone first!
        		poly.getExteriorRing());
        shellRing = alignPolyRing(shell);
        int nHoles = poly.getNumInteriorRing();
        if (nHoles > 0) {
            holes = new LinearRing[nHoles];
            for (int i=0; i < nHoles; i++) {
	            LineString hole = GeoUtils.removeRedundantPoints(poly.getInteriorRingN(i));
            	holes[i] = alignPolyRing( hole);
            }
        }
    }

    /**
     * Analyze the ring and determine which point is the most likely to be
     * accurate and should remain immovable.  Also determine the point on 
     * either side, desingated the move point, that is likely to have the 
     * less accurate interior angle and so would be the best candidate point to move.
     * Also determine the align tolerance that will be used.
     * @param ring - a polygon represented as a closed linestring
     */
    public void findAnchorAndTolerance(LineString ring)  {
    	Coordinate[] da = GeoUtils.getDistanceAngleArray(ring);
    	int n = da.length;
    	double[] orthoError = new double[n];
    	double orthoErrorMax = 0;
    	double curveAproxErrorSum = 0; //sum the errors from 180 - likely curve aproximations
    	for (int i=0; i<n; i++) {
    		double interiorAngle = da[i].y; //0 to +180
    		double diff180 = Math.abs(180.0 - interiorAngle);
    		double diff90 = Math.abs(90.0 - interiorAngle);
    		if (diff180 < diff90) curveAproxErrorSum += diff180;    			
    		double error = Math.min(diff90, diff180);
    		orthoError[i] = error;
    		orthoErrorMax = Math.max(orthoErrorMax, error);        	
    	}
    	double maxDist = 0;
    	double minDist = Double.MAX_VALUE;
    	int max = 0;
    	for (int i=0; i<n; i++) {
    		double dist = da[i].x;  //x is distance
    		if ( dist > maxDist ) {
    			maxDist = dist;
    			max = i;
    		}
    		if (dist < minDist) minDist = dist;
    	}
    	int cw = (max == n-1) ? 0 : max+1;
    	int ccw = (max == 0) ? n-1 : max-1;
    	anchor = max;
    	if (orthoError[cw] < orthoError[ccw]) {
    		move = anchor;
    		anchor = anchor +1;       	
    	} else {
    		move = anchor +1;    		
    	}
    	if ((orthoErrorMax>angleTolerance) || (curveAproxErrorSum > 10d))
    		alignTolerance = 0; //disable
    	else
    		alignTolerance = Math.round(minDist/2d); //set to half of smallest side
    }
        
    /**
     * @param coords - array of Coordinate points
     * @param rotationPoint - point about which to rotate
     * @param angle - in radians
     */
    public void rotateRing(Coordinate[] coords, Coordinate rotationPoint, double angle) {
    	double rx = rotationPoint.x;
    	double ry = rotationPoint.y;
    	double cosAngle = Math.cos(angle);
    	double sinAngle = Math.sin(angle);
    	for (Coordinate coord : coords) {
    		double x = coord.x - rx;
    		double y = coord.y - ry;
    		coord.x = rx + (x*cosAngle) + (y*sinAngle);
    		coord.y = ry + (y*cosAngle) - (x*sinAngle);
    	}
    }

    /**
     * Align the sides of the LineString to make them orthogonal.  This
     * is done by finding the longest side, desingated the anchor side, and 
     * rotating the entire LineString to this side so that the angle is 0.
     * Aligning the other sides is now a simple matter of setting either the 
     * x or y components equal, depending on the calculated tolerance.
     * @param ring the ring to align
     * @return LinearRing after alignment
     */
    public LinearRing alignPolyRing(LineString ring) { //, int anchor)  {
    	boolean closedRing = ring.isClosed();
    	findAnchorAndTolerance(ring);
    	Coordinate[] coords = ring.getCoordinates();
    	if (alignTolerance > 0d) {
    		int n = coords.length;
    		int maxIndex = n-1;
    		Coordinate anchorPt = new Coordinate(coords[anchor]); //get anchor point
    		double angle = GeoUtils.getBearingRadians( anchorPt, coords[move]);
    		rotateRing(coords, anchorPt, angle);
    		//first make the two sides that include the anchor point orthogonal
    		if ( anchor == n-1 ) {
    			anchor = 0; //last point is duplicate of first point
    		}
    		int i = anchor-1;
    		if ( i < 0 ) {
    			i = maxIndex-1;
    		}
    		alignSide (i,anchor,anchor,coords);
    		i = anchor+1;
    		if ( i > maxIndex ) {
    			i = 1;
    		}
    		alignSide (i,anchor,anchor,coords);
    		i = 2;
    		for (int k=0; k<n; k++) {
    			int j = i-1;
    			if ( i == maxIndex && closedRing) { //don't process last point
    				i = 0;
    			}
    			alignSide(i,j,anchor,coords);
    			i = i+1;
    		} 
    		if (closedRing) {
    			coords[maxIndex].x = coords[0].x; //close polygon
    			coords[maxIndex].y = coords[0].y; //close polygon
    		}
    		rotateRing(coords, anchorPt, -angle);
    	}
			return ring.getFactory().createLinearRing(new CoordinateArraySequence(coords));
    }
		
    /**
     * Align the side defined by (coords[i],coords[j]) so that if either deltaX
     * or deltaY is less than the computed alignTolerance, the x or y components are set
     * equal to the average.
     * @param anchor - this point must not be moved
     * @param coords - array of Coordinate
     */
    private void alignSide(int i, int j, int anchor, Coordinate[] coords) {
    	//Align side Pi-Pj of coords without moving Anchor
    	double dx = coords[j].x - coords[i].x;
    	double dy = coords[j].y - coords[i].y;
    	if ( Math.abs(dx) < alignTolerance) { 
    		if ( i == anchor ) {
    			coords[j].x = coords[anchor].x;
    		} else {
    			if ( j == anchor ) {
    				coords[i].x = coords[anchor].x;
    			} else {
    				coords[i].x = coords[i].x +(dx/2); //take out half the difference
    				coords[j].x = coords[i].x;
    			}
    		}
    	} else if ( Math.abs(dy) < alignTolerance) {
    		if ( i == anchor ) { 
    			coords[j].y = coords[anchor].y;
    		} else {
    			if ( j == anchor ) {
    				coords[i].y = coords[anchor].y;
    			} else {
    				coords[i].y = coords[i].y+(dy/2); //take out half the difference
    				coords[j].y = coords[i].y;
    			}
    		}
    	}
    }
    
}
