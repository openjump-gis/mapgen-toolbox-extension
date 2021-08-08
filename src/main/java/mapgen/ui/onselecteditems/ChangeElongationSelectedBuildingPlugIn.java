/*
 * The Unified Mapping Platform (JUMP) is an extensible, interactive GUI 
 * for visualizing and manipulating spatial features with geometry and attributes.
 *
 * JUMP is Copyright (C) 2003 Vivid Solutions
 *
 * This program implements extensions to JUMP and is
 * Copyright (C) Stefan Steiniger.
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
 * Stefan Steiniger
 * perriger@gmx.de
 */
package mapgen.ui.onselecteditems;

import mapgen.algorithms.polygons.PolygonChangeElongation;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.WorkbenchContext;
import com.vividsolutions.jump.workbench.plugin.AbstractPlugIn;
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.plugin.ThreadedPlugIn;
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.MultiInputDialog;
import com.vividsolutions.jump.workbench.ui.plugin.FeatureInstaller;
import com.vividsolutions.jump.workbench.ui.zoom.*;
import java.util.ArrayList;
import java.util.Iterator;
import com.vividsolutions.jump.workbench.model.StandardCategoryNames;
import java.util.Collection;

/**
 * Changes the elongation of a building / polygon
 * by a given ScaleFactor using PolygonChangeElongation algorithm
 * (Angle and center point can be given as well if
 * plugin will be changed)
 *
 * created:  		07.01.2004
 * @author sstein
 */
public class ChangeElongationSelectedBuildingPlugIn extends AbstractPlugIn implements ThreadedPlugIn{

    private final ZoomToSelectedItemsPlugIn myZoom = new ZoomToSelectedItemsPlugIn();
    private static final String T1 = "scalefactor";
    double scale = 1;

    public void initialize(PlugInContext context) throws Exception {
        context.getFeatureInstaller().addMainMenuPlugin(this,
						new String[] {MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Not Scale Dependent Algorithms" ,"Buildings"}, 	//menu path
						"Change Elongation of Selected Buildings", //name methode .getName recieved by AbstractPlugIn
						false,null,
						getEnableCheck(context)); //enable check
    }
    
    public MultiEnableCheck getEnableCheck(final PlugInContext context) {
        EnableCheckFactory checkFactory = context.getCheckFactory();

        return new MultiEnableCheck()
						.add(checkFactory.createWindowWithLayerNamePanelMustBeActiveCheck())
            .add(checkFactory.createAtLeastNItemsMustBeSelectedCheck(1));
    }
    
	public boolean execute(PlugInContext context) throws Exception{
	    MultiInputDialog dialog = new MultiInputDialog(
	            context.getWorkbenchFrame(), getName(), true);
	        setDialogValues(dialog, context);
	        GUIUtil.centreOnWindow(dialog);
	        dialog.setVisible(true);
	        if (! dialog.wasOKPressed()) { return false; }
	        getDialogValues(dialog);
	        return true;
	}
	
    private void setDialogValues(MultiInputDialog dialog, PlugInContext context)
	  {
	    dialog.setSideBarDescription(
	        "Change Building Elongation");
	    dialog.addDoubleField(T1, 1.0, 3);
	  }

	private void getDialogValues(MultiInputDialog dialog) {
	    this.scale = dialog.getDouble(T1);
	  }

    public void run(TaskMonitor monitor, PlugInContext context) throws Exception{
        
	    //this.zoom2Feature(context);	    
	    FeatureCollection fc = this.changeElong(context, this.scale, monitor);
	    context.addLayer(StandardCategoryNames.WORKING, "modified buildings", fc); 
	    System.gc();          		
    	}
	
	/**
	 * centers the selected feature
	 * @param context plugin context
	 * @throws Exception
	 */
	private void zoom2Feature(PlugInContext context) throws Exception {
		    
	    this.myZoom.execute(context);	    
	}

	private FeatureDataset changeElong(PlugInContext context, double scale, TaskMonitor monitor) throws Exception{
		
		System.gc(); //flush garbage collector
		// --------------------------	    
		//-- get selected items
		final Collection<Feature> features =
				context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();
		
		int count=0; int noItems = features.size();
		//Geometry resultgeom = null;
		
		//--get single object in selection to analyse
		FeatureDataset resultFeatures = null;
		FeatureDataset elimFeatures = null;
		//ArrayList problematicEdges = new ArrayList();
		//List resultList = new ArrayList();
		FeatureSchema fs = new FeatureSchema();
		int eliminated = 0;
		for (Feature ft : features) {
			count++;
			//-- clone to avoid that original features get changed
			Feature f = ft.clone();
			if (count == 1){      			
				//-- not sure to do that, since feature schemas of selected objects might be different 
				fs = copyFeatureSchema(f.getSchema());
				resultFeatures = new FeatureDataset(fs);
				elimFeatures = new FeatureDataset(fs);
			}      		
			Geometry geom = f.getGeometry(); //= erste Geometrie
			Polygon poly;
			if ( geom instanceof Polygon){
				poly = (Polygon) geom; //= erste Geometrie
				PolygonChangeElongation pce = new PolygonChangeElongation(poly,scale);
				f.setGeometry(pce.getOutPolygon());
				resultFeatures.add(f);	        
			}
			else{
				context.getWorkbenchFrame().warnUser("no polygon selected");
			}
			String mytext = "item: " + count + " / " + noItems;
			monitor.report(mytext);	       		       	
		}// end loop over item selection
		context.getWorkbenchFrame().warnUser("eliminated: " + eliminated + " from: " + count);
		return resultFeatures;        
	}
	
	private FeatureSchema copyFeatureSchema(FeatureSchema oldSchema){
		FeatureSchema fs = new FeatureSchema();
		for (int i = 0; i < oldSchema.getAttributeCount(); i++) {
			AttributeType at = oldSchema.getAttributeType(i);
			String aname = oldSchema.getAttributeName(i);
			fs.addAttribute(aname,at);
			fs.setCoordinateSystem(oldSchema.getCoordinateSystem());
		}		
		return fs;
	}
	
  
}
