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


import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import mapgen.agents.goals.BuildingGoals;
import mapgen.algorithms.polygons.BuildingEnlargeWidthLocaly;
import mapgen.constraints.buildings.BuildingLocalWidth;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureDatasetFactory;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.WorkbenchContext;
import com.vividsolutions.jump.workbench.model.Layer;
import com.vividsolutions.jump.workbench.model.StandardCategoryNames;
import com.vividsolutions.jump.workbench.plugin.AbstractPlugIn;
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.plugin.ThreadedPlugIn;
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.MultiInputDialog;
import com.vividsolutions.jump.workbench.ui.plugin.FeatureInstaller;
import com.vividsolutions.jump.workbench.ui.zoom.ZoomToSelectedItemsPlugIn;

/**
 * Changes locally the width of a building / polygon
 * if the inner distances are to narrow. <p>
 * Only the strongest conflict will be solved. Therefore an
 * iterative solution is possible.
 * to stop the iterative solution (currently 50 steps = max), a change
 * detection should be used since conflicts on buildings with holes
 * influence each other.<p>
 * <p>
 * TODO: find conflict minimum in solutions
 * <p>
 * created:  		07.01.2005
 * last modified:  	16.05.2005 (selection of several items)
 *
 * @author sstein
 **/
public class BuildingSpreadNarrowPartsPlugIn extends AbstractPlugIn implements ThreadedPlugIn {

  private final ZoomToSelectedItemsPlugIn myZoom = new ZoomToSelectedItemsPlugIn();
  private static final String T1 = "MapScale";
  private static final String T4 = "Do only move Edges?";
  private static final String T5 = "Do solve iterative?";
  private static final String T2 = "If YES specify maximum No of iterations?";
  private final String newAttributString = "BuildingSpread";
  int scale = 1;
  boolean fixPoints = false;
  boolean solveIter = true;
  int iterMax = 50;

  public void initialize(PlugInContext context) throws Exception {
    FeatureInstaller featureInstaller = new FeatureInstaller(context.getWorkbenchContext());
    featureInstaller.addMainMenuPlugin(
        this,                //exe
        new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Scale Dependent Algorithms", "Buildings"},  //menu path
        "Spread Narrow Parts of Selected Buildings", //name methode .getName recieved by AbstractPlugIn
        false,      //checkbox
        null,      //icon
        createEnableCheck(context.getWorkbenchContext())); //enable check
  }

  public static MultiEnableCheck createEnableCheck(WorkbenchContext workbenchContext) {
    EnableCheckFactory checkFactory = new EnableCheckFactory(workbenchContext);

    return new MultiEnableCheck()
        .add(checkFactory.createWindowWithLayerNamePanelMustBeActiveCheck())
        .add(checkFactory.createAtLeastNItemsMustBeSelectedCheck(1));
  }

  public boolean execute(PlugInContext context) throws Exception {
    this.reportNothingToUndoYet(context);
    MultiInputDialog dialog = new MultiInputDialog(
        context.getWorkbenchFrame(), getName(), true);
    setDialogValues(dialog, context);
    GUIUtil.centreOnWindow(dialog);
    dialog.setVisible(true);
    if (!dialog.wasOKPressed()) {
      return false;
    }
    getDialogValues(dialog);
    return true;
  }

  private void setDialogValues(MultiInputDialog dialog, PlugInContext context) {
    dialog.setSideBarDescription(
        "Spread narrow parts: map scale is used to detect narrow (inlegible) parts");
    dialog.addIntegerField(T1, 25000, 7, T1);
    dialog.addCheckBox(T5, true);
    dialog.addIntegerField(T2, 30, 4, T2);
    dialog.addCheckBox(T4, false);
  }

  private void getDialogValues(MultiInputDialog dialog) {
    this.scale = dialog.getInteger(T1);
    this.fixPoints = dialog.getBoolean(T4);
    this.solveIter = dialog.getBoolean(T5);
    this.iterMax = dialog.getInteger(T2);
  }

  public void run(TaskMonitor monitor, PlugInContext context) throws Exception {

    FeatureCollection fc = this.changeWidth(context, this.scale, this.fixPoints, this.solveIter, monitor);
    context.addLayer(StandardCategoryNames.WORKING, "spread buildings", fc);
    System.gc();
  }

  /**
   * centers the selected feature
   *
   * @param context the plugin context
   * @throws Exception if an exception occurs during the zoom operation
   */
  private void zoom2Feature(PlugInContext context) throws Exception {

    this.myZoom.execute(context);
  }

  protected Layer layer(PlugInContext context) {
    return context.getLayerViewPanel().getSelectionManager()
        .getLayersWithSelectedItems().iterator().next();
  }


  private FeatureCollection changeWidth(PlugInContext context, int scale,
                                        boolean fixVertex, boolean solveIterative, TaskMonitor monitor) {

    //-- get selected items
    final Collection<Feature> features =
        context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();
    /**
     EditTransaction transaction = new EditTransaction(features, this.getName(), layer(context),
     this.isRollingBackInvalidEdits(context), false, context.getWorkbenchFrame());
     **/
    int count = 0;
    int noItems = features.size();
    //Geometry resultgeom = null;
    FeatureDataset resultFeatures = null;
    List<LineString> problematicEdges = new ArrayList<>();
    List<Point> problematicPoints = new ArrayList<>();
    FeatureSchema fs = new FeatureSchema();
    //--get single object in selection to analyse
    for (Feature f : features) {
      count++;
      //System.out.println("========== Item featureID: " + f.getID() + " ==========");
      //-- set schema by using the first selected item
      //check if attribute name exists
      boolean attributeExists = false;
      if (count == 1) {
        //-- not sure to do that, since feature schemas of selected objects might be different
        fs = copyFeatureSchema(f.getSchema());
        attributeExists = fs.hasAttribute(this.newAttributString);
        if (!attributeExists) {
          fs.addAttribute(this.newAttributString, AttributeType.STRING);
          //problematicFeatures = new FeatureDataset(fs);
        }
        resultFeatures = new FeatureDataset(fs);
      }
      //--create new Feature with one new attribute and copy attributvalues
      Feature fnew = new BasicFeature(fs);
      Object[] attribs = f.getAttributes();
      if (!attributeExists) {
        Object[] attribsnew = new Object[attribs.length + 1];
        for (int i = 0; i < attribs.length; i++) {
          attribsnew[i] = attribs[i];
        }
        attribsnew[attribs.length] = "init";
        fnew.setAttributes(attribsnew);
      } else {
        fnew.setAttributes(attribs);
      }

      Geometry geom = f.getGeometry(); //= erste Geometrie
      Polygon poly;
      if (geom instanceof Polygon) {
        poly = (Polygon) geom; //= erste Geometrie
        // --------------------------
	       		/*
	           	List resultList = new ArrayList();
	           	List conflictListA = new ArrayList();
	           	List conflictListB = new ArrayList();
	           	*/
        //---- detect conflicts
        BuildingGoals goals = new BuildingGoals(scale);
        BuildingLocalWidth plw = new BuildingLocalWidth(poly,
            goals.getMinWidthReal(), goals.getMinWidthFlexibility());
        //---
        if (plw.measure.hasConflicts()) {
          context.getWorkbenchFrame().setStatusMessage("conflicts detected!");
          //conflictListA.addAll(plw.measure.getDispVecPointEdgeLStringList());
          //conflictListB.addAll(plw.measure.getDispVecPointPointLStringList());
          //--- solve conflicts ---
          if (!solveIterative) {
            BuildingEnlargeWidthLocaly enlarge = new BuildingEnlargeWidthLocaly(poly,
                plw.measure.getMwclist(),
                fixVertex);
            //resultList.add(enlarge.getOutPolygon());
            fnew.setGeometry(enlarge.getOutPolygon());
            plw = new BuildingLocalWidth(enlarge.getOutPolygon(),
                goals.getMinWidthReal(), goals.getMinWidthFlexibility());
            if (plw.measure.hasConflicts()) {
              fnew.setAttribute(this.newAttributString, "not solved");
              problematicEdges.addAll(plw.measure.getMinEdgeLineStringList());
              problematicPoints.addAll(plw.measure.getMinVertexPointList());
            } else {
              fnew.setAttribute(this.newAttributString, "enlarged");
            }

            /**
             transaction.setGeometry(count-1,enlarge.getOutPolygon());
             **/
          } else {
            //====================================
            // if solution should be done iterative
            //====================================
            BuildingEnlargeWidthLocaly enlarge = null;
            int j = 0;
            boolean tosolve = plw.measure.hasConflicts();
            while (tosolve) {
              enlarge = new BuildingEnlargeWidthLocaly(poly,
                  plw.measure.getMwclist(),
                  fixVertex);
              poly = enlarge.getOutPolygon();
              plw = new BuildingLocalWidth(poly,
                  goals.getMinWidthReal(), goals.getMinWidthFlexibility());
              fnew.setAttribute(this.newAttributString, "enlarged");
              tosolve = plw.measure.hasConflicts();
              //--notbremse:
              j = j + 1;
              if (j == this.iterMax) {
                tosolve = false;
                context.getWorkbenchFrame().warnUser("stopped at step: " + j);
                fnew.setAttribute(this.newAttributString, "not solved");
                problematicEdges.addAll(plw.measure.getMinEdgeLineStringList());
                problematicPoints.addAll(plw.measure.getMinVertexPointList());
              }

              //-- visualisation
			           	    /*
			               	List stepList = new ArrayList();
			           	    stepList.add(0,enlarge.getOutPolygon());
				    	    FeatureCollection myCollD = FeatureDatasetFactory.createFromGeometry(stepList);
				    	    if (myCollD.size() > 0){
				    		    context.addLayer(StandardCategoryNames.WORKING, "stepList", myCollD);
				    		    }	    	  
				    	    */
            }
            //resultList.add(enlarge.getOutPolygon());
            fnew.setGeometry(enlarge.getOutPolygon());
            /**
             transaction.setGeometry(count-1,enlarge.getOutPolygon());
             **/
          }
          //resultList.addAll(enlarge.getIntersectionPoints());
          // ===== visulisation =====
		           	/*
		    	    FeatureCollection myCollA = FeatureDatasetFactory.createFromGeometry(conflictListA);
		    	    if (myCollA.size() > 0){
		    		    context.addLayer(StandardCategoryNames.WORKING, "point-edge conflicts", myCollA);
		    		    }	           	
	           		FeatureCollection myCollC = FeatureDatasetFactory.createFromGeometry(conflictListB);
	           		if (myCollC.size() > 0){
	           		    context.addLayer(StandardCategoryNames.WORKING, "point-point conflicts", myCollC);
	    		    }           		           	           	
		    	    FeatureCollection myCollB = FeatureDatasetFactory.createFromGeometry(resultList);
		    	    if (myCollB.size() > 0){
		    		    context.addLayer(StandardCategoryNames.WORKING, "result", myCollB);
		    		    }
		    		*/
        }// ========================
        else {
          context.getWorkbenchFrame().setStatusMessage("no conflict detected!");
          fnew.setAttribute(this.newAttributString, "no conflict");
        }
      } else {
        context.getWorkbenchFrame().warnUser("no polygon selected");
        fnew.setAttribute(this.newAttributString, "no polygon");
      }
      resultFeatures.add(fnew);
      String mytext = "item: " + count + " / " + noItems + " : spreading finalized";
      monitor.report(mytext);
    }//  end loop for selection
    /**
     transaction.commit();
     **/
    if (problematicEdges.size() > 0) {
      FeatureCollection myCollE = FeatureDatasetFactory.createFromGeometry(problematicEdges);
      context.addLayer(StandardCategoryNames.WORKING, "problematic edges", myCollE);
      FeatureCollection myCollP = FeatureDatasetFactory.createFromGeometry(problematicPoints);
      context.addLayer(StandardCategoryNames.WORKING, "problematic edges", myCollP);
    }

    return resultFeatures;
  }

  private FeatureSchema copyFeatureSchema(FeatureSchema oldSchema) {
    FeatureSchema fs = new FeatureSchema();
    for (int i = 0; i < oldSchema.getAttributeCount(); i++) {
      AttributeType at = oldSchema.getAttributeType(i);
      String aname = oldSchema.getAttributeName(i);
      fs.addAttribute(aname, at);
      fs.setCoordinateSystem(oldSchema.getCoordinateSystem());
    }
    return fs;
  }


}
