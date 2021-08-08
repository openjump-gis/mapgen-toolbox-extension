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

import mapgen.agents.goals.BuildingGoals;
import mapgen.algorithms.polygons.BuildingSquaring;
import mapgen.constraints.buildings.BuildingSquareness;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDatasetFactory;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.WorkbenchContext;
import com.vividsolutions.jump.workbench.plugin.AbstractPlugIn;
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.plugin.ThreadedPlugIn;
import com.vividsolutions.jump.workbench.ui.EditTransaction;
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.MultiInputDialog;
import com.vividsolutions.jump.workbench.ui.plugin.FeatureInstaller;
import com.vividsolutions.jump.workbench.ui.zoom.*;

import java.util.ArrayList;

import com.vividsolutions.jump.workbench.model.Layer;
import com.vividsolutions.jump.workbench.model.StandardCategoryNames;

import java.util.Collection;

/**
 * Rectifies (squares) the walls of a building. Therefore the building main
 * directions are obtained from the longest building walls. The algorithm works
 * with respect to two thresholds:  First it allows a maximum change in the
 * wall angle given by the user; second it respects a maximum point displacement
 * of the corner points calculated from the user given target map scale value.
 * The Algorithm is described by N. Regnauld, A. Edwardes and M. Barrault
 * (ACI Workshop, 1999) and in Agent Work Package D1.
 * <p>
 * created:  		09.01.2004
 * last modified:  	16.05.2005 (undo and selection of several items)
 *
 * @author sstein
 **/
public class SquareBuildingPlugIn extends AbstractPlugIn implements ThreadedPlugIn {

  private final ZoomToSelectedItemsPlugIn myZoom = new ZoomToSelectedItemsPlugIn();
  private static final String T1 = "MapScale";
  private static final String T2 = "Max angle change in degrees";
  private static final String T3 = "Max allowed area change in percent";
  int scale = 25000;
  double maxAngle = 15.0;
  double allowedAreaChange = 10;

  public void initialize(PlugInContext context) throws Exception {
    context.getFeatureInstaller().addMainMenuPlugin(
        this,                //exe
        new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Scale Dependent Algorithms", "Buildings"},  //menu path
        "Square Selected Buildings", //name methode .getName recieved by AbstractPlugIn
        false,      //checkbox
        null,      //icon
        getEnableCheck(context)); //enable check
  }

  public MultiEnableCheck getEnableCheck(final PlugInContext context) {
    EnableCheckFactory checkFactory = context.getCheckFactory();

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
        "Square Building: MapScale influences max point displacement. The value for 'Max allowed area change in percent' is used to identify transformation errors. You can try then the 'Orthogonalize' function as well.");
    dialog.addIntegerField(T1, scale, 7, T1);
    dialog.addDoubleField(T2, maxAngle, 4);
    dialog.addDoubleField(T3, allowedAreaChange, 4);
  }

  private void getDialogValues(MultiInputDialog dialog) {
    this.maxAngle = dialog.getDouble(T2);
    this.scale = dialog.getInteger(T1);
    this.allowedAreaChange = dialog.getDouble(T3);
  }

  public void run(TaskMonitor monitor, PlugInContext context) throws Exception {

    monitor.allowCancellationRequests();
    //this.zoom2Feature(context);
    this.square(context, this.scale, this.maxAngle, this.allowedAreaChange, monitor);
    System.gc();
  }

  /**
   * centers the selected feature
   *
   * @param context plugin context
   * @throws Exception if an exception occurs
   */
  private void zoom2Feature(PlugInContext context) throws Exception {

    this.myZoom.execute(context);
  }

  protected Layer layer(PlugInContext context) {
    return context.getLayerViewPanel().getSelectionManager()
        .getLayersWithSelectedItems().iterator().next();
  }

  private boolean square(PlugInContext context, int scale,
                         double maxDevAngle, double allowedAreaChangeInPercent, TaskMonitor monitor) throws Exception {

    //-- get selected items
    final Collection<Feature> features =
        context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();
    ArrayList<Geometry> notTransformedBuildings = new ArrayList<>();
    ArrayList<Geometry> resultErrorGeoms = new ArrayList<>();

    EditTransaction transaction = new EditTransaction(features, this.getName(), layer(context),
        this.isRollingBackInvalidEdits(context), false, context.getWorkbenchFrame());

    int count = 0;
    int noItems = features.size();
    //Geometry resultgeom = null;
    //--get single object in selection to analyse
    for (Feature f : features) {
      count++;
      Geometry geom = f.getGeometry(); //= erste Geometrie
      Polygon poly;
      if (geom instanceof Polygon) {
        poly = (Polygon) geom;
        // --------------------------
        //---- detect conflicts
        BuildingGoals goals = new BuildingGoals(scale);
        //-- tolerance value is set to zero tolerance
        BuildingSquareness bs = new BuildingSquareness(poly, 0.0);
        double posAccuracy = 3 * goals.getPositionAccuracyReal();
        //---
        if ((bs.getSeverity() > 0)) {
          context.getWorkbenchFrame().setStatusMessage("conflicts detected!");

          BuildingSquaring squaring = new BuildingSquaring(poly, this.maxAngle, posAccuracy);
          //BuildingSquaring squaring = new BuildingSquaring(poly,this.maxAngle);
          Polygon pout = squaring.getOutPolygon();
          double orgArea = poly.getArea();
          double areaChange = (Math.abs(pout.getArea() - orgArea)) / ((orgArea) / 100.0);
          if (pout.isValid() && (areaChange <= allowedAreaChangeInPercent)) {
            transaction.setGeometry(f, pout);
          } else {
            System.out.println("Feature with ID " + f.getID() + " maybe invalid or large areaChange in Percent: " + areaChange + " <=>  limit: " + allowedAreaChangeInPercent);
            notTransformedBuildings.add(poly);
            resultErrorGeoms.add(pout);
          }
        } else {
          context.getWorkbenchFrame().setStatusMessage("no conflict detected!");
        }
      } else {
        context.getWorkbenchFrame().warnUser("item is not a polygon");
      }
      String mytext = "item: " + count + " / " + noItems + " : squaring finalized";
      monitor.report(mytext);
    }//end loop for selection
    transaction.commit();
    if (context != null) {
      if (notTransformedBuildings.size() > 0) {
        FeatureCollection fcNonTransformed =
            FeatureDatasetFactory.createFromGeometry(notTransformedBuildings);
        context.addLayer(StandardCategoryNames.SYSTEM, "not squared original", fcNonTransformed);
      }
      if (resultErrorGeoms.size() > 0) {
        FeatureCollection fcResTransformed =
            FeatureDatasetFactory.createFromGeometry(resultErrorGeoms);
        context.addLayer(StandardCategoryNames.SYSTEM, "invalid results", fcResTransformed);
      }
    }
    return true;
  }


}
