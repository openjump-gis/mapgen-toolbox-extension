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
import mapgen.algorithms.polygons.BuildingDeletePointsInLine;
import mapgen.constraints.buildings.BuildingPointInLine;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureDatasetFactory;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.task.TaskMonitor;
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
import com.vividsolutions.jump.workbench.ui.zoom.ZoomToSelectedItemsPlugIn;

/**
 * Deletes the points on a line which could emerge from the union operation of
 * adjacent polygons.
 * It deletes the points only if the angle is about 180degrees and the
 * distance (point, to new building wall) is smaller than a threshold
 * <p>
 * created:  		26.11.2005
 *
 * @author sstein
 */
public class EliminatePointsInLineOfBuildingPlugIn extends AbstractPlugIn implements ThreadedPlugIn {

  private final ZoomToSelectedItemsPlugIn myZoom = new ZoomToSelectedItemsPlugIn();
  private static final String T1 = "MapScale";
  private int scale = 1;
  private FeatureDataset elimFeatures = null;

  public void initialize(PlugInContext context) throws Exception {
    context.getFeatureInstaller().addMainMenuPlugin(this,
        new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Scale Dependent Algorithms", "Buildings"},  //menu path
        "Eliminate Points in Line of Selected Buildings",
        false,        null,
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
        "Eliminate points on a line (wall of) buidlings where a change of wall direction is not visible for the specific scale");
    dialog.addIntegerField(T1, 25000, 7, T1);
  }

  private void getDialogValues(MultiInputDialog dialog) {
    this.scale = dialog.getInteger(T1);
  }

  public void run(TaskMonitor monitor, PlugInContext context) throws Exception {

    //this.zoom2Feature(context);
    FeatureCollection fc = this.eliminate(context, this.scale, monitor);
    context.addLayer(StandardCategoryNames.WORKING, "buildings wall simplify", fc);
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

  private FeatureCollection eliminate(PlugInContext context, int scale,
                                      TaskMonitor monitor) {

    double flexInRad = 10.0 * Math.PI / 180;
    // --------------------------
    //-- get selected items
    final Collection<Feature> features =
        context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();

    int count = 0;
    int noItems = features.size();
    //Geometry resultgeom = null;

    //--get single object in selection to analyse
    FeatureDataset resultFeatures = null;
    FeatureDataset elimFeatures = null;
    //List<LineString> problematicEdges = new ArrayList<>();
    List<Polygon> errorBdgs = new ArrayList<>();
    //List resultList = new ArrayList();
    FeatureSchema fs = new FeatureSchema();
    int modified = 0;
    for (Feature ft : features) {
      count++;
      //-- clone to avoid that original features get changed
      Feature f = ft.clone();
      if (count == 1) {
        //-- not sure to do that, since feature schemas of selected objects might be different
        fs = copyFeatureSchema(f.getSchema());
        resultFeatures = new FeatureDataset(fs);
        elimFeatures = new FeatureDataset(fs);
      }
      Geometry geom = f.getGeometry(); //= erste Geometrie
      Polygon poly;
      if (geom instanceof Polygon) {
        poly = (Polygon) geom; //= erste Geometrie
        // ---- init goals
        BuildingGoals goals = new BuildingGoals(scale);
        //---- detect point in line conflicts
        BuildingPointInLine bpil = new BuildingPointInLine(poly, goals.getPositionAccuracy(), flexInRad);
        if (!bpil.isfullfilled()) {
          try {
            BuildingDeletePointsInLine dpil = new BuildingDeletePointsInLine(poly, bpil.measure.getConflicList());
            poly = dpil.getOutPolygon();
            f.setGeometry(poly);
            modified++;
          } catch (Exception e) {
            errorBdgs.add((Polygon)f.getGeometry().copy());
            System.out.println("xxxxxxxxxxxxxxxxxxxxxxx");
            System.out.println("Exception: Bdg delete points in line");
            System.out.println(e);
            System.out.println("xxxxxxxxxxxxxxxxxxxxxxx");
          }
        }
        resultFeatures.add(f);
      } else {
        context.getWorkbenchFrame().warnUser("no polygon selected");
      }
      String mytext = "item: " + count + " / " + noItems + " : tested";
      monitor.report(mytext);
    }// end loop over item selection
    if (errorBdgs.size() > 0) {
      FeatureCollection myCollE = FeatureDatasetFactory.createFromGeometry(errorBdgs);
      if (myCollE.size() > 0) {
        context.addLayer(StandardCategoryNames.WORKING, "Exception Bdg", myCollE);
      }
    }
    context.getWorkbenchFrame().setStatusMessage("polygons modified: " + modified + " from: " + count);
    this.elimFeatures = elimFeatures;
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
