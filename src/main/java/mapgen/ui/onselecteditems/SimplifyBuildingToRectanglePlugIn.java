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

import mapgen.algorithms.polygons.BuildingEnlargeToRectangle;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.WorkbenchContext;
import com.vividsolutions.jump.workbench.plugin.AbstractPlugIn;
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.plugin.ThreadedPlugIn;
import com.vividsolutions.jump.workbench.ui.EditTransaction;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.plugin.FeatureInstaller;

import java.util.Iterator;

import com.vividsolutions.jump.workbench.model.Layer;

import java.util.Collection;

/**
 * Changes a building / polygon to a rectangle. <p>
 * The solution algorithm deletes the holes. <p>
 * <p>
 * created:  		02.07.2004
 *
 * @author sstein
 */
public class SimplifyBuildingToRectanglePlugIn extends AbstractPlugIn implements ThreadedPlugIn {

  public void initialize(PlugInContext context) throws Exception {
    FeatureInstaller featureInstaller = new FeatureInstaller(context.getWorkbenchContext());
    featureInstaller.addMainMenuPlugin(
        this,                //exe
        new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Not Scale Dependent Algorithms", "Buildings"},  //menu path
        "Simplify Selected Building to Rectangle", //name methode .getName recieved by AbstractPlugIn
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
    return true;
  }

  public void run(TaskMonitor monitor, PlugInContext context) throws Exception {

    //this.zoom2Feature(context);
    this.changeToRectangle(context, monitor);
    System.gc();
  }

  protected Layer layer(PlugInContext context) {
    return context.getLayerViewPanel().getSelectionManager()
        .getLayersWithSelectedItems().iterator().next();
  }

  private boolean changeToRectangle(PlugInContext context, TaskMonitor monitor) {

    //-- get selected items
    final Collection<Feature> features =
        context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();

    EditTransaction transaction = new EditTransaction(features, this.getName(), layer(context),
        this.isRollingBackInvalidEdits(context), false, context.getWorkbenchFrame());

    int count = 0;
    int noItems = features.size();
    //--get single object in selection to analyse
    for (Feature f : features) {
      count++;
      Geometry geom = f.getGeometry(); //= erste Geometrie
      Polygon poly;
      if (geom instanceof Polygon) {
        poly = (Polygon) geom; //= erste Geometrie
        BuildingEnlargeToRectangle enlarge = new BuildingEnlargeToRectangle(poly);
        transaction.setGeometry(f, enlarge.getOutPolygon());
      } else {
        context.getWorkbenchFrame().warnUser("no polygon selected");
      }
      String mytext = "item: " + count + " / " + noItems + " : squaring finalized";
      monitor.report(mytext);
    }
    transaction.commit();
    return true;
  }

}
