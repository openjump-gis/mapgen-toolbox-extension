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

import mapgen.algorithms.jtssimplify.DouglasPeuckerSimplifier;
import mapgen.algorithms.jtssimplify.TopologyPreservingSimplifier;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
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
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.MultiInputDialog;
import com.vividsolutions.jump.workbench.ui.plugin.FeatureInstaller;

import com.vividsolutions.jump.workbench.model.Layer;

import java.util.Collection;

/**
 * Simplifies a selected line, criterion is a maximal line displacement <p>
 * it is used the JTS 1.5 douglas peucker simplification with topology
 * preservation for polygons and DouglasPeuckerSimplifier for linestrings
 * n.b.: the jts-algorithm handles all geometry types
 * <p>
 * created:  		30.05.2005
 * last modified:  	31.05.2005
 *                  15.05.2021 deprecated by mmichaud (included in OpenJUMP)
 *
 * @author sstein
 * @deprecate use OpenJUMP geometry functions instead
 */
@Deprecated
public class LineSimplifyJTS15AlgorithmPlugIn extends AbstractPlugIn implements ThreadedPlugIn {

  private static final String T3 = "maximum point displacement in meter:";
  double maxPDisp = 0;
  private int geomType = 0; // 1 = line, 2= polygon, 0 = others,

  public void initialize(PlugInContext context) throws Exception {
    FeatureInstaller featureInstaller = new FeatureInstaller(context.getWorkbenchContext());
    featureInstaller.addMainMenuPlugin(
        this,                //exe
        new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Not Scale Dependent Algorithms", "Lines"},  //menu path
        "Simplify Selected Lines (JTS Algorithm)", //name methode .getName recieved by AbstractPlugIn
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
        "Line simplification for a selected line or polygon");
    dialog.addDoubleField(T3, 1.0, 5);
  }

  private void getDialogValues(MultiInputDialog dialog) {
    this.maxPDisp = dialog.getDouble(T3);

  }

  protected Layer layer(PlugInContext context) {
    return context.getLayerViewPanel().getSelectionManager()
        .getLayersWithSelectedItems().iterator().next();
  }

  public void run(TaskMonitor monitor, PlugInContext context) throws Exception {

    monitor.allowCancellationRequests();
    this.simplify(context, this.maxPDisp, monitor);
    System.gc();
  }


  private boolean simplify(PlugInContext context, double maxDisp, TaskMonitor monitor) {

    //-- get selected items
    final Collection<Feature> features =
        context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();

    EditTransaction transaction = new EditTransaction(features, this.getName(), layer(context),
        this.isRollingBackInvalidEdits(context), false, context.getWorkbenchFrame());

    int count = 0;
    int noItems = features.size();
    Geometry resultgeom = null;
    //--get single object in selection to analyse
    for (Feature f : features) {
      count++;
      //System.out.println("========= simplify item: " + count + " ============ ");
      Geometry geom = f.getGeometry(); //= erste Geometrie
      LineString line = null;
      Polygon poly = null;
      if (geom instanceof LineString) {
        line = (LineString) geom;
        this.geomType = 1;
      } else if (geom instanceof Polygon) {
        poly = (Polygon) geom;
        line = poly.getExteriorRing();
        this.geomType = 2;
      } else {
        this.geomType = 0;
        context.getWorkbenchFrame().warnUser("geometry not line or polygon");
      }
      //****************************************
      if (this.geomType > 0) {
        //-- update geometry --------
        if (this.geomType == 1) {  //linestring
          resultgeom = DouglasPeuckerSimplifier.simplify(line, maxDisp);
        } else if (this.geomType == 2) { //polygon
          //poly = (Polygon)geom.clone();
          resultgeom = TopologyPreservingSimplifier.simplify(poly, maxDisp);
        }
        String mytext = "item: " + count + " / " + noItems + " : simplification finalized";
        //context.getWorkbenchFrame().setStatusMessage(mytext);
        monitor.report(mytext);
        //-- commit changes to undo history
        transaction.setGeometry(f, resultgeom);
      }//end if : polygon or linestring
    } //end for loop over selected objects
    transaction.commit();
    return true;
  }

}
