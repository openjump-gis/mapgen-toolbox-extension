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

import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import javax.swing.DefaultComboBoxModel;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JRadioButton;

import mapgen.algorithms.jts17qtree.Quadtree;
import mapgen.algorithms.polygons.PolygonMerge;
import mapgen.algorithms.polygons.PolygonSetMerger;
import mapgen.ui.MultiInputDialog;

import org.locationtech.jts.geom.Geometry;
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
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.plugin.ThreadedBasePlugIn;
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.plugin.FeatureInstaller;


/**
 * Merges selected polygons by different ways
 * <p>
 * created:  		07.09.2005
 * last modified: 04.10.2005 append attributes
 * 12.03.2005 merge by attributes
 *
 * @author sstein
 **/
public class MergeSelectedPolygonsPlugIn extends ThreadedBasePlugIn {

  private final static String MERGE2POLYS = "merge 2 polygons with attributes";
  private final static String MERGEGEOMS = "merge all touching polygons";
  private final static String MERGEBYTYPE = "merge all touching polygons if attribute value is similar";
  private boolean merge2polysB = false;
  private boolean mergeGeomsB = false;
  private boolean mergeByTypeB = false;
  private boolean workOnLayer = false;
  private final static String WORK_ON_LAYER = "use all layer items";
  private Layer srcLayer = null;
  private final static String SRC_LAYER = "source layer";
  private String attrName = "";
  private final static String SRC_ATTRIB = "select attribute";

  private MultiInputDialog dialog;
  private MultiInputDialog selectTypeDialog;
  //private PlugInContext pc = null;

  public void initialize(PlugInContext context) throws Exception {
    FeatureInstaller featureInstaller = new FeatureInstaller(context.getWorkbenchContext());
    featureInstaller.addMainMenuPlugin(
        this,                //exe
        new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Not Scale Dependent Algorithms", "Polygons"},  //menu path
        "Merge Selected Polyons", //name methode .getName recieved by AbstractPlugIn
        false,      //checkbox
        null,      //icon
        createEnableCheck(context.getWorkbenchContext())); //enable check
  }

  public static MultiEnableCheck createEnableCheck(WorkbenchContext workbenchContext) {
    EnableCheckFactory checkFactory = new EnableCheckFactory(workbenchContext);

    return new MultiEnableCheck()
        .add(checkFactory.createWindowWithLayerNamePanelMustBeActiveCheck())
        .add(checkFactory.createAtLeastNLayersMustExistCheck(1))
        /*.add(checkFactory.createAtLeastNItemsMustBeSelectedCheck(2)*)*/;
  }

  /*
   * do some dialog things first - processing is done in #run()
   */
  public boolean execute(PlugInContext context) throws Exception {
    dialog = new MultiInputDialog(
        context.getWorkbenchFrame(), getName(), true);
    setDialogValues(dialog, context);
    GUIUtil.centreOnWindow(dialog);
    dialog.setVisible(true);
    if (!dialog.wasOKPressed()) {
      return false;
    }
    getDialogValues(dialog);
    //-- open second dialog if merge should be done by similar type
    if (this.mergeByTypeB) {
      this.selectTypeDialog = new MultiInputDialog(
          context.getWorkbenchFrame(), getName(), true);
      setTypeDialogValues(selectTypeDialog, context);
      GUIUtil.centreOnWindow(selectTypeDialog);
      selectTypeDialog.setVisible(true);
      if (!dialog.wasOKPressed()) {
        return false;
      }
      getTypeDialogValues(selectTypeDialog);
    }
    return true;
  }

  public void run(TaskMonitor monitor, PlugInContext context) throws Exception {

    monitor.allowCancellationRequests();
    //this.pc = context;

    Collection<Feature> features = context.getWorkbenchContext().getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();
    //************************
    // Two features should be merged
    //***********************/
    if (this.merge2polysB) {
      if (features.size() == 2) {
        Iterator<Feature> iter = features.iterator();
        Feature f1 = iter.next();
        Feature f2 = iter.next();
        PolygonMerge merge = new PolygonMerge(f1.getGeometry(), f2.getGeometry());
        if (merge.isMergeSuccesfull() == 1) {
          Geometry g = merge.getOutPolygon();
          Feature newF = this.appendAttributesFromSecondFeature(f1, f2);
          newF.setGeometry(g);
          FeatureDataset myCollA = new FeatureDataset(newF.getSchema());
          myCollA.add(newF);
          context.addLayer(StandardCategoryNames.WORKING, "mergedPolygonFeature", myCollA);
        } else if (merge.isMergeSuccesfull() == 2) {
          context.getWorkbenchFrame().warnUser("Multipolygon created");
          Geometry g = merge.getOutPolygon();
          Feature newF = this.appendAttributesFromSecondFeature(f1, f2);
          newF.setGeometry(g);
          FeatureDataset myCollA = new FeatureDataset(newF.getSchema());
          myCollA.add(newF);
          context.addLayer(StandardCategoryNames.WORKING, "MultiPolygonFeature", myCollA);
        } else if (merge.isMergeSuccesfull() == 0) {
          context.getWorkbenchFrame().warnUser("polygons don't touch");
        }
      } else {
        context.getWorkbenchFrame().warnUser("more than 2 objects selected");
      }
    }
    //************************
    // set of poly geometries should be merged
    //***********************/
    else if (this.mergeGeomsB) {
      if (this.workOnLayer) {
        features = this.srcLayer.getFeatureCollectionWrapper().getFeatures();
      }
      //put all geoms in a tree for faster search
      List<Polygon> resultGeoms;
      List<Polygon> geoms = new ArrayList<>();
      Quadtree qtree = new Quadtree();
      for (Feature element : features) {
        if (element.getGeometry() instanceof Polygon) {
          Polygon poly = (Polygon) element.getGeometry();
          geoms.add(poly);
          qtree.insert(poly.getEnvelopeInternal(), poly);
        } else {
          context.getWorkbenchFrame().warnUser("no polygon");
        }
      }
      if (geoms.size() > 0) {
        resultGeoms = PolygonSetMerger.mergeGeoms(geoms, qtree, monitor);
        FeatureCollection myCollA = FeatureDatasetFactory.createFromGeometry(resultGeoms);
        if (myCollA.size() > 0) {
          context.addLayer(StandardCategoryNames.WORKING, "mergedPolygons", myCollA);
        }
      }
    }
    /*************************
     * Features should be merged according to similar attribute value
     ************************/
    else if (this.mergeByTypeB) {
      features = this.srcLayer.getFeatureCollectionWrapper().getFeatures();
      FeatureCollection myCollA = PolygonSetMerger.mergePolySetByType(features, this.attrName, context, monitor);
      context.addLayer(StandardCategoryNames.WORKING, "mergedPolygons", myCollA);
    } else {
      context.getWorkbenchFrame().warnUser("not implemented");
    }
    //context.getWorkbenchContext().getLayerViewPanel().getSelectionManager().clear();
  }


  //============================================================
  // Attribute Schema methods
  //============================================================


  /**
   * append attributes of given second feature to the first feature
   *
   * @param f1 first feature
   * @param f2 second feature
   * @return
   */
  private Feature appendAttributesFromSecondFeature(Feature f1, Feature f2) {
    //-- create newSchema
    FeatureSchema fs = this.mergeFSchemas(f1.getSchema(), f2.getSchema());
    //-- create a new Feature which has already the attributes of the
    //   the first feature
    Feature newF = this.copyFeature(f1, fs);
    //-- append the other attribute Values from Feature 2
    //   but without a second Geometry
    FeatureSchema secondSchema = f2.getSchema();
    int indx = f1.getSchema().getAttributeCount();
    for (int i = 0; i < secondSchema.getAttributeCount(); i++) {
      AttributeType at = secondSchema.getAttributeType(i);
      Object value = f2.getAttribute(i);
      //-- don't store a second geometry
      if (at != AttributeType.GEOMETRY) {
        newF.setAttribute(indx, value);
        indx++;
      }
    }
    return newF;
  }

  /**
   * Append a second FeatureSchema to another.
   * The second geometry attribute will be omitted.
   *
   * @param firstSchema first schema
   * @param secondSchema second schema
   * @return a schema including attributes from first and second schema
   */
  private FeatureSchema mergeFSchemas(FeatureSchema firstSchema, FeatureSchema secondSchema) {
    FeatureSchema newSchema = this.copyFeatureSchema(firstSchema);
    for (int i = 0; i < secondSchema.getAttributeCount(); i++) {
      AttributeType at = secondSchema.getAttributeType(i);
      String aname = secondSchema.getAttributeName(i);
      if (newSchema.hasAttribute(aname)) {
        aname = aname + "N";
      }
      //-- don't store a second geometry
      if (at != AttributeType.GEOMETRY) {
        newSchema.addAttribute(aname, at);
      }
    }
    return newSchema;
  }

  /**
   * copy the input feature to a new Schema whereby the new
   * Feature Schema musst be an extended or shortened one
   *
   * @param feature the Feature to Copy
   * @param newSchema the schema to copy the feature to
   * @return a new Feature based on newSchema
   */
  private Feature copyFeature(Feature feature, FeatureSchema newSchema) {
    FeatureSchema oldSchema = feature.getSchema();
    Feature newF = new BasicFeature(newSchema);
    int n;
    if (oldSchema.getAttributeCount() > newSchema.getAttributeCount()) {
      //for schema shortening
      n = newSchema.getAttributeCount();
    } else {
      //for schema extension
      n = oldSchema.getAttributeCount();
    }
    for (int i = 0; i < n; i++) {
      String aname = oldSchema.getAttributeName(i);
      Object value = feature.getAttribute(aname);
      newF.setAttribute(aname, value);
    }
    return newF;
  }

  /**
   * copy/clone the input featureSchema and  since it is not proper implemented in Jump
   *
   * @param oldSchema
   * @return a new schema with the same attributes
   */
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

  //============================================================
  // dialog things
  //============================================================

  JComboBox<Layer> layerboxB;
  private JComboBox<Layer> layerbox;
  private JCheckBox checkbox;
  private JComboBox<String> attribbox;

  private void setDialogValues(MultiInputDialog dialog, PlugInContext context) {

    JRadioButton merge2polysRB;
    JRadioButton mergeGeomsRB;
    JRadioButton mergeGeomsByTypeRB;


    //dialog.setSideBarImage(new ImageIcon(getClass().getResource("")));
    dialog.setSideBarDescription("Choose a method to merge polygon features!" +
        " Be aware of either using selected item or items of a layer." +
        " Merging 2 polygons with attributes works only for 2 selected items.");

    final String OUTPUT_GROUP = "action";

    merge2polysRB = dialog.addRadioButton(MERGE2POLYS, OUTPUT_GROUP, false, "Merge 2 selected polygons while preserving attributes.");
    if (context.getWorkbenchContext().getLayerViewPanel().getSelectionManager().getSelectedItems().size() > 1) {
      merge2polysRB.setEnabled(true);
    } else {
      merge2polysRB.setEnabled(false);
    }
    mergeGeomsRB = dialog.addRadioButton(MERGEGEOMS, OUTPUT_GROUP, false, "Merge a set of overlaping or touching polygons either the selected or from a layer)");
    mergeGeomsRB.addItemListener(new MethodItemListener());
    mergeGeomsByTypeRB = dialog.addRadioButton(MERGEBYTYPE, OUTPUT_GROUP, false, "Merging a set of overlaping or touching polygons which must be of same type and in same layer");
    mergeGeomsByTypeRB.setEnabled(true);
    mergeGeomsByTypeRB.addItemListener(new MethodItemListener());
    checkbox = dialog.addCheckBox(WORK_ON_LAYER, this.workOnLayer);
    checkbox.addItemListener(new MethodItemListener());
    checkbox.setEnabled(false);
    if (srcLayer == null) srcLayer = context.getCandidateLayer(0);
    layerbox = dialog.addLayerComboBox(SRC_LAYER, srcLayer, "choose layer with polygons", context.getLayerManager());
    layerbox.setEnabled(false);
  }

  private final Object attrValue = null;
  private final ArrayList myColl = new ArrayList();

  /**
   * @param selectTypeDialog2
   * @param context
   */
  private void setTypeDialogValues(MultiInputDialog selectTypeDialog2, PlugInContext context) {

    this.selectTypeDialog.setSideBarDescription("Set Layer and attribute on which union of polygons should be based");

    if (srcLayer == null) srcLayer = context.getCandidateLayer(0);
    layerboxB = this.selectTypeDialog.addLayerComboBox(SRC_LAYER, srcLayer, "choose layer with polygons", context.getLayerManager());
    layerboxB.addItemListener(new MethodItemListenerB());
		
		/*
		attribbox= new JComboBox();
		this.selectTypeDialog.getContentPane().add(attribbox,
                	new GridBagConstraints(1, 0, 1, 1, 0.0, 0.0,
                    GridBagConstraints.WEST, GridBagConstraints.NONE,
                    new Insets(2, 2, 2, 2), 0, 0));*/
    attribbox = this.selectTypeDialog.addComboBox(SRC_ATTRIB, attrValue, myColl, "set attribute on which union should be based");
    updateUIForAttributes();

  }

  private void updateUIForMethod() {
    //-- disable checkbox if only two features should be merged
    if ((dialog.getBoolean(MERGEGEOMS)) ||
        (dialog.getBoolean(MERGEBYTYPE))) {
      checkbox.setEnabled(true);
      //-- make layer selector usable if all geoms should be merged
      if (!dialog.getBoolean(WORK_ON_LAYER)) {
        layerbox.setEnabled(false);
      } else {
        layerbox.setEnabled(true);
      }
    } else {
      checkbox.setEnabled(false);
    }
    //-- if merge by type is selected, LAYER must be set
    if (dialog.getBoolean(MERGEBYTYPE)) {
      checkbox.setEnabled(true);
      checkbox.setSelected(true);
    }
    dialog.validate();
  }

  private void updateUIForAttributes() {
    DefaultComboBoxModel<String> model = new DefaultComboBoxModel<>();
    for (int i = 0; i < srcLayer.getFeatureCollectionWrapper().getFeatureSchema().getAttributeCount(); i++) {
      if (i == srcLayer.getFeatureCollectionWrapper().getFeatureSchema().getGeometryIndex()) {
        continue;
      }
      model.addElement(srcLayer.getFeatureCollectionWrapper()
          .getFeatureSchema().getAttributeName(i));
    }
    attribbox.setModel(model);

    if (model.getSize() == 0) {
      //Can get here if the only attribute is the geometry. [Jon Aquino]
    }

    this.selectTypeDialog.validate();

  }

  private void getDialogValues(MultiInputDialog dialog) {
    merge2polysB = dialog.getBoolean(MERGE2POLYS);
    mergeGeomsB = dialog.getBoolean(MERGEGEOMS);
    mergeByTypeB = dialog.getBoolean(MERGEBYTYPE);
    srcLayer = dialog.getLayer(SRC_LAYER);
    workOnLayer = dialog.getBoolean(WORK_ON_LAYER);
  }

  private void getTypeDialogValues(MultiInputDialog dialog) {
    srcLayer = dialog.getLayer(SRC_LAYER);
    attrName = (String) attribbox.getSelectedItem();
  }

  //============================================================
  // dialog listeners
  //============================================================

  private class MethodItemListener implements ItemListener {

    public void itemStateChanged(ItemEvent e) {
      updateUIForMethod();
    }
  }

  private class MethodItemListenerB implements ItemListener {

    public void itemStateChanged(ItemEvent e) {
      updateUIForAttributes();
    }
  }

}