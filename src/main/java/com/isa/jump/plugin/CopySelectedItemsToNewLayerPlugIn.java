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
import java.util.Collection;
import javax.swing.JPopupMenu;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.workbench.WorkbenchContext;
import com.vividsolutions.jump.workbench.model.Category;
import com.vividsolutions.jump.workbench.model.StandardCategoryNames;
import com.vividsolutions.jump.workbench.plugin.AbstractPlugIn;
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.ui.LayerViewPanel;
import com.vividsolutions.jump.workbench.ui.plugin.FeatureInstaller;
import com.vividsolutions.jump.workbench.ui.plugin.clipboard.CopySelectedItemsPlugIn;
import com.vividsolutions.jump.workbench.model.Layer;
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import java.awt.Toolkit;
import com.vividsolutions.jump.workbench.ui.plugin.clipboard.CollectionOfFeaturesTransferable;

public class CopySelectedItemsToNewLayerPlugIn extends AbstractPlugIn {

	private Layer newLayer = null;
	
    public void initialize(PlugInContext context) throws Exception
    {     
        WorkbenchContext workbenchContext = context.getWorkbenchContext();
        FeatureInstaller featureInstaller = new FeatureInstaller(workbenchContext);
        JPopupMenu popupMenu = LayerViewPanel.popupMenu();
        featureInstaller.addPopupMenuPlugin(popupMenu,
            this, "Copy Selected Items To New Layer",
            false, null,  //to do: add icon
            this.createEnableCheck(workbenchContext)); 
    }
    
    public boolean execute(final PlugInContext context) throws Exception
    {
        reportNothingToUndoYet(context);
        new CopySelectedItemsPlugIn().execute(context);
        
        FeatureSchema featureSchema;
        Collection<Layer> selectedLayers =
            context.getLayerViewPanel().getSelectionManager().getLayersWithSelectedItems();
        
        if (selectedLayers.size() > 1)
        {
        	featureSchema = new FeatureSchema();
        	featureSchema.addAttribute("GEOMETRY", AttributeType.GEOMETRY);
        }
        else
        {
        	Layer sourceLayer = selectedLayers.iterator().next();
        	featureSchema = sourceLayer.getFeatureCollectionWrapper().getFeatureSchema();
        }
        
        FeatureDataset featureDataset = new FeatureDataset(featureSchema);
        
        Collection<Category> selectedCategories = context.getLayerNamePanel().getSelectedCategories();
        newLayer = context.addLayer(selectedCategories.isEmpty()
        ? StandardCategoryNames.WORKING
        : selectedCategories.iterator().next().toString(), "New",
        featureDataset);
        
        newLayer.setFeatureCollectionModified(true).setEditable(true);
                
        Collection<Feature> features = (Collection<Feature>) GUIUtil
            .getContents(Toolkit.getDefaultToolkit().getSystemClipboard())
            .getTransferData(CollectionOfFeaturesTransferable.COLLECTION_OF_FEATURES_FLAVOR);
        newLayer.getFeatureCollectionWrapper().addAll(features);
        return true;
    }
    
    public Layer getNewLayer() {
    	return newLayer;
    }
    public MultiEnableCheck createEnableCheck(final WorkbenchContext workbenchContext) {
        EnableCheckFactory checkFactory = new EnableCheckFactory(workbenchContext);
        return new MultiEnableCheck()
            .add(checkFactory.createWindowWithLayerViewPanelMustBeActiveCheck())
            .add(checkFactory.createAtLeastNItemsMustBeSelectedCheck(1));
    }
}
