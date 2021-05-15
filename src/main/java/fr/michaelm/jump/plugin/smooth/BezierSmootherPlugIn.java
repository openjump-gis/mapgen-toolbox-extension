package fr.michaelm.jump.plugin.smooth;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryCollection;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

import com.vividsolutions.jump.I18N;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.model.Layer;
import com.vividsolutions.jump.workbench.model.StandardCategoryNames;
import com.vividsolutions.jump.workbench.model.UndoableCommand;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.MultiInputDialog;
import com.vividsolutions.jump.workbench.ui.SelectionManager;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;


import mapgen.MapGenExtension;

import org.openjump.core.ui.plugin.AbstractThreadedUiPlugIn;

public class BezierSmootherPlugIn extends AbstractThreadedUiPlugIn {
    
    private final static String SMOOTH = MapGenExtension.SMOOTH;

    // Dialog (dataset)
    private final static String SMOOTH_LAYER               = I18N.getText(SMOOTH, "smooth-layer");
    private final static String SMOOTH_SELECTION           = I18N.getText(SMOOTH, "smooth-selection");
    private final static String LAYER                      = I18N.getText(SMOOTH, "layer");
    private final static String CREATE_NEW_LAYER           = I18N.getText(SMOOTH, "create-new-layer");
    // Dialog (smoothing options)
    private final static String MIN_SEGMENT_LENGTH         = I18N.getText(SMOOTH, "min-segment-length");
    private final static String MIN_SEGMENT_LENGTH_TOOLTIP = I18N.getText(SMOOTH, "min-segment-length-tooltip");
    private final static String NUMBER_OF_POINTS           = I18N.getText(SMOOTH, "number-of-points"); 
    private final static String NUMBER_OF_POINTS_TOOLTIP   = I18N.getText(SMOOTH, "number-of-points-tooltip");
    private final static String SMOOTH_FACTOR              = I18N.getText(SMOOTH, "smooth-factor");
    private final static String SMOOTH_FACTOR_TOOLTIP      = I18N.getText(SMOOTH, "smooth-factor-tooltip");
    // Dialog (left panel description
    private final static String PLUGIN_DESCRIPTION         = I18N.getText(SMOOTH, "plugin-description");
    
    // Monitor
    private final static String SMOOTHING                  = I18N.getText(SMOOTH, "smoothing");
    
    // Results
    private final static String SMOOTHED_SELECTION         = I18N.getText(SMOOTH, "smoothed-selection");
    private final static String SMOOTHED_SUFFIX            = I18N.getText(SMOOTH, "smoothed-suffix");
    
    
    private String layerName;
    private boolean newLayer = true;
    private boolean selection;
    private double minLength = 0.0;
    private int numberOfPoints = 5;
    private double smoothFactor = 0.5;
    
    LineSmoother lineSmoother;
    PolygonSmoother polygonSmoother;
    SmootherControl control;

    public BezierSmootherPlugIn() { }

    /**
     * Returns a very brief description of this task.
     * @return the name of this task
     */
    public String getName() {
        return I18N.getText(SMOOTH, "BezierSmootherPlugIn");
    }

    public void initialize(PlugInContext context) throws Exception {
        context.getFeatureInstaller().addMainMenuPlugin(this,
            new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION,"Not Scale Dependent Algorithms", "Lines"},
            getName(), false, null,
            new MultiEnableCheck()
                .add(context.getCheckFactory().createTaskWindowMustBeActiveCheck())
                .add(context.getCheckFactory().createAtLeastNLayersMustExistCheck(1)));
        
        context.getFeatureInstaller().addMainMenuPlugin(this,
                new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION,"Not Scale Dependent Algorithms", "Polygons"},
                getName(), false, null,
                new MultiEnableCheck()
                    .add(context.getCheckFactory().createTaskWindowMustBeActiveCheck())
                    .add(context.getCheckFactory().createAtLeastNLayersMustExistCheck(1)));
        
    }

    public boolean execute(PlugInContext context) throws Exception {
        MultiInputDialog dialog = new MultiInputDialog(
            context.getWorkbenchFrame(),
            getName(),
            true);
        SelectionManager selectionManager = context.getLayerViewPanel().getSelectionManager();
        if (selectionManager.getLayersWithSelectedItems().size() > 0) {
            selection = true;
        }
        else {
            selection = false;
        }
        setDialogValues(dialog, context);
        GUIUtil.centreOnWindow(dialog);
        dialog.setVisible(true);
        if (!dialog.wasOKPressed()) { return false; }
        getDialogValues(dialog);
        return true;
    }

    public void run(TaskMonitor monitor, PlugInContext context) throws Exception {
        monitor.allowCancellationRequests();
        monitor.report(I18N.getText(SMOOTH, SMOOTHING + "..."));
        Collection<Feature> features = new ArrayList<>();
        FeatureSchema schema = new FeatureSchema();
        if (selection) {
            features.addAll(context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems());
            schema.addAttribute("GEOMETRY",AttributeType.GEOMETRY);
        }
        else {
            Layer layer = context.getLayerManager().getLayer(layerName);
            features.addAll(layer.getFeatureCollectionWrapper().getFeatures());
            schema = layer.getFeatureCollectionWrapper().getFeatureSchema();
        }
        // Parametrization of smoothers
        GeometryFactory factory = features.iterator().next().getGeometry().getFactory();
        lineSmoother = new LineSmoother(factory);
        polygonSmoother = new PolygonSmoother(factory);
        setControl();
        
        // Update case, with undoable commit management 
        if (selection && !newLayer) {
            Collection<Layer> layers = context.getLayerViewPanel().getSelectionManager().getLayersWithSelectedItems();
            for (Layer layer : layers) {
                Collection<Feature> inputFeatures = context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems(layer);
                List<Feature> outputFeatures = new ArrayList<>();
                for (Feature feature : inputFeatures) {
                    Feature newFeature = feature.clone(true);
                    newFeature.setGeometry(smooth(feature.getGeometry(), smoothFactor, new ArrayList<>()));
                    outputFeatures.add(newFeature);
                    if (monitor.isCancelRequested()) return;
                }
                commitUpdate(context, layer, inputFeatures, outputFeatures);
            }
        }
        // Copy to new layer
        else {
            FeatureDataset dataset = new FeatureDataset(schema);
            for (Feature feature : features) {
                Feature newFeature = selection ? 
                        new BasicFeature(schema) : feature.clone(true);
                newFeature.setGeometry(smooth(feature.getGeometry(), smoothFactor, new ArrayList<>()));
                dataset.add(newFeature);
                if (monitor.isCancelRequested()) return;
            }
            layerName = selection ? 
                        I18N.getText(SMOOTH, "smoothed-selection") : 
                        layerName + I18N.getText(SMOOTH, "smoothed-suffix");
            context.addLayer(StandardCategoryNames.RESULT_SUBJECT, layerName, dataset);
        }
    }
    
    private void commitUpdate(PlugInContext context, final Layer layer,
                 final Collection<Feature> inputFeatures, final Collection<Feature> newFeatures) {
        context.getLayerManager().getUndoableEditReceiver().reportNothingToUndoYet();
        UndoableCommand cmd = new UndoableCommand(getName()) {
            public void execute() {
                layer.getFeatureCollectionWrapper().removeAll(inputFeatures);
                layer.getFeatureCollectionWrapper().addAll(newFeatures);
            }
            
            public void unexecute() {
                layer.getFeatureCollectionWrapper().removeAll(newFeatures);
                layer.getFeatureCollectionWrapper().addAll(inputFeatures);
            }
        };
        boolean exceptionOccurred = true;
		try {
		    cmd.execute();
		    exceptionOccurred = false;
		} 
		finally {
		    if (exceptionOccurred) {
		        context.getLayerManager().getUndoableEditReceiver()
                       .getUndoManager().discardAllEdits();
            }
		}
		context.getLayerManager().getUndoableEditReceiver().receive(cmd.toUndoableEdit());
    }
    
    public Geometry smooth(Geometry geometry, double alpha, List<Geometry> geoms) {
        int numberOfGeometries = geometry.getNumGeometries();
        for (int i = 0 ; i < numberOfGeometries ; i++) {
            Geometry g = geometry.getGeometryN(i);
            // case of nested collections
            if (g instanceof GeometryCollection) {
                smooth(g, alpha, geoms);
            }
            else if (g.getDimension()==0) geoms.add(g);
            else if (g.getDimension()==1) geoms.add(lineSmoother.smooth((LineString)g, alpha));
            else if (g.getDimension()==2) geoms.add(polygonSmoother.smooth((Polygon)g, alpha));
            else;
        }
        return geometry.getFactory().buildGeometry(geoms);
    }
    
    private void setControl() {
        control = new SmootherControl() {
            public double getMinLength() {
                return minLength;
            }
            public int getNumVertices(double length) {
                return Math.min(numberOfPoints, (int)(length/minLength));
            }
        };
        if (lineSmoother != null) lineSmoother.setControl(control);
        if (polygonSmoother != null) polygonSmoother.setControl(control);
    }
    
    


    private void setDialogValues(MultiInputDialog dialog, PlugInContext context) {
      dialog.setSideBarDescription(PLUGIN_DESCRIPTION);
      if (selection) {
          dialog.addLabel(SMOOTH_SELECTION);
          dialog.addCheckBox(CREATE_NEW_LAYER, newLayer);
      }
      else {
          dialog.addLabel(SMOOTH_LAYER);
          Layer layer = context.getLayerManager().getLayer(layerName);
          if (layer == null) {
              layer = context.getCandidateLayer(0);
          }
          dialog.addLayerComboBox(LAYER, layer, null, context.getLayerManager());
      }
      dialog.addSeparator();
      dialog.addDoubleField(MIN_SEGMENT_LENGTH, minLength, 8, MIN_SEGMENT_LENGTH_TOOLTIP);
      dialog.addIntegerField(NUMBER_OF_POINTS, numberOfPoints, 4, NUMBER_OF_POINTS_TOOLTIP);
      dialog.addDoubleField(SMOOTH_FACTOR, smoothFactor, 8, SMOOTH_FACTOR_TOOLTIP);
    }

    private void getDialogValues(MultiInputDialog dialog) {
        if (selection) {
            newLayer = dialog.getBoolean(CREATE_NEW_LAYER);
        }
        else {
            layerName = dialog.getLayer(LAYER).getName();
        }
        minLength = dialog.getDouble(MIN_SEGMENT_LENGTH);
        numberOfPoints = dialog.getInteger(NUMBER_OF_POINTS);
        smoothFactor = dialog.getDouble(SMOOTH_FACTOR);
    }
}
