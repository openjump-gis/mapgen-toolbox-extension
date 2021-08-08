package mapgen.ui.onselecteditems;

import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.plugin.AbstractPlugIn;
import com.vividsolutions.jump.workbench.plugin.EnableCheckFactory;
import com.vividsolutions.jump.workbench.plugin.MultiEnableCheck;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;
import com.vividsolutions.jump.workbench.plugin.ThreadedPlugIn;
import com.vividsolutions.jump.workbench.ui.GUIUtil;
import com.vividsolutions.jump.workbench.ui.MenuNames;
import com.vividsolutions.jump.workbench.ui.MultiInputDialog;
import com.vividsolutions.jump.workbench.ui.zoom.*;

import com.vividsolutions.jump.workbench.model.Layer;
import com.vividsolutions.jump.workbench.model.StandardCategoryNames;

import java.util.Collection;

import mapgen.algorithms.snakes.LineDisplacementSnakes;

/**
 * displaces lines from lines (and ?) to ensure cartographic legibility,
 * that means ensures visual separability of lines. <p>
 * The algorithm:<p>
 * - splits long lines with a huge number of vertices to avoid big matrices.<p>
 * - allows only a maximum number of lines to proceed to avoid a to big network matric.<p>
 * - might be useful for generalization of contour lines in
 * combination with simplification and smoothing algorithms. <p>
 * Ideally, to avoid line crossings: <p>
 * 1. simplify and displace at the same time <p>
 * 2. smooth and displace at the same time <p>
 * The displacement algorithm is based on the "Snakes technique" <p>
 * Literature: <p>
 * See Steiniger and Meier (2004): Snakes a technique for line displacement and smoothing.
 * See also Burghardt and Meier (1997), Burghardt (2001) or Bader (2001) (e.g. use http://schoolar.google.com)
 * =================
 * TODO: implement external displacement forces resulting from polygons
 * => see #evaluateLineWithDataset()
 * TODO: polygon features or lines can be of different feature type = different Schema!
 * => check that, or work only with geometries for nonLineFeatures!!!
 * <p>
 * created:  		04.07.2005
 * last modified:  	21.08.2005
 *
 * @author sstein
 */
public class DisplaceLinesPlugIn extends AbstractPlugIn implements ThreadedPlugIn {

  private final ZoomToSelectedItemsPlugIn myZoom = new ZoomToSelectedItemsPlugIn();
  private final String DISPID = "displaceID";
  private static final String T1 = "MapScale";
  private static final String T3 = "Line Signature Diameter [mm]:";
  private static final String T2 = "If YES specify maximum No of iterations?";
  private static final String T5 = "Do solve iterative?";
  //-- for tests
  private static final String T6 = "min sepparation distance in m ?";
  double signaturRadiusTestInMeter = 0;
  //-------
  //private final String newAttributString = "DisplaceLines";
  //private final String newSplitLineAttributeString = "SplitIdLine";
  //private final String newSplitSegmentAttributeString = "SplitIdLineSegm";
  double signatureRadiusA = 0;
  int scale = 1;
  int iterMax = 20;
  //boolean solveIter = false;
  //int maxPoints = 100; //defines the maximal number of vertices for a line to process the line
  //other wise matrices become to big (memory and speed problem can occure)
  //int minPoints = 6; //necessary to calculate snakes matrix
  //double crucialDistance = 0.2; //this values is needed to evaluate Networkstate
  //if distance of points is below that, the points and subsequently the lines
  //are connected
  //double maxNodeMoveDistance = 10.0; //this should be reset according to visual perceptible maxDisplacement
  //double maxObjects = 1000; //to avoid to big matrix calculated for network state evaluation

  //--parameters for snakes
  //double alpha=1;
  //double beta =1;


  public void initialize(PlugInContext context) throws Exception {
    context.getFeatureInstaller().addMainMenuPlugin(this,
        new String[]{MenuNames.PLUGINS, MenuNames.GENERALIZATION, "Scale Dependent Algorithms", "Lines"},  //menu path
        "Displace Selected Lines",
        false,null,
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

    this.setDialogValues(dialog, context);
    //this.setDialogValuesNew(dialog, context);

    GUIUtil.centreOnWindow(dialog);
    dialog.setVisible(true);
    if (!dialog.wasOKPressed()) {
      return false;
    }

    this.getDialogValues(dialog);
    //this.getDialogValuesNew(dialog);

    return true;
  }

  private void setDialogValuesNew(MultiInputDialog dialog, PlugInContext context) {
    dialog.setSideBarDescription(
        "Displace lines: ");
    dialog.addDoubleField(T6, 80.0, 4);
    //dialog.addCheckBox(T5,true);
    dialog.addIntegerField(T2, 2, 4, T2);
  }


  private void setDialogValues(MultiInputDialog dialog, PlugInContext context) {
    dialog.setSideBarDescription(
        "Displace lines: map scale is used to detect narrow parts");
    dialog.addIntegerField(T1, 25000, 7, T1);
    dialog.addDoubleField(T3, 1.0, 4);
    //dialog.addCheckBox(T5,true);
    dialog.addIntegerField(T2, 2, 4, T2);
  }

  private void getDialogValues(MultiInputDialog dialog) {
    this.scale = dialog.getInteger(T1);
    //this.solveIter = dialog.getBoolean(T5);
    this.iterMax = dialog.getInteger(T2);
    this.signatureRadiusA = dialog.getDouble(T3) / 2.0; //it receives the diameter
  }

  private void getDialogValuesNew(MultiInputDialog dialog) {
    //this.solveIter = dialog.getBoolean(T5);
    this.iterMax = dialog.getInteger(T2);
    this.signaturRadiusTestInMeter = dialog.getDouble(T6) / 2.0; //divide => diameter=> radius
  }

  public void run(TaskMonitor monitor, PlugInContext context) throws Exception {

    final Collection<Feature> features =
        context.getLayerViewPanel().getSelectionManager().getFeaturesWithSelectedItems();

    monitor.report("start calculations: see console for status report");
    //LineDisplacementSnakes dispSnakes = new LineDisplacementSnakes(features,this.iterMax,this.signaturRadiusTestInMeter*2.0);
/*
	    	LineDisplacementSnakes dispSnakes = new LineDisplacementSnakes(features,this.iterMax, 
	    			this.signatureRadiusA, this.scale, monitor);
*/
    LineDisplacementSnakes dispSnakes = new LineDisplacementSnakes(features, this.iterMax,
        this.signatureRadiusA, this.scale);

    FeatureCollection lines = dispSnakes.getDisplacedLines();
    if (lines.size() > 0) {
      context.addLayer(StandardCategoryNames.WORKING, "displaced lines", lines);
    }
	    	/*
	    	FeatureCollection nrgPoints = dispSnakes.getInitialPointEnergies();
	    	if(nrgPoints.size() > 0){ 
	    	    context.addLayer(StandardCategoryNames.WORKING, "initial energy vertices", nrgPoints);	    	    
	    	}
	    	
	    	FeatureCollection netNodes = dispSnakes.getNetworkNodesAndSplitPoints();
	    	if(netNodes.size() > 0){ 
	    	    context.addLayer(StandardCategoryNames.WORKING, "networkNodes and Splits", netNodes);	    	    
	    	}
	    	*/
    FeatureCollection buffers = dispSnakes.getMinDistAndSignatureBuffers();
    if (buffers.size() > 0) {
      context.addLayer(StandardCategoryNames.WORKING, "mindist + signature buffers", buffers);
    }

  }

  /**
   * Centers the selected feature
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

}
