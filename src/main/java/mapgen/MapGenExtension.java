/*
 * Created on 10.01.2005
 *
 */
package mapgen;

import mapgen.ui.onselecteditems.ChangeElongationSelectedBuildingPlugIn;
import mapgen.ui.onselecteditems.DisplaceLinesPlugIn;
import mapgen.ui.onselecteditems.EliminatePointsInLineOfBuildingPlugIn;
import mapgen.ui.onselecteditems.EliminateSmallBuildingsPlugIn;
import mapgen.ui.onselecteditems.BuildingSpreadNarrowPartsPlugIn;
import mapgen.ui.onselecteditems.EnlargeBuildingToRectanglePlugIn;
import mapgen.ui.onselecteditems.LineSmoothSimpleVersionPlugIn;
import mapgen.ui.onselecteditems.MergeSelectedPolygonsPlugIn;
import mapgen.ui.onselecteditems.SimplifyBuildingToRectanglePlugIn;
import mapgen.ui.onselecteditems.SimplifyOutlineSelectedBuildingPlugIn;
import mapgen.ui.onselecteditems.SquareBuildingPlugIn;

import com.isa.jump.plugin.OrthogonalizePlugIn;
import com.vividsolutions.jump.workbench.plugin.Extension;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;

import fr.michaelm.jump.plugin.smooth.BezierSmootherPlugIn;

/**
 * @author sstein
 * @author mmichaud
 * @author mbedward
 * @author lbecker
 *   
 *  Loads several functions useful for map generalization 
 */
// History
// 2.1.0 (2021-08-08) : refactoring for changes in OJ i18n and FeatureInstaller
// 2.0 (2021-05-15) : migration to OpenJUMP 2.0
public class MapGenExtension extends Extension{
    
    public String getName() {
        return "Map Generalization Toolbox (Stefan Steiniger, Michaël Michaud, Michael Bedward, Larry Becker)";
    }

    public String getVersion() {
        return "2.0 (2021-05-15)";
    }
    
	/**
	 * calls PlugIn using class method xplugin.initialize() 
	 */
	public void configure(PlugInContext context) throws Exception{
		
		new BuildingSpreadNarrowPartsPlugIn().initialize(context);
		new EnlargeBuildingToRectanglePlugIn().initialize(context);
		new SquareBuildingPlugIn().initialize(context);
		new EliminateSmallBuildingsPlugIn().initialize(context);
		new EliminatePointsInLineOfBuildingPlugIn().initialize(context);
		new SimplifyOutlineSelectedBuildingPlugIn().initialize(context);
		new SimplifyBuildingToRectanglePlugIn().initialize(context);
		new ChangeElongationSelectedBuildingPlugIn().initialize(context);
		new DisplaceLinesPlugIn().initialize(context);
		new LineSmoothSimpleVersionPlugIn().initialize(context);
		//new LineSimplifyJTS15AlgorithmPlugIn().initialize(context);
		new MergeSelectedPolygonsPlugIn().initialize(context);
		new BezierSmootherPlugIn().initialize(context);
		new OrthogonalizePlugIn().initialize(context);
	}
	
}
