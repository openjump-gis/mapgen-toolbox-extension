package mapgen.algorithms.polygons;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import mapgen.algorithms.jts17qtree.Quadtree;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureCollection;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;
import com.vividsolutions.jump.task.TaskMonitor;
import com.vividsolutions.jump.workbench.plugin.PlugInContext;

/**
 * description: merges a set of polyons. The merge is done with
 * the PolygonMerge class based on JTS-union#.
 *
 * created on 		13.03.2006
 * @author sstein
 */
public class PolygonSetMerger {

	/**
	 * Does a union of all touching or intersection geometries.
	 * In geomsQtree and in polys are only geometries of type Polygon allowed.<p>
	 * the (jump)monitor can can be null. 
	 * @param polys list of polygons
	 * @param geomsQtree index
	 * @param monitor monitor
	 * @return merged polygon geometries
	 */
	public static List<Polygon> mergeGeoms(List<Polygon> polys,
									Quadtree geomsQtree, TaskMonitor monitor) {
		
		int totalsize = polys.size();
		List<Polygon> singlePolys = new ArrayList<>();
		while (!polys.isEmpty()) {
			if (monitor != null){
				monitor.report(polys.size() + " / " + totalsize + " items left");
			}
			List<Polygon> tempList = new ArrayList<>();
			int count = 0;
			for (Polygon g : polys) {
				//-- remove from tree
				boolean foundInTree = geomsQtree.remove(
						g.getEnvelopeInternal(), g);
				if (!foundInTree) {
					// item is not in tree anymore, so it has been merged already
					// so dont do anything; the item will not be on the new list
				} else {
					//-- get list of candidates
					List candidates = geomsQtree.query(g.getEnvelopeInternal());
					boolean oneFound = false;
					//-- check every candidate
					for (int i = 0; i < candidates.size(); i++) {
						Geometry cand = (Polygon) candidates.get(i);
						PolygonMerge merge = new PolygonMerge(g, cand);
						if (merge.isMergeSuccesfull() == 1) {
							oneFound = true;
							geomsQtree.remove(cand.getEnvelopeInternal(), cand); // remove from tree
							g = (Polygon) merge.getOutPolygon();
						}
						if (monitor != null){
							String msg = "candidate: " + i;
							monitor.report((polys.size() - count) + " / " + 
									totalsize + " items left. -- " + msg);
						}
					}
					if (!oneFound) {
						//-- this is a single poly
						singlePolys.add(g);
					} else {//since the object may have further neighbours add
						// it again to the list and the tree
						geomsQtree.insert(g.getEnvelopeInternal(), g);
						tempList.add(g);
					}
				}
				count = count + 1;
				if (monitor != null){
					monitor.report((polys.size() - count) + " / " + totalsize + " items left");
				}
			}
			polys = tempList;
		}
		return singlePolys;
	}
	
	/**
	 * merges touching or overlapping polygons (only polygons) with similar attribute value.
	 * 
	 * @param features features to process
	 * @param attrName Name of the Attribute
	 * @param context can be null
	 * @param monitor can be null
	 * @return
	 */
	public static FeatureCollection mergePolySetByType(Collection<Feature> features, String attrName,
			PlugInContext context,	TaskMonitor monitor){
		//-- if item selection is used, then items can have different Schemas!
		//	 we take the schema from the first item
		Iterator it = features.iterator();
		Feature firstF = (Feature)it.next();
		FeatureSchema fs = firstF.getSchema();
		
		//-----------------------------------
		//   check how many values for attribute do exists 
		//   and sort geoms in lists  
		//-----------------------------------
		List<Object> attrValues = new ArrayList<>(); //stores the attr values
		List<List<Polygon>> geomCollList = new ArrayList<>();
		List<Polygon> tempList;
		Object val;
		int idx=0; int index;
		for (Feature f : features) {
			Geometry geom = f.getGeometry();
			if (!(geom instanceof Polygon)) continue;
			Polygon poly = (Polygon)geom;
			val = f.getAttribute(attrName);
			if (idx == 0){ //first time add value directly to valuelist
				attrValues.add(val);
				List<Polygon> newtype = new ArrayList<>();	//create new Arrayist
				newtype.add(poly);      	//store item in new list
				geomCollList.add(newtype);//add List to TypeList (at the end)
			}
			else{//compare if already exist in list 
				index=0; boolean found = false;
				// TODO: [sstein] eventually optimize by finish loop if value has been found
				//       but should play not such a big role, since their should not be so much 
				//		 attribute values for merging polygons 
				for (Object storedValue : attrValues) {
					if (val.equals(storedValue)){
						found = true;
						tempList = geomCollList.get(index); //get geom-list for the value
						tempList.add(poly); //add item
					}
					index = index+1;
				}
				if (!found){
					//-- add value to list if not in list
					attrValues.add(val);
					List<Polygon> newtype = new ArrayList<>();	//create new Arrayist
					newtype.add(poly);      //store item in new list
					geomCollList.add(newtype);        //add List to TypeList (at the end)
				}
			}			
			idx=idx+1;
		}
		int noOfValues = geomCollList.size();
		//-----------------------------------
		// merge the geoms 
		//-----------------------------------
		List<List<Polygon>> resultGeomList = new ArrayList<>();
		idx = 0;
		for (List<Polygon> geomsOfOneValue : geomCollList) {
			idx = idx + 1;
			if (monitor != null){
				monitor.report("processing set " + idx + " / " + noOfValues);
			}
			// put all geoms in a tree for faster search
			// and check if polygon			
			Quadtree qtree = new Quadtree(); 
			for (Geometry element : geomsOfOneValue) {
				if (element instanceof Polygon){
					Polygon poly = (Polygon)element;
					qtree.insert(poly.getEnvelopeInternal(), poly);
				}
				else{
					if (context != null){
						context.getWorkbenchFrame().warnUser("no polygon");
					}
				}
			}
			List<Polygon> resultGeoms = PolygonSetMerger.mergeGeoms(geomsOfOneValue, qtree, monitor);
			resultGeomList.add(resultGeoms);
		}				
		//-----------------------------------		
		// generate featuredataset for output
		// create a new featureSchema
		//-----------------------------------
		FeatureSchema newFs = new FeatureSchema();		
		newFs.addAttribute("Geometry", AttributeType.GEOMETRY);
		newFs.addAttribute(attrName,fs.getAttributeType(attrName));
		FeatureDataset resFeatures = new FeatureDataset(newFs);
		//-- create features
		idx=0;
		for (List<Polygon> typegeoms : resultGeomList) {
			for (Geometry geom : typegeoms) {
				Feature f = new BasicFeature(newFs);
				f.setGeometry(geom.copy()); // set geometry
				f.setAttribute(attrName,attrValues.get(idx)); // set attribute value
				resFeatures.add(f);
			}
			idx = idx + 1;
		}	
		
		return resFeatures;
	}

}