package mapgen.algorithms.snakes;

import java.util.ArrayList;
import java.util.List;

import mapgen.geomutilities.SecondGeodeticTask2d;

import org.jmat.MatlabSyntax;
import org.jmat.data.Matrix;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureSchema;

/**
 * Describes a line which is in the pool of possible lines which might be displaced.
 * <p>
 * Stores the different line states during displacement, the energy states and energy
 * access/calculation methods
 * <p>
 * created:  		04.07.2005
 * last modified:
 * 21.08.2005 change in energy calculation division
 * by noPoint values > zero instead n=all points
 * 26.8.2005  new method checkIfclosed()
 *
 * @author sstein
 */
public class LineToDisplace {
  private final Feature originalJumpFeature;
  private Geometry geometryTemp;
  private final List<Geometry> allGeometryStates = new ArrayList<>();
  private final List<Matrix> dispVectorDxMatrixList = new ArrayList<>();
  private final List<Matrix> dispVectorDyMatrixList = new ArrayList<>();
  private final List<Matrix> intEnergyMatrixList = new ArrayList<>();
  private final List<Matrix> extEnergyMatrixList = new ArrayList<>();
  private Matrix extEnergy;
  private Matrix intEnergy;
  private Matrix deltaIntEnergyToOrigState;
  private final List<Double> integralEnergyList = new ArrayList<>(); //contains overall energy during pocessing for every step
  private final List<Double> integralEnergyViaVertexList = new ArrayList<>();
  private int networkstateStartPt;
  private int networkstateEndPt;
  private int noOfDisplacements = 0;
  private boolean closedLine = false;
  private int featureID;

  /**
   * Constructor
   * Initilizes the field geometryTemp and deltaIntEnergy
   *
   * @param f JumpFeature
   */
  public LineToDisplace(Feature f, int dispFeatureID) {
    this.originalJumpFeature = f;
    this.geometryTemp = this.originalJumpFeature.getGeometry().copy();
    this.networkstateStartPt = 0;
    this.networkstateEndPt = 0;
    this.deltaIntEnergyToOrigState = MatlabSyntax.zeros(this.geometryTemp.getNumPoints(), 2);
    this.featureID = dispFeatureID;
    //------------------
    // check for first time if object is a closed LineString
    //-----------------
    LineString myLine = null;
    try {
      myLine = (LineString) this.getOriginalGeometry();
    } catch (Exception e) {
      System.out.println("LineToDisplace.constructor: geometry not a LineString");
    }
    this.checkIfclosed(0.1);
  }

  /**
   * checks if the linestring is closed by using crucialDistance and
   * the jts geometry.isClosed().<p>
   * This is done since a line might be not recognized as close by the jts
   * function. <p>
   * This method is called from the constructor with the threshold of 0.1m.
   *
   * @param crucialDistance in meter
   */
  public void checkIfclosed(double crucialDistance) {
    LineString line = (LineString) this.getOriginalGeometry();
    Point start = line.getStartPoint();
    Point end = line.getEndPoint();
    double dist = SecondGeodeticTask2d.calcDistancePoints(start, end);
    if (line.isClosed() || (dist < crucialDistance)) {
      this.setNetworkstateStartPt(1);
      this.setNetworkstateEndPt(1);
      this.closedLine = true;
      System.out.println("LineToDisplace.constructor: LineString is closed ..or start-endpoint distance < 0.1m!");
    }
  }

  /**
   * @return networkstate 0,1,2 or 3<p>
   * 0 = line is free
   * 1 = line is connected to network with startpoint<p>
   * 2 = line is connected to network with endpoint<p>
   * 3 = line is part of network<p>
   */
  public int getNetworkstate() {
    int state = 0;
    if (this.networkstateStartPt == 1) {
      state = 1;
    }
    if (this.networkstateEndPt == 1) {
      state = 2;
    }
    if ((this.networkstateEndPt == 1) && (this.networkstateStartPt == 1)) {
      state = 3;
    }
    return state;

  }

  /**
   * @return Returns the dispVectorDyMatrixList.
   */
  public List<Matrix> getDispVectorDyMatrixList() {
    return dispVectorDyMatrixList;
  }

  /**
   * @return Returns the dispVectorDxMatrixList.
   */
  public List<Matrix> getDispVectorDxMatrixList() {
    return dispVectorDxMatrixList;
  }

  /**
   * @return Returns the geometryTemp.
   */
  public Geometry getTempGeometry() {
    return geometryTemp;
  }

  /**
   * @param geometryTemp The geometryTemp to set.
   */
  public void setTempGeometry(Geometry geometryTemp) {
    //-- copy old geometry to list
    this.allGeometryStates.add(this.geometryTemp);
    this.geometryTemp = geometryTemp;
  }

  /**
   * @return Returns the networkstateEndPt.
   */
  public int getNetworkstateEndPt() {
    return networkstateEndPt;
  }

  /**
   * @param networkstateEndPt The networkstateEndPt to set.
   */
  public void setNetworkstateEndPt(int networkstateEndPt) {
    this.networkstateEndPt = networkstateEndPt;
  }

  /**
   * @return Returns the networkstateStartPt.
   */
  public int getNetworkstateStartPt() {
    return networkstateStartPt;
  }

  /**
   * @param networkstateStartPt The networkstateStartPt to set.
   */
  public void setNetworkstateStartPt(int networkstateStartPt) {
    this.networkstateStartPt = networkstateStartPt;
  }

  /**
   * @return Returns the originalFeature.
   */
  public Feature getOriginalJumpFeature() {
    return originalJumpFeature;
  }

  /**
   * @return Returns the extEnergy.
   */
  public Matrix getExtEnergy() {
    return extEnergy;
  }

  /**
   * @param extEnergy The extEnergy to set.<p>
   *                  Adds the energy to a evolution List.
   */
  public void setExtEnergy(Matrix extEnergy) {
    this.extEnergy = extEnergy;
    this.extEnergyMatrixList.add(extEnergy);
  }

  /**
   * @return Returns the intEnergy.
   */
  public Matrix getIntEnergy() {
    return intEnergy;
  }

  /**
   * @param intEnergy The intEnergy to set. <p>
   *                  Adds the energy to a evolution List.
   */
  public void setIntEnergy(Matrix intEnergy) {
    this.intEnergy = intEnergy;
    this.intEnergyMatrixList.add(intEnergy);
  }

  /**
   * @return the original geometry
   * obtained from field object originalJumpFeature
   */
  public Geometry getOriginalGeometry() {
    return this.originalJumpFeature.getGeometry();
  }

  /**
   * @return the number of steps which the line has already evaluated
   * and possibly modified<p>
   * the value is obtained using integralEnergyList.size()
   */
  public int noOfLoops() {
    return this.integralEnergyList.size();
  }

  /**
   * @return Returns the integralEnergyList.
   * List contains Double Objects.
   * IntegralEnergy has been calculated without consideration
   * of network state
   */
  public List<Double> getIntegralEnergyList() {
    return integralEnergyList;
  }

  public double getExternalEnergySum() {
    /** old
     //-- do abs() first since vector components can be negative
     Matrix ee = MatlabSyntax.abs((Matrix)this.extEnergy);
     AbstractMatrix ee2d = ee.sum();
     AbstractMatrix ees2d = ee2d.sum();
     //makes that sense to sum displacements of all points???
     double extEnergy = Math.sqrt(ees2d.get(0,0)*ees2d.get(0,0) + ees2d.get(0,1)*ees2d.get(0,1));
     return extEnergy;
     **/
    //--- new => it gives the same result :)
    double energy = 0;
    int n = this.extEnergy.getRowDimension();
    int points = 0;
    double dx, dy;
    for (int i = 0; i < n; i++) {
      dx = this.extEnergy.get(i, 0);
      dy = this.extEnergy.get(i, 1);
      energy = energy + Math.sqrt(dx * dx + dy * dy);
      //[sstein] new 21.08.05
      if ((dx > 0) || (dy > 0)) {
        points = points + 1;
      }
    }
    //-- avoid divison with zero
    if (points == 0) {
      points = 1;
    }
    return energy / points;
  }

  public double getInternalEnergySum() {
    double energy = 0;
    int n = this.intEnergy.getRowDimension();
    int points = 0;
    double dx, dy;
    for (int i = 0; i < n; i++) {
      dx = this.intEnergy.get(i, 0);
      dy = this.intEnergy.get(i, 1);
      energy = energy + Math.sqrt(dx * dx + dy * dy);
      if ((dx > 0) || (dy > 0)) {
        points = points + 1;
      }
    }
    //-- avoid divison with zero
    if (points == 0) {
      points = 1;
    }
    return energy / points;
  }

  /**
   * is false if  abs(extE) < 0.1 that is about 0.1m <p>
   * threshold of 0.1  due to numerical problems<p>
   * uses getExternalEnergySum() to receive extEnergy
   *
   * @return calculated from external energy
   */
  public boolean hasExternalConflicts() {
    if (this.getExternalEnergySum() < 0.1) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * transforms the line vertices into points with their
   * energys as attributes
   *
   * @return ArrayList of JumpPointFeatures
   */
  public List<Feature> createJumpPointFeaturesOfTempGeom() {
    List<Feature> myFeatures = new ArrayList<>();
    //-- create Schema
    FeatureSchema fs = new FeatureSchema();
    fs.addAttribute("Geometry", AttributeType.GEOMETRY);
    fs.addAttribute("eExt", AttributeType.DOUBLE);
    fs.addAttribute("eInt", AttributeType.DOUBLE);
    fs.addAttribute("integralEnergy", AttributeType.DOUBLE);
    double[][] energies = this.getVertexEnergies();
    //-- add vertices
    if (this.geometryTemp instanceof LineString) {
      LineString myLine = (LineString) this.geometryTemp;
      for (int i = 0; i < myLine.getNumPoints(); i++) {
        BasicFeature point = new BasicFeature(fs);
        point.setGeometry(myLine.getPointN(i));
        point.setAttribute("eExt", energies[i][0]);
        point.setAttribute("eInt", energies[i][1]);
        point.setAttribute("integralEnergy", energies[i][2]);
        myFeatures.add(point);
      }
    }
    return myFeatures;
  }

  /**
   * @return Returns the noOfDisplacements.
   */
  public int getNoOfDisplacements() {
    return noOfDisplacements;
  }

  /**
   * Increases noOfdisplacements + 1 .
   */
  public void noOfDisplacementsAdd() {
    this.noOfDisplacements = this.noOfDisplacements + 1;
    this.calcIntegralEnergieViaVerticesAndAddToList();
  }

  /**
   * @return Returns the allGeometryStates a list of Geometries
   * including the current geometry = geometryTemp.
   */
  public List<Geometry> getAllGeometryStates() {
    List<Geometry> geoms = new ArrayList<>(this.allGeometryStates);
    //-- add the actual geometry since it is not part of the list
    //   => setTempGeometry(geom x) makes that it is added to the list.
    geoms.add(this.geometryTemp);
    return geoms;
  }

  /**
   * does the line has conflicts? <br>
   * checks if external energy exists .. if yes then it also checks
   * if energies might be balanced (the integralEnergie is calculated from
   * the integralEnergy for every Vertex and summed up over the line)
   *
   * @return
   */
  public boolean hasConflicts() {
    boolean retvalue;
    if (this.hasExternalConflicts()) {
      double[][] energies = this.getVertexEnergies();
      double integralEnergy = 0;
      LineString myLine = (LineString) this.geometryTemp;
      for (int i = 0; i < myLine.getNumPoints(); i++) {
        integralEnergy = integralEnergy + energies[i][2];
      }
      //-- balance of energy
      if (Math.abs(integralEnergy) < 0.1) {
        retvalue = false;
      } else { // one of the energies is much more stronger
        retvalue = true;
      }
    } else {
      retvalue = false;
    }
    return retvalue;
  }

  /**
   * calculates energies per vertex
   * double array with energies [ ext | int  | integral ]
   * v1	  [ ... | ...  | ...      ]
   * v2	  [ ... | ...  | ...      ]
   *
   * @return
   */
  private double[][] getVertexEnergies() {
    LineString myLine = (LineString) this.geometryTemp;
    double[][] energies = new double[myLine.getNumPoints()][3];
    for (int i = 0; i < myLine.getNumPoints(); i++) {
      double eExt = Math.sqrt(this.extEnergy.get(i, 0) * this.extEnergy.get(i, 0) +
          this.extEnergy.get(i, 1) * this.extEnergy.get(i, 1));
      if ((i == 0) && (this.getNetworkstateStartPt() == 1)) {
        eExt = 0;
      }
      if ((i == myLine.getNumPoints() - 1) && (this.getNetworkstateEndPt() == 1)) {
        eExt = 0;
      }
      double eInt = Math.sqrt(this.intEnergy.get(i, 0) * this.intEnergy.get(i, 0) +
          this.intEnergy.get(i, 1) * this.intEnergy.get(i, 1));
      //--
      double dx = this.extEnergy.get(i, 0) + this.intEnergy.get(i, 0);
      double dy = this.extEnergy.get(i, 1) + this.intEnergy.get(i, 1);
      double nrg = Math.sqrt(dx * dx + dy * dy);
      //--
      energies[i][0] = eExt;
      energies[i][1] = eInt;
      energies[i][2] = nrg;
    }
    return energies;
  }

  /**
   * Integral Energy calculated for every vertex
   * and then summed up for the line by considering the network state. <br>
   * This is used for calculation of hasConflict().
   * Evaluation takes place if noOfDisplacementsAdd() is used.<br>
   * Thus, the list will contain not the actual state after displacement
   * use therefore getCurrentIntegralEnergyViaVertices.
   *
   * @return double value
   */
  public double[] getIntegralEnergieListViaVertices() {
    double[] eng = new double[this.integralEnergyViaVertexList.size() + 1];
    for (int i = 0; i < eng.length - 1; i++) {
      Double val = this.integralEnergyViaVertexList.get(i);
      eng[i] = val;
    }
    return eng;
  }

  /**
   * this function is called if noOfDisplacementsAdd() is called. <br>
   * Thus, the list will contain not the actual state after displacement.
   */
  private void calcIntegralEnergieViaVerticesAndAddToList() {
    double[][] energies = this.getVertexEnergies();
    double integralEnergy = 0;
    LineString myLine = (LineString) this.geometryTemp;
    for (int i = 0; i < myLine.getNumPoints(); i++) {
      integralEnergy = integralEnergy + energies[i][2];
    }
    this.integralEnergyViaVertexList.add(integralEnergy);
  }

  /**
   * The actual integralEnergy calculated for every vertex and summed up for the line.
   * Use that value as last value to the energies obtained by getIntegralEnergieListViaVertices(),
   * if a new energy evaluation has been done. This value considers the networkstate.
   *
   * @return
   */
  public double getCurrentIntegralEnergyViaVertices() {
    double[][] energies = this.getVertexEnergies();
    double integralEnergy = 0;
    LineString myLine = (LineString) this.geometryTemp;
    int n = myLine.getNumPoints();
    int points = 0;
    for (int i = 0; i < n; i++) {
      integralEnergy = integralEnergy + energies[i][2];
      if (energies[i][2] > 0) {
        points = points + 1;
      }
    }
    //-- avoid divison with zero
    if (points == 0) {
      points = 1;
    }
    return integralEnergy / points;
  }

  /**
   * @return Returns a list of 2D matrices containing
   * external energy per vertex.
   */
  public List<Matrix> getExtEnergyMatrixList() {
    return extEnergyMatrixList;
  }

  /**
   * @return Returns a list of 2D matrices containing
   * internal energy per vertex.
   */
  public List<Matrix> getIntEnergyMatrixList() {
    return intEnergyMatrixList;
  }

  /**
   * @return Returns the differenz between the
   * Internal Energies of the Original Line and the actual displaced Line.
   */
  public Matrix getDeltaIntEnergyToOrigState() {
    //-- calc deltaIntEnergy
    int size = this.intEnergyMatrixList.size();
    if (size > 1) {
      this.deltaIntEnergyToOrigState = SnakesEnergyDisplacement.calcIntEnergyDiff(
          this.getIntEnergyMatrixList().get(0),
          this.getIntEnergyMatrixList().get(size - 1));
    }
    return deltaIntEnergyToOrigState;
  }

  public double getDeltaIntEnergySum() {
    double energy = 0;
    Matrix deltaEnergy = this.getDeltaIntEnergyToOrigState();
    int n = deltaEnergy.getRowDimension();
    int points = 0;
    double dx, dy;
    for (int i = 0; i < n; i++) {
      dx = deltaEnergy.get(i, 0);
      dy = deltaEnergy.get(i, 1);
      energy = energy + Math.sqrt(dx * dx + dy * dy);
      if ((dx > 0) || (dy > 0)) {
        points = points + 1;
      }
    }
    //-- avoid divison with zero
    if (points == 0) {
      points = 1;
    }
    return energy / points;
  }

  /**
   * @return Returns the lineClosed.
   */
  public boolean isClosedLine() {
    return closedLine;
  }

  /**
   * Should be used if original LineString was closed. Use therefore #.isClosedLine()<p>
   * Calculate new start and endpoint coordinates by interpolation
   * of actual start- and endPoint coordinates
   */
  public void closeTempGeometry() {
    try {
      LineString myLine = (LineString) this.geometryTemp;
      Point ptStart = myLine.getStartPoint();
      Point ptEnd = myLine.getEndPoint();
      double x = (ptStart.getX() + ptEnd.getX()) / 2;
      double y = (ptStart.getY() + ptEnd.getY()) / 2;
      Coordinate coordStart = ptStart.getCoordinate();
      Coordinate coordEnd = ptEnd.getCoordinate();
      coordStart.x = x;
      coordStart.y = y;
      coordEnd.x = x;
      coordEnd.y = y;
    } catch (Exception e) {
      System.out.println("LineToDisplace.constructor: geometry not a LineString");
    }
  }

  /**
   * @return Returns the featureID.
   */
  public int getFeatureID() {
    return featureID;
  }
}
