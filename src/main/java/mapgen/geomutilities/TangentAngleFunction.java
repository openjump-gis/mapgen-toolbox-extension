package mapgen.geomutilities;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.locationtech.jts.geom.Coordinate;

/**
 * description
 * calculates - azimut
 * - tangent angle function
 * - curvature
 * - variance of curvature
 * <p>
 * created on 		12.11.2004
 * last modified: 	17.11.2004
 *
 * @author sstein
 */
public class TangentAngleFunction {

  private final List<Double> azimut = new ArrayList<>();
  private final List<Double> taf = new ArrayList<>();
  private final List<Double> curv = new ArrayList<>();
  private final List<Double> distances = new ArrayList<>();
  private double curvVar = 0;
  private double curvMean = 0;
  private double curvDeviation = 0;
  private double nrVertices = 0;
  private double nrVariancePoints = 0;
  private double curvThreshold = Math.PI * 2;
  private double distanceMean = 0;
  private double distanceVariance = 0;
  private double distanceDeviation = 0;
  private boolean curvWasCalculated;
  private boolean varianceWasCalculated;
  private boolean isRing;
  private boolean distMomentsCalculated;

  public TangentAngleFunction(Coordinate[] coordArray) {

    // %%%%%%%% sTWFss.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //  erstellt: 5.11.2002
    //  modifiziert: 1.03.2003
    //
    //  urspruenglich in xy2tawidyad.m enthalten
    //
    //  Berechnung von s und TWF
    //  wobei twf moeglichst ohne Spruenge berechnet wird
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //  function[ss,kat] = TWFss (x,y)

    this.curvWasCalculated = false;
    this.varianceWasCalculated = false;
    this.distMomentsCalculated = false;
    this.isRing = false;
    // --- if polygon ring then add the second point to end of list
    //     to calculate the azimuth for the start/end point
    int end = coordArray.length;
    if ((coordArray[0].x == coordArray[end - 1].x) &&
        (coordArray[0].y == coordArray[end - 1].y)) {
      this.isRing = true;
      Coordinate[] newArray = new Coordinate[end + 1];
      for (int i = 0; i < newArray.length; i++) {
        if (i < newArray.length - 1) {
          newArray[i] = (Coordinate) coordArray[i].clone();
        } else { //new last point = second point
          newArray[i] = (Coordinate) coordArray[1].clone();
        }
      }
      coordArray = newArray;
    }
    //--------
    double sprung = 0, gw = 0, indx = 0;
    int fall;
    this.nrVertices = coordArray.length;
    double[] phi = new double[(int) this.nrVertices];
    for (int i = 0; i < phi.length - 1; i++) {

      double rp = coordArray[i].x;      //r=east , h=north
      double hp = coordArray[i].y;
      double rpn = coordArray[i + 1].x;   //naechst point east,north
      double hpn = coordArray[i + 1].y;

      double dr = rpn - rp;          //zaehler = delta easting
      double dh = hpn - hp;          //nenner = delta north
      double ss = Math.sqrt(dr * dr + dh * dh);
      if ((isRing = true) && (i == phi.length - 2)) {
        //don't calc distance
      } else {
        this.distances.add(ss);
      }
      //-- calculation of angle to horizontal = tangent angle =  phi=atan(dy/dx)
      double rawAngle = Math.atan(dh / dr);
      if ((dr > 0) && (dh > 0)) {
        phi[i] = rawAngle;
        fall = 1;
      } else if ((dr > 0) && (dh < 0)) {
        phi[i] = rawAngle + 2.0 * Math.PI;
        fall = 2;
      } else if ((dr < 0) && (dh < 0)) {
        phi[i] = rawAngle + Math.PI;
        fall = 3;
      } else if ((dr < 0) && (dh > 0)) {
        phi[i] = rawAngle + Math.PI;
        fall = 4;
      } else if ((dr == 0) && (dh < 0)) {
        phi[i] = Math.PI * 3.0 / 2;
        fall = 5;
      } else if ((dr == 0) && (dh > 0)) {
        phi[i] = Math.PI / 2.0;
        fall = 6;
      } else if ((dh == 0) && (dr > 0)) {
        phi[i] = 0;
        fall = 7;
      } else if ((dh == 0) && (dr < 0)) {
        phi[i] = Math.PI;
        fall = 8;
        sprung = sprung + 1;
        System.out.println(sprung + " uncontinuousity  possible!");
      } else {
        System.out.println("angle was not calculated dr:" + dr + " dh: " + dh);
        System.out.println("prevoius angle used");
        if (i == 1) {
          phi[i] = 0;
        } else {
          phi[i] = phi[i - 1];
        }
      }
      this.azimut.add(phi[i]);

      //-- correction of azimuths to values > 2pi to obtain continuous function
      if (i == 0) {
        if (phi[i] < Math.PI / 2) {
          phi[i] = phi[i] + 2 * Math.PI;
        } // neu: 5.12.02
      } else {
        //double me = phi[i - 1];
        //double you = phi[i];
        if ((phi[i - 1] > (3.0 / 2 * Math.PI)) && (phi[i] <= 1 * Math.PI)) {
          double j = this.fix(phi[i] / (2 * Math.PI)) + 1;
          phi[i] = j * (2 * Math.PI) + phi[i];
        }
      }
      this.taf.add(phi[i]);
    }
    //-- add zero value for last point ..
    //   if object is Ring an additional last point was set already
    if (!this.isRing) {
      this.azimut.add((double) 0);
      this.taf.add((double) 0);
    }

  }

  private void calcCurvature() {
    this.curvWasCalculated = true;
    //-- set first value
    this.curv.add((double) 0);
    //-- calc other values ..
    //   .. until last but one value for lines (last is zero)
    //   .. until last value for rings
    int to = 1;
    if (!this.isRing) {
      to = 2;
    }
    for (int i = 0; i < this.taf.size() - to; i++) {

      Double val1 = this.azimut.get(i);
      Double val2 = this.azimut.get(i + 1);
      double curv = val2 - val1;
      //-- rule 1
      if (curv >= Math.PI) {
        curv = -1 * (2 * Math.PI - curv);
      }
      //-- rule 2
      if (curv <= (-1 * Math.PI)) {
        curv = 2 * Math.PI - Math.abs(curv);
      }

      this.curv.add(curv);

      //-- the following short algorithm makes still some
      //   mistakes: calcs sometimes curv > pi/2
            /*Double val3= (Double)this.taf.get(i);
            Double val4= (Double)this.taf.get(i+1);
            double curv2 =val3.doubleValue() - val4.doubleValue();
            this.curv.add(new Double(curv2));
            */
    }
    //-- add last value
    //   if not Ring, then set last value to zero
    if (!this.isRing) {
      this.curv.add(0.0);
    } else { //if it is a ring, than set first value to last value
      int lastIndex = this.curv.size() - 1;
      Double lastcurv = this.curv.get(lastIndex);
      this.curv.set(0, lastcurv);
    }
  }

  /**
   * calculates curvature variance,
   * does not take into account curvValues == 0
   * and over the given threshold
   */
  private void calcCurvatureVariance(double threshold) {
    this.varianceWasCalculated = true;
    double nrPt = 0;
    double sum = 0;
    double sumMean = 0;
    for (int i = 0; i < this.curv.size() - 1; i++) {
      Double val = this.curv.get(i);
      if ((Math.abs(val) <= threshold) &&
          (val != 0)) {
        nrPt = nrPt + 1;
        sum = sum + (val * val);
        sumMean = sumMean + val;
      }
    }
    this.nrVariancePoints = nrPt;
    if (nrPt == 1) {
      nrPt = 2;
    }
    this.curvVar = 1.0 / (nrPt - 1) * sum;

    this.curvDeviation = Math.sqrt(this.curvVar);
    this.curvMean = sumMean / nrPt;
  }

  /**
   * fix rounds double to integer - towards zero
   */
  private double fix(double val) {
    double returnval = 0;
    if (val > 0) {
      returnval = Math.floor(val);
    }
    if (val < 0) {
      returnval = Math.ceil(val);
    }
    return returnval;
  }

  private void calcDistMoments() {
    this.distMomentsCalculated = true;
    double sumMean = 0;
    double sumVar = 0;
    double count = 0;
    // -- mean (0. moment)
    int max;
    if (this.isRing) {
      max = this.distances.size() - 1;
    } else {
      max = this.distances.size();
    }
    for (int i = 0; i < max; i++) {
      count = count + 1;
      Double value = this.distances.get(i);
      sumMean = sumMean + value;
    }
    this.distanceMean = sumMean / count;

    // -- variance (1. moment)
    for (int i = 0; i < max; i++) {
      count = count + 1;
      Double value = this.distances.get(i);
      double dx = value - this.distanceMean;
      sumVar = sumVar + dx * dx;
    }
    this.distanceVariance = sumVar / (count - 1.0);
    this.distanceDeviation = Math.sqrt(this.distanceVariance);
  }

  /************************** getters and setters ************************/

  public double getCurvThreshold() {
    return curvThreshold;
  }

  public void setCurvThreshold(double curvThreshold) {
    this.curvThreshold = curvThreshold;
  }

  public double[] getAzimut() {
    double[] azimut = new double[this.azimut.size()];
    int i = 0;
    for (Double value : this.azimut) {
      azimut[i] = value;
      i = i + 1;
    }
    return azimut;
  }

  public double[] getCurv() {
    if (!this.curvWasCalculated) {
      this.calcCurvature();
    }
    double[] curvs = new double[this.curv.size()];
    int i = 0;
    for (Double value : this.curv) {
      curvs[i] = value;
      i = i + 1;
    }
    return curvs;
  }

  /**
   * @return variance of curvature for a line
   * does not take zero curvature values into account
   */
  public double getCurvVar() {
    if (!this.curvWasCalculated) {
      this.calcCurvature();
    }
    if (!this.varianceWasCalculated) {
      this.calcCurvatureVariance(this.curvThreshold);
    }
    return curvVar;
  }

  public double getCurvDeviation() {
    if (!this.curvWasCalculated) {
      this.calcCurvature();
    }
    if (!this.varianceWasCalculated) {
      this.calcCurvatureVariance(this.curvThreshold);
    }
    return this.curvDeviation;
  }

  public double getCurvMean() {
    if (!this.curvWasCalculated) {
      this.calcCurvature();
    }
    if (!this.varianceWasCalculated) {
      this.calcCurvatureVariance(this.curvThreshold);
    }
    return this.curvMean;
  }

  public double[] getDistances() {

    double[] dists = new double[this.distances.size()];
    int i = 0;
    for (Double value : this.distances) {
      dists[i] = value;
      i = i + 1;
    }
    return dists;
  }

  /**
   * @return the number of points which were used to
   * calculate curvature variance,
   * if smaller than 10 => variance value should not be used
   */
  public double getNrVariancePoints() {
    return nrVariancePoints;
  }

  public double getNrVertices() {
    return nrVertices;
  }

  public double[] getTaf() {

    double[] taf = new double[this.taf.size()];
    int i = 0;
    for (Double value : this.taf) {
      taf[i] = value;
      i = i + 1;
    }
    return taf;

  }

  /**
   * describes the given coordinates array a ring (last=first point)?
   *
   * @return boolean value
   */
  public boolean isRing() {
    return isRing;
  }

  public double getDistanceMean() {
    if (!this.distMomentsCalculated) {
      this.calcDistMoments();
    }
    return this.distanceMean;
  }

  public double getDistanceVariance() {
    if (!this.distMomentsCalculated) {
      this.calcDistMoments();
    }
    return this.distanceVariance;
  }

  public double getDistanceDeviation() {
    if (!this.distMomentsCalculated) {
      this.calcDistMoments();
    }
    return this.distanceDeviation;
  }


}

	