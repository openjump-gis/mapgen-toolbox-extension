package mapgen.measures.supportclasses;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


/**
 * Special ArrayList for objects of type ShortEdgeConflict
 * <p>
 * created on 		07.06.2005
 * last modified: 	03.07.2005 (method clone() added)
 *
 * @author sstein
 */
public class PointInLineConflictList {

  List<PointInLineConflict> conflicts;

  /**
   * Constructs a new, empty ShortEdgeConflict list.
   * <p>
   * All elements inserted into the List must implement the Comparable interface. Furthermore,
   * all such elements must be mutually comparable: e1.compareTo(e2) must not throw a ClassCastException
   * for any elements e1 and e2 in the List.
   */
  public PointInLineConflictList() {
    conflicts = new ArrayList<>();
  }

  /**
   * Adds the specified element to this List.
   * <p>
   * All elements inserted into the PriorityQueue must implement the Comparable interface. Furthermore,
   * all such elements must be mutually comparable: e1.compareTo(e2) must not throw a ClassCastException
   * for any elements e1 and e2 in the List.
   *
   * @see java.util.Collection#add(java.lang.Object)
   */
  public boolean add(PointInLineConflict conflict) {
    return conflicts.add(conflict);
  }

  /**
   * Removes all elements from the priority queue.
   *
   * @see java.util.Collection#clear()
   */
  public void clear() {
    conflicts.clear();
  }

  public int size() {
    return conflicts.size();
  }


  public boolean remove(PointInLineConflict conflict) {
    return conflicts.remove(conflict);
  }

  public void removeByIndex(int listIndex) {
    conflicts.remove(listIndex);
  }

  public List<PointInLineConflict> getList() {
    return conflicts;
  }

  public PointInLineConflict get(int index) {
    return conflicts.get(index);
  }

  public Iterator<PointInLineConflict> iterator() {
    return conflicts.iterator();
  }

  /**
   * @return strongest conflict, means smallest edge
   */
  public PointInLineConflict getStrongestConflict() {
    PointInLineConflict smallestE = null;
    try {
      //--init
      smallestE = this.conflicts.get(0);
      double smallestDist = smallestE.distance;
      //--search
      for (int i = 1; i < this.conflicts.size(); i++) {
        PointInLineConflict sec = this.conflicts.get(i);
        if (sec.distance < smallestDist) {
          smallestDist = sec.distance;
          smallestE = (PointInLineConflict) sec.clone();
        }
      }
    } catch (Exception e) {
      System.out.println("PointInLineConflictList: getStrongestConflict .. not possible => no conflicts");
    }
    return smallestE;
  }

  public int getSmallestPointInLineConflictListIndex() {
    PointInLineConflict smallestE;
    int index = 0;
    try {
      //--init
      smallestE = this.conflicts.get(0);
      index = 0;
      double smallestDist = smallestE.distance;
      //--search
      for (int i = 1; i < this.conflicts.size(); i++) {
        PointInLineConflict sec = this.conflicts.get(i);
        if (sec.distance < smallestDist) {
          smallestDist = sec.distance;
          smallestE = (PointInLineConflict) sec.clone();
          index = i;
        }
      }
    } catch (Exception e) {
      System.out.println("PointInLineConflictList: getSmallesPointInLineConflict .. not possible => no conflicts");
    }
    return index;
  }

  public Object clone() {
    PointInLineConflictList secl = new PointInLineConflictList();
    for (PointInLineConflict element : this.conflicts) {
      secl.add(element);
    }
    return secl;
  }
}
