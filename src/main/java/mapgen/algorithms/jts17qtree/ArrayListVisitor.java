package mapgen.algorithms.jts17qtree;

import java.util.*;

import mapgen.algorithms.jts17qtree.ItemVisitor;

/**
 * @version 1.7
 */
public class ArrayListVisitor implements ItemVisitor {

  private ArrayList items = new ArrayList();
    public ArrayListVisitor() {
  }

  public void visitItem(Object item)
  {
    items.add(item);
  }

  public ArrayList getItems() { return items; }

}