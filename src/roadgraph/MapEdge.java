package roadgraph;

import java.util.LinkedList;
import java.util.List;
import geography.GeographicPoint;

/**
 * @author Menglong Xing
 *
 *  A class which represents a directed edge in the map.
 * 
 */

class MapEdge {
  // name
  private String roadName;

  // type
  private String roadType;

  // start and end Node
  private MapNode start,end;

  // length
  private double length;

  static final double DEFAULT_LENGTH = 0.01;  
  
  /**
   *  Constructor
   *
   *  @param roadName
   *  @param n1  The Node at one end of the segment
   *  @param n2  The Node at the other end of the segment
   *  
   */
  MapEdge(String roadName, MapNode n1, MapNode n2){
      this(roadName, "", n1, n2, DEFAULT_LENGTH);
  }

  /**
   *  Constructor
   *
   *  @param roadName
   *  @param roadType
   *  @param n1  The Node at one end of the segment
   *  @param n2  The Node at the other end of the segment
   *  
   */
  MapEdge(String roadName, String roadType, MapNode n1, MapNode n2){
      this(roadName, roadType, n1, n2, DEFAULT_LENGTH);
  }

  /**
   *  Constructor
   *
   *  @param roadName
   *  @param roadType
   *  @param n1  The Node at one end of the segment
   *  @param n2  The Node at the other end of the segment
   *  @param length
   *  
   */
  MapEdge(String roadName, String roadType, MapNode n1, MapNode n2, double length){
      this.roadName = roadName;
      start = n1;
      end = n2;
      this.roadType = roadType;
      this.length = length;
  }
  
  /**
   * @return the MapNode for the end point
   */
  MapNode getEndNode() {
  	return end;
  }

  /**
   * @return the location of the start point as a GeographicPoint
   */
  GeographicPoint getStartPoint() {
  	return start.getLocation();
  }

  /**
   * @return the location of the End point as a GeographicPoint
   */
  GeographicPoint getEndPoint() {
  	return end.getLocation();
  }

  /**
   * @return the length of the road segment
   */
  double getLength(){
  	return length;
  }  

  /**
   * @return the name of the road 
   */
  public String getRoadName(){
  	return roadName;
  }

  /**
   * Given one of the nodes on this edge, return the other one
   * @param node One node
   * @return the other one
   */
   MapNode getOtherNode(MapNode node){
	if (node.equals(start)) 
		return end;
	else if (node.equals(end))
		return start;
	throw new IllegalArgumentException("Looking for " +
		"a point that is not in the edge");
   }

   /**
    * Return a String representation for this edge.
    */
   @Override
   public String toString(){
   	 String toReturn = "[EDGE between ";
	 toReturn += "\n\t" + start.getLocation();
	 toReturn += "\n\t" + end.getLocation();
	 toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
	 		" Segment length: " + String.format("%.3g", length) + "km";

     return toReturn;
   }

} // MapEdge

