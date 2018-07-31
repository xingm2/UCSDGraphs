/**
 * @author Menglong Xing
 *
 * A class which represents a node in the map
 * 
 */

package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

class MapNode implements Comparable {
	// The list of edges associated with this node
	private HashSet<MapEdge> edges;

	// The location of this node
	private GeographicPoint location;

	// WEEK 4 SOLUTIONS
	
	/** the predicted distance of this node (used in Week 3 algorithms) */
	private double distance;
	
	/** the actual distance of this node from start (used in Week 3 algorithms) */
	private double actualDistance;
	
	// END WEEK 4 SOLUTIONS

	/**
	 * Constructor
	 */
	MapNode(GeographicPoint loc){
	  location = loc;
	  edges = new HashSet<MapEdge>();
	}

	/**
	 * Add an edge that is outgoing from this node in the graph
	 * @param edge The edge to be added
	 */
	void addEdge(MapEdge edge){
		edges.add(edge);
	}

	/**
	 * @return a set containing all the neighbors of this Mapnode
	 */
	Set<MapNode> getNeighbors(){
		Set<MapNode> neighbors = new HashSet<MapNode>();
        for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
    
    /**
     * @return the geographic location of this node.
     */
    GeographicPoint getLocation(){
    	return location;
    }

    /**
     * @return a set containing all the edges out of this node.
     */
    Set<MapEdge> getEdges(){
    	return edges;
    }

    /**
     * Returns whether two nodes are equal.
     * Nodes are considered equal if their locations are the same,
     * even if their street list is different.
     * @param o the node to compare to 
     * @return true if the two nodes are at the same location, false otherwise
     */
     @Override
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}

    /** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode()
	{
		return location.hashCode();
	}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString()
	{
		String toReturn = "(";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	} 

    //  WEEK 4 SOLUTIONS 
	
	// get node distance (predicted)
	public double getDistance() {
		return this.distance;
	}
	
	// set node distance (predicted)
	public void setDistance(double distance) {
	    this.distance = distance;
	}
 	// get node distance (actual)
	public double getActualDistance() {
		return this.actualDistance;
	}
	
	// set node distance (actual)	
	public void setActualDistance(double actualDistance) {
	    this.actualDistance = actualDistance;
	}
	
    // Code to implement Comparable
	public int compareTo(Object o) {
		// convert to map node, may throw exception
		MapNode m = (MapNode)o; 
		return ((Double)(this.getDistance() + this.getActualDistance())).compareTo((Double) (m.getDistance() + m.getActualDistance()));
	}
 	// END WEEK 4 SOLUTIONS

}