package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

class MapNode implements Comparable{
	
	/*
	 * Lists of edges of neighbours
	 */
	private HashSet<MapEdge>edges;
	
	/*
	 * Lat and Long for this node
	 */
	private GeographicPoint location;
	
	private double distance;
	private double actualDistance;
	/*
	 * Constructor for this class
	 */
	public MapNode(GeographicPoint loc){
		location = loc;
		edges = new HashSet<MapEdge>();
	}
	
	//Add edge to this Node 
	public void addEdge(MapEdge edge){
		edges.add(edge);
	}
	
	// @return location of this MapNode
	public GeographicPoint getLocation(){
		return location;
	}
	
	// @return set of edges from this MapNode
	public Set<MapEdge> getEdges(){
		return edges;
	}

	// @return set of neighbors for this node
	public Set<MapNode>getNeighbors(){
		
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge: edges){
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	// method to compare if two nodes are equal or not
	public boolean equals(Object o){
		
		if (!(o instanceof MapNode)|| o == null){
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	public double getDistance(){
		return distance;
	}
	
	public void setDistance(double distance){
		this.distance = distance;
	}
	
	public double getActualDistance(){
		return actualDistance;
	}
	
	public void setActualDistance(double actualDistance){
		this.actualDistance = actualDistance;
	}
	
	public int compareTo(Object o){
		MapNode m = (MapNode)o;
		return ((Double)this.getDistance()).compareTo((Double)m.getDistance());
	}
}
