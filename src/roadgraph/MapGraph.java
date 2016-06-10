/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;
import roadgraph.MapEdge;
/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		pointNodeMap = new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return pointNodeMap.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		MapNode node = pointNodeMap.get(location);
		if (node == null){
			node = new MapNode(location);
			pointNodeMap.put(location, node);
			return true;
		}
		else{
			System.out.println("Node already exists!!");
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		MapNode start = pointNodeMap.get(from);
		MapNode end = pointNodeMap.get(to);
		if (start == null || end == null || length < 0){
			throw new IllegalArgumentException("Node not available!!");
		}
		else{
			MapEdge edge = new MapEdge(roadName,roadType,start,end,length);
			edges.add(edge);
			start.addEdge(edge);
		}
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	private Set<MapNode> getNeighbors(MapNode node){
		return node.getNeighbors();
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		// Setup for initialization and check validity if inputs
		if (start == null || goal == null){
			throw new NullPointerException("Cannot find node for gv=iven point!");
		}
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null || endNode == null){
			return null;
		}
		
		// Setup to begin bfs
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		toExplore.add(startNode);
		
		MapNode next = null;
		
		while(!toExplore.isEmpty()){
			
			next = toExplore.remove();
			
			// Hook for visualization.  See write up.
			nodeSearched.accept(next.getLocation());
			if (next.equals(endNode))break;
			
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors){
				if(!visited.contains(neighbor)){
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
			
		}
		if (!next.equals(endNode)){
			return null;
		}
		
		// Reconstruct the path now
		List<GeographicPoint> path = reconstructPath(parentMap,startNode,endNode);
		return path;
		
	}
	
	private List<GeographicPoint> reconstructPath(HashMap<MapNode,MapNode> parentMap, MapNode start, MapNode goal){
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null){
			throw new NullPointerException("Null values!");
		}
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null || endNode == null){
			return null;
		}
		HashMap<MapNode,MapNode>parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		for (MapNode node : pointNodeMap.values()){
			node.setDistance(Double.POSITIVE_INFINITY);
		}
		startNode.setDistance(0);
		
		toExplore.add(startNode);
		MapNode next = null;
		int count = 0;
		
		while (!toExplore.isEmpty()){
			next = toExplore.remove();
			nodeSearched.accept(next.getLocation());
			count++;
			
			if(next.equals(endNode))break;
			if(!visited.contains(next)){
				visited.add(next);
				Set<MapEdge> edges = next.getEdges();
				
				for(MapEdge edge : edges){
					MapNode neighbor = edge.getOtherNode(next);
					if (!visited.contains(neighbor)){
						
							double curr = next.getDistance() + edge.getLength();
							if (curr < neighbor.getDistance()){
								toExplore.add(neighbor);
								neighbor.setDistance(curr);
								parentMap.put(neighbor,next);
							}
					}
				}
			}
			
		}
		
		if (!next.equals(endNode))return null;
		
		List<GeographicPoint> path = reconstructPath(parentMap,startNode,endNode);
		System.out.println(count);
		return path;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if (start == null || goal == null){
			throw new NullPointerException("Null values!");
		}
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null || endNode == null){
			return null;
		}
		HashMap<MapNode,MapNode>parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		for (MapNode node : pointNodeMap.values()){
			node.setDistance(Double.POSITIVE_INFINITY);
			node.setActualDistance(Double.POSITIVE_INFINITY);
		}
		startNode.setDistance(0);
		startNode.setActualDistance(0);
		toExplore.add(startNode);
		MapNode next = null;
		int count = 0;
		
		while (!toExplore.isEmpty()){
			next = toExplore.remove();
			nodeSearched.accept(next.getLocation());
			count++;
			
			if(next.equals(endNode))break;
			if(!visited.contains(next)){
				visited.add(next);
				Set<MapEdge> edges = next.getEdges();
				
				for(MapEdge edge : edges){
					MapNode neighbor = edge.getEndNode();
					if (!visited.contains(neighbor)){
						
							double curr = next.getActualDistance() + edge.getLength();
							double preDist = curr + (neighbor.getLocation().distance(endNode.getLocation()));
							if (preDist < neighbor.getDistance()){
								toExplore.add(neighbor);
								neighbor.setDistance(preDist);
								neighbor.setActualDistance(curr);
								parentMap.put(neighbor,next);
							}
					}
				}
			}
			
		}
		
		if (!next.equals(endNode))return null;
		
		List<GeographicPoint> path = reconstructPath(parentMap,startNode,endNode);
		System.out.println(count);
		return path;
	}

	
	
	public static void main(String[] args)
	{
		//System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		//System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		//MapGraph theMap = new MapGraph();
		//System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		//System.out.println("DONE.");

		//GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		//GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		GeographicPoint start = new GeographicPoint(1.0, 1.0); 
		GeographicPoint end = new GeographicPoint(8.0, -1.0); // 
		//List<GeographicPoint> route = theMap.bfs(start,end); // L
		List<GeographicPoint> route = theMap.dijkstra(start,end); 
		List<GeographicPoint> route1 = theMap.aStarSearch(start,end);

		//List<GeographicPoint> route = theMap.dijkstra(start,end);
		//List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
