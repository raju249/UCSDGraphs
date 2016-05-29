package roadgraph;

import geography.GeographicPoint;

class MapEdge {

		// start and end of the edge on the Node
		private MapNode start;
		private MapNode end;
		
		// Name of the Road joining the vertices with this edge
		private String roadName;
		
		// type of road
		private String roadType;
		
		// length of the road in km
		private double length;
				
		// Constructor
		public MapEdge(String roadName, String roadType, 
				MapNode node1, MapNode node2,double length){
			
				this.roadName = roadName;
				this.roadType = roadType;
				start = node1;
				end = node2;
				this.length = length;
		}
		
		// @return endNode for this node
		public MapNode getEndNode(){
			return end;
		}
		
		// @return location of start
		public GeographicPoint getStartPoint(){
			return start.getLocation();
		}
		
		// @return location of end 
		public GeographicPoint getEndPoint(){
			return end.getLocation();
		}
		
		// @return length
		public double getLength(){
			return length;
		}
		
		// @return roadName
		public String getRoadName(){
			return roadName;
		}
		
		// @return roadType
		public String getRoadType(){
			return roadType;
		}
		
		// @return the other node given one node
		public MapNode getOtherNode(MapNode node){
			
			if (node.equals(start)){
				return end;
			}
			else if (node.equals(end)){
				return start;
			}
			else{
				throw new IllegalArgumentException("No point found!");
			}
		}
}
