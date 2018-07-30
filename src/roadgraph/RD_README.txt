Class: MapGraph

Modifications made to MapGraph (what and why):

1. What: Implemented a constructor public MapGraph(). 
		 Implemented methods:
		 a. public int getNumVertices()
		 b. public Set<GeoraphicPoint> getVertices()
		 c. public int getNumEdges()

   Why:  These are required methods.

2. What: Implemented methods:
		 a. public boolean addVertex(GeographicPoint location)
		 b. public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)

   Why:  These methods are used to build the graph.

3. What: Implemented methods:
		 a. public List<GeographicPoint> bfs(GeographicPoint start, GeographicPpint goal, Consumer<GeographicPoint> nodeSearched)
		 b. helper method: private List<GeographicPoint> reconstructPath(HashMap<MapNode,MapNode> parentMap, MapNode start, MapNode goal)

   Why:  These are the methods which really find the shortest path using BFS algorithm.

Class name: MapNode

Purpose and description of class: A class which represents a node in the map.

Class name: MapEdge

Purpose and description of class: A class which represents a directed edge in the map.


Overall Design Justification :

As you may see here, I closely followed the suggestions and the instructions given by Christine, Mia and Leo for this week's homework. I am trying to complete this course with limited time. I truly appreciate your understanding. Thank you. 