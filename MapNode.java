package roadgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {

	private GeographicPoint node;
	private ArrayList<GeographicPoint> neighbor;

	

	
	public MapNode(GeographicPoint pt1)
	{
		node = pt1;
		neighbor = new ArrayList<GeographicPoint>();
	}

	public void addNeighbor(GeographicPoint point){
		
		neighbor.add(point);
	}
	
	public ArrayList<GeographicPoint> getNeighbor(){
		return neighbor;
	}

	// return edge as String
	public GeographicPoint getNode()
	{			
		return node;
	}
	
}
