/**
 * @author UCSD MOOC development team and Khoa Tran
 * 
 * A class which represents an edge  in the graph between intersections
 *
 */
package roadgraph;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import geography.GeographicPoint;

public class MapEdge {

	private GeographicPoint start;
	private GeographicPoint end;

	private String roadName;
	private String roadType;
	private double length;

	public MapEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName, String roadType, double length)
	{
		start = pt1;
		end = pt2;
		this.roadName = roadName;
		this.roadType = roadType;
		this.setLength(length);
	}
	// return edge as String
	public String toString()
	{
		String toReturn = this.roadName + ", " +this.roadType + " [" + start +"; " + end + "]";
		return toReturn;
	}
	
	public double getLength() {
		return length;
	}
	
	public void setLength(double length) {
		this.length = length;
	}	
}
