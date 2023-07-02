#include "Course.h"
using namespace std;
Course::Course(string fileName)
{
	//Read in list of waypoints from csv file

	//Open file
	std::ifstream file(fileName);
	vector<string> row;

	if (file.is_open())
	{
		//Read in lines
		std::string line, word;
		while (getline(file, line))
		{
			row.clear();

			stringstream str(line);

			while (getline(str, word, ','))
				row.push_back(word);

			//Convert tokens to doubles
			double x = 0;
			double y = 0;
			double z = 0;
			try
			{
				x = std::stod(row[0]);
				y = std::stod(row[1]);
				z = std::stod(row[2]);
			}
			catch (std::exception e)
			{
				continue;
			}
			//Add waypoint
			this->waypoints.push_back(ChVector<>(x, y, z));
		}
		file.close();
	}
	else
	{
		std::cout << "Unable to open file" << std::endl;
		throw std::exception("Unable to open file");
	}
}


std::vector<ChVector<>> Course::getWaypoints()
{
	return this->waypoints;
}
/*
Returns the next point along the segment from currentWaypoint to nextWaypoint that is a distance lookahead away from currentPos
*/
ChVector<> getPathCross(ChVector<> currentWaypoint, ChVector<> nextWaypoint, ChVector<> currentPosition, float lookahead)
{

	float x0 = currentWaypoint[0];
	float	y0 = currentWaypoint[1];
	float	z0 = currentWaypoint[2];
	float	x1 = nextWaypoint[0];
	float	y1 = nextWaypoint[1];
	float	z1 = nextWaypoint[2];
	float	x = currentPosition[0];
	float	y = currentPosition[1];
	float	z = currentPosition[2];

	float	A = pow((x0 - x), 2) + pow((y0 - y), 2) + pow((z0 - z), 2) - pow(lookahead, 2);
	float	C = pow((x0 - x1), 2) + pow((y0 - y1), 2) + pow((z0 - z1), 2);
	float	B = pow((x1 - x), 2) + pow((y1 - y), 2) + pow((z1 - z), 2) - A - C - pow(lookahead, 2);

	float determinent = pow(B, 2) - 4 * A * C;
	if (determinent < 0)
	{
		//No intersection
		return NULL;
	}
	float t1 = (-B + sqrt(determinent)) / (2 * C);
	float t2 = (-B - sqrt(determinent)) / (2 * C);
	if (t1 < 0 && t2 < 0)
	{
		//No intersection
		return NULL;
	}
	else if (t1 > 1 && t2 > 1)
	{
		return NULL;
	}
	else if (t1 > 1 && t2 < 0)
	{
		return NULL;
	}
	else if (t1 < 0 && t2 > 1)
	{
		return NULL;
	}

	//There should always be two solutions, but we will pick the one closest to 1
	float t = max(t1, t2);
	//plug t into the parametric equation of the line to get the point of intersection
	float xCross = x0 * (1 - t) + t * (x1);
	float yCross = y0 + t * (y1 - y0);
	float zCross = z0 + t * (z1 - z0);
	return ChVector<>(xCross, yCross, zCross);

}



ChVector<> Course::getLookaheadPoint(ChVector<> currentPosition, float lookahead) {
//	intersectionPoint = None
//		while (intersectionPoint is None) :
//			if (self.waypointIndex + 2 >= len(self.waypoints)) :
//				#We have reached the end of the path
//				return self.waypoints[-1]
//				#Not the most efficient but looksahead to next segment first.If not there, looks at current.Then increments waypoint index to look two ahead
//				intersectionPoint = self._getPathCross(self.waypoints[self.waypointIndex + 1], self.waypoints[self.waypointIndex + 2], currentPoint)
//				if (intersectionPoint is None) :
//
//					intersectionPoint = self._getPathCross(self.waypoints[self.waypointIndex], self.waypoints[self.waypointIndex + 1], currentPoint)
//				else:
//	self.waypointIndex += 1
//# if (self.waypointIndex > len(self.waypoints)-3):
//#     #We have reached the end of the path
//		#     return self.waypoints[-1]
//
//		return intersectionPoint

	ChVector<> intersectionPoint = NULL;
	while (intersectionPoint == NULL)
	{
		if (this->waypointIndex + 2 >= this->waypoints.size())
		{
			//We have reached the end of the path
			return this->waypoints.back();
		}
		//Not the most efficient but looksahead to next segment first.If not there, looks at current.Then increments waypoint index to look two ahead
		intersectionPoint = getPathCross(this->waypoints[this->waypointIndex + 1], this->waypoints[this->waypointIndex + 2], currentPosition, lookahead);
		if (intersectionPoint == NULL)
		{
			intersectionPoint = getPathCross(this->waypoints[this->waypointIndex], this->waypoints[this->waypointIndex + 1], currentPosition, lookahead);
		}
		else
		{
			this->waypointIndex += 1;
		}
	}
return intersectionPoint;
}


