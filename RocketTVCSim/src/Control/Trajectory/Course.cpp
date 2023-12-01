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
ChVector<> getPathCross(ChVector<> currentWaypoint, ChVector<> nextWaypoint, ChVector<> currentPosition, double lookahead)
{

	double x0 = currentWaypoint[0];
	double	y0 = currentWaypoint[1];
	double	z0 = currentWaypoint[2];
	double	x1 = nextWaypoint[0];
	double	y1 = nextWaypoint[1];
	double	z1 = nextWaypoint[2];
	double	x = currentPosition[0];
	double	y = currentPosition[1];
	double	z = currentPosition[2];

	double	A = pow((x0 - x), 2) + pow((y0 - y), 2) + pow((z0 - z), 2) - pow(lookahead, 2);
	double	C = pow((x0 - x1), 2) + pow((y0 - y1), 2) + pow((z0 - z1), 2);
	double	B = pow((x1 - x), 2) + pow((y1 - y), 2) + pow((z1 - z), 2) - A - C - pow(lookahead, 2);

	double determinent = pow(B, 2) - 4 * A * C;
	if (determinent < 0)
	{
		//No intersection
		return NULL;
	}
	double t1 = (-B + sqrt(determinent)) / (2 * C);
	double t2 = (-B - sqrt(determinent)) / (2 * C);
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
	double t = max(t1, t2);
	//plug t into the parametric equation of the line to get the point of intersection
	double xCross = x0 * (1 - t) + t * (x1);
	double yCross = y0 + t * (y1 - y0);
	double zCross = z0 + t * (z1 - z0);
	return ChVector<>(xCross, yCross, zCross);

}



ChVector<> Course::getLookaheadPoint(ChVector<> currentPosition, double lookahead) {
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
			if (intersectionPoint == NULL) {
				//Its not in current or next, we must have deviated too far from the path
				return NULL;
			}
		}
		else
		{
			this->waypointIndex += 1;
		}
	}
	return intersectionPoint;
}




ChVector<> Course::getClosestPoint(ChVector<> currentPosition)
{
	double closestDistance = 1000000;
	ChVector<> closestPoint;
	for (int i = 0; i < this->waypoints.size() - 2; i++) {
		ChVector<> closestPointOnSeg = getClosestPointOnSegment(currentPosition, waypoints.at(i), waypoints.at(i + 1));
		double distanceToClosestPoint = (closestPointOnSeg - currentPosition).Length();
		if (distanceToClosestPoint < closestDistance) {
			closestDistance = distanceToClosestPoint;
			closestPoint = closestPointOnSeg;
		}
	}
	return closestPoint;
}

ChVector<> Course::getClosestPointOnSegment(ChVector<> currentPosition, ChVector<> segmentStart, ChVector<> segmentEnd)
{
	//https://stackoverflow.com/questions/64663170/how-to-find-nearest-point-in-segment-in-a-3d-space
	ChVector<> v = segmentEnd - segmentStart;
	ChVector<> w = currentPosition - segmentStart;
	double c1 = w.Dot(v);
	if (c1 <= 0) {
		return segmentStart;
	}
	double c2 = v.Dot(v);
	if (c2 <= c1) {
		return segmentEnd;
	}
	double b = c1 / c2;
	ChVector<> Pb = segmentStart + b * v;
	return Pb;
}

bool Course::isPointOnSegment(ChVector<> currentPosition, ChVector<> segmentStart, ChVector<> segmentEnd)
{
	ChVector<> closestPoint = getClosestPointOnSegment(currentPosition, segmentStart, segmentEnd);
	double distanceToClosestPoint = (closestPoint - currentPosition).Length();
	if (distanceToClosestPoint < 10) {
		return true;
	}
	else {
		return false;
	}
};

double Course::getPercentComplete(ChVector<> closestPoint)
{
	double distanceOnCourse = 0;
	double totalDistance = 0;
	bool foundClosestPoint = false;
	for (int i = 0; i < this->waypoints.size() - 2; i++) {
		totalDistance += (this->waypoints.at(i) - this->waypoints.at(i + 1)).Length();
		if (!foundClosestPoint) {
			if (isPointOnSegment(closestPoint, this->waypoints.at(i), this->waypoints.at(i + 1))) {
				distanceOnCourse += (closestPoint - this->waypoints.at(i)).Length();
				foundClosestPoint = true;
			}
			else {
				distanceOnCourse += (this->waypoints.at(i) - this->waypoints.at(i + 1)).Length();
			}
		}
	}

	return distanceOnCourse / totalDistance;
}


