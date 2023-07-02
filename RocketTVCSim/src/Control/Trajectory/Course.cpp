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
