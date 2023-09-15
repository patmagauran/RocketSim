#pragma once
#include <string>
#include <vector>
class CSVReader
{
	std::vector<std::vector<std::string>> data;
	std::string filename;
	public:
	CSVReader(std::string filename);
	const std::vector<std::vector<std::string>>& getData();
};

