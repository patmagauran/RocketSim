#pragma once
#include <unordered_map>
#include <string>
class DataRow {

public: 
	float timestamp;
	std::unordered_map<std::string, float> data;
	DataRow(float timestamp, std::unordered_map<std::string, float> data) {
		this->timestamp = timestamp;
		this->data = data;
	}
	DataRow() {

	}
};