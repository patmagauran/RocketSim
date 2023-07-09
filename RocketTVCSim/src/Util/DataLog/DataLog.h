#pragma once
#include <string>
#include "concurrent/readerwriterqueue.h"
#include <unordered_map>
#include <thread>
#include "DataRow.h"
#include <map>
#include "PlotDataContainer.h"
#include "CSVLogger.h"
#include "PlotUI.h"
#include <iostream>
#include <fstream>
enum class EventType {
	MESSAGE,
	STAGE,
	DONE,
	PAUSE
};
class DataLog
{


	inline static bool initialized = false;

	inline static DataLog * datalogInstance = nullptr;

	inline static PlotUI * plotUIInstance = nullptr;
	inline static CSVLogger * csvLoggerInstance = nullptr;

	inline static std::thread dataThread = std::thread();



	static void startDataThread();

	DataLog();

	void run();

	inline static moodycamel::ReaderWriterQueue<DataRow> dataQueue = moodycamel::ReaderWriterQueue<DataRow>();

	inline static std::unordered_map<std::string, double> currentData = std::unordered_map<std::string, double>();

	inline static PlotDataContainer plotData = PlotDataContainer();

public:
	static void initialize(std::string filename);
	static void logData(std::string name, double value);
	static void pushTimestamp(double timestamp);
	static void pushEvent(EventType eventType, std::string message); //TODO: Implement

	static void cleanup();

};

