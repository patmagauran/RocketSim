#pragma once
#include <string>
#include "concurrent/readerwriterqueue.h"
#include <unordered_map>
#include <thread>
#include "DataRow.h"
#include <map>
#include "PlotDataContainer.h"
#include "PlotUI.h"
class DataLog
{


	inline static bool initialized = false;

	inline static DataLog * datalogInstance = nullptr;

	inline static PlotUI * plotUIInstance = nullptr;

	inline static std::thread dataThread = std::thread();

	static void initialize();


	static void startDataThread();

	DataLog();

	void run();

	void drawPlots();

	inline static moodycamel::ReaderWriterQueue<DataRow> dataQueue = moodycamel::ReaderWriterQueue<DataRow>();

	inline static std::unordered_map<std::string, double> currentData = std::unordered_map<std::string, double>();

	inline static PlotDataContainer plotData = PlotDataContainer();

public:
	static void logData(std::string name, double value);
	static void pushTimestamp(double timestamp);
	static void cleanup();

};

