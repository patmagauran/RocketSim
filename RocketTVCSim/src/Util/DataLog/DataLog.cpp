#include "DataLog.h"



void DataLog::initialize() {
	if (initialized) return;
	DataLog::initialized = true;
	dataThread = std::thread(DataLog::startDataThread);
	plotUIInstance = new PlotUI(plotData);
}
void DataLog::cleanup() {
	//dataThread.~thread();
}
void DataLog::startDataThread()
{
	datalogInstance = new DataLog();
	datalogInstance->run();
}
void DataLog::logData(std::string name, double value)
{
	initialize();
	currentData[name] = value;
}
void DataLog::pushTimestamp(double timestamp)
{
	initialize();
	DataRow row(timestamp, currentData);
	dataQueue.enqueue(row);
	currentData.clear();
}
DataLog::DataLog()
{
}
void DataLog::run() {
	while (true) {
		
		DataRow currentRow;

		if (dataQueue.try_dequeue(currentRow)) {
			for (const auto& [name, value] : currentRow.data) {
				plotData.putData(name, currentRow.timestamp, value);
			}
		}
	}
}



