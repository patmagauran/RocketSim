#include "DataLog.h"



void DataLog::initialize(std::string filename) {
	if (initialized) return;
	DataLog::initialized = true;
	dataThread = std::thread(DataLog::startDataThread);
	plotUIInstance = new PlotUI(plotData);
	csvLoggerInstance = new CSVLogger(filename);
	
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
	currentData[name] = value;
}
void DataLog::pushTimestamp(double timestamp)
{
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
			csvLoggerInstance->addRow(currentRow);
			for (const auto& [name, value] : currentRow.data) {
				plotData.putData(name, currentRow.timestamp, value);
			}
		}
	}
}



