#include "..\include\CLibErrorLogger.h"
#include <ctime>
#include <fstream>
#include <windows.h>

namespace clib
{
	using std::ofstream;

	ErrorLogger::ErrorLogger() {
		errorLog = std::stringstream();
		time_t now = time(0);
		char buff[128];
		ctime_s(buff, sizeof buff, &now);
		errorLog << "Logging startet at: " << string(buff) <<"\n \n";
		errorLog << "****** Errors start here: ******** \n \n";
	}

	ErrorLogger& ErrorLogger::instance () {
		static ErrorLogger logger;
		return logger;
	}

	void ErrorLogger::appendErrorMsg(const string& errorMsg) {
		time_t now = time(0);
		char buff[128];
		ctime_s(buff, sizeof buff, &now);
		errorLog << "*** An error occured! - Time stamp: " << string(buff) << "\n";
	    errorLog << "Error message was: \n" << errorMsg << "\n *** \n";
	}
	const string ErrorLogger::toString() const {
		return errorLog.str();
	}
	const string ErrorLogger::getLogFilePath() const {
		WCHAR exePath[MAX_PATH];
		if (!GetModuleFileName(NULL, exePath, MAX_PATH)) throw std::runtime_error("Log file could not be saved *.exe path not found!");
		std::wstring p(exePath);
		int size_needed = WideCharToMultiByte(CP_UTF8, 0, &p[0], (int)p.size(), NULL, 0, NULL, NULL);
		std::string strTo(size_needed, 0);
		WideCharToMultiByte(CP_UTF8, 0, &p[0], (int)p.size(), &strTo[0], size_needed, NULL, NULL);

		size_t i = strTo.find_last_of("/\\");
		string folder = strTo.substr(0, i+1);

		time_t now = time(0);

		stringstream logFilePath;
		logFilePath << folder <<"logs\\"<< now << ".txt";
		return logFilePath.str();
	}

	void ErrorLogger::saveToLogToFile() {
		static bool isSaved = false;
		if (isSaved) return;
		time_t now = time(0);
		char buff[128];
		ctime_s(buff, sizeof buff, &now);
		errorLog << "****** Errors end here! ******** \n \n \n";
		errorLog << "Logging ended at: " << string(buff) << "\n \n";
		string path = getLogFilePath();
		ofstream logFile(path);
		logFile << toString();
		logFile.close();
		isSaved = true;
	}
}
	