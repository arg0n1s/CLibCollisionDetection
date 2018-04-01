#include "..\include\CLibErrorLogger.h"
#include <ctime>
#include <fstream>
#include <windows.h>
#include <iostream>

namespace clib
{
	using std::ofstream;
	using std::stringstream;

	ErrorLogger::ErrorLogger() {
		errorLogged = false;
		initLogFilePath();
		initLogFile();
	}

	ErrorLogger::~ErrorLogger() {
		finishLogging();
		if (errorLogged) {
			std::cout << "Program logged some errors. For more information refer to the error log: ";
			std::cout << filePath << std::endl;
		}
		else {
			std::cout << "Program finished without errors.";
		}
	}

	void ErrorLogger::initLogFile() {
		auto errorLog = stringstream();
		time_t now = time(0);
		char buff[128];
		ctime_s(buff, sizeof buff, &now);
		errorLog << "Logging startet at: " << string(buff) << "\n \n";
		errorLog << "****** Errors start here: ******** \n \n";


		ofstream logFile(filePath);
		logFile.close();

		saveStringToFile(errorLog.str());
	}

	ErrorLogger& ErrorLogger::instance () {
		static ErrorLogger logger;
		return logger;
	}

	void ErrorLogger::saveStringToFile(const string& msg) {
		ofstream ofs;
		ofs.open(filePath, std::ofstream::out | std::ofstream::app);
		ofs << msg;
		ofs.close();
	}

	void ErrorLogger::appendErrorMsg(const string& errorMsg) {
		time_t now = time(0);
		char buff[128];
		ctime_s(buff, sizeof buff, &now);
		auto errorLog = stringstream();
		errorLog << "*** An error occured! - Time stamp: " << string(buff) << "\n";
	    errorLog << "Error message was: \n" << errorMsg << "\n *** \n";
		saveStringToFile(errorLog.str());
		errorLogged = true;
	}

	void ErrorLogger::changeLogFileFolder(const string& folder) {
		auto logFilePath = stringstream();
		time_t now = time(0);
		logFilePath << folder << "\\" << now << ".txt";
		filePath = logFilePath.str();
		initLogFile();
	}

	void ErrorLogger::initLogFilePath() {
		WCHAR exePath[MAX_PATH];
		if (!GetModuleFileName(NULL, exePath, MAX_PATH)) throw std::runtime_error("Log file could not be initialized *.exe path not found!");
		std::wstring p(exePath);
		int size_needed = WideCharToMultiByte(CP_UTF8, 0, &p[0], (int)p.size(), NULL, 0, NULL, NULL);
		std::string strTo(size_needed, 0);
		WideCharToMultiByte(CP_UTF8, 0, &p[0], (int)p.size(), &strTo[0], size_needed, NULL, NULL);

		size_t i = strTo.find_last_of("/\\");
		string folder = strTo.substr(0, i+1);

		time_t now = time(0);

		auto logFilePath = stringstream();
		logFilePath << folder <<"logs\\"<< now << ".txt";
		filePath = logFilePath.str();
	}

	void ErrorLogger::finishLogging() {
		static bool isSaved = false;
		if (isSaved) return;
		time_t now = time(0);
		char buff[128];
		ctime_s(buff, sizeof buff, &now);
		auto errorLog = stringstream();
		errorLog << "****** Errors end here! ******** \n \n \n";
		errorLog << "Logging ended at: " << string(buff) << "\n \n";
		saveStringToFile(errorLog.str());
		isSaved = true;
	}
}
	