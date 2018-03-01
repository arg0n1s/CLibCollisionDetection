#pragma once
#include <string>
#include <memory>
#include <sstream>

namespace clib {
	using std::string;
	using std::stringstream;
	using std::unique_ptr;

	class ErrorLogger {
	private:
		stringstream errorLog;
		ErrorLogger();
		const string getLogFilePath() const;

	public:
		static ErrorLogger& instance();
		void appendErrorMsg(const string& errorMsg);
		const string toString() const;
		void saveToLogToFile();
	};

#define LOG_ERROR(X) (ErrorLogger::instance().appendErrorMsg(X))
}


