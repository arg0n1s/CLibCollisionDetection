#pragma once
#include <string>
#include <memory>
#include <sstream>

namespace clib {
	// Aliases for used namespaces
	using std::string;
	using std::stringstream;
	using std::unique_ptr;

	/**
		\brief This class provides error logging functionality and should be used when catching
		Exceptions or malformed user input of any kind.
		Error messages are written to files with timestamps attached to them.
		The output path is in the "logs"-folder within the same directory as
		the executable of using the CLibCollisionDetection-library.
	*/
	class ErrorLogger {
	private:

		/* Path to the current log file. */
		string filePath;

		/* Path to the current log file folder. */
		string logFolder;

		bool errorLogged;

		/**
			\brief Initiates a new error log upon construction with the current time in µSeconds as file name.
		*/
		ErrorLogger();

		/**
			\brief Initialize logger
		*/
		void initLogFile();

		/**
			\brief Helper method to get the path of the executable using the CLibCollisionDetection-library.
			\returns Directory of the executable using the CLibCollisionDetection-library.
		*/
		void initLogFilePath();

		/**
			\brief Call to save a formatted string to file.
		*/
		void saveStringToFile(const string& msg);

		/**
			\brief Call to save the current log to file and mark end of logging.
		*/
		void finishLogging();

	public:

		~ErrorLogger();

		/**
			\brief Call to get the singleton of this class.
			\returns Returns a reference to the error logger singleton.
			\note This is the only way to "make" an ErrorLogger-object, i.e. Constructor is private!
		*/
		static ErrorLogger& instance();

		/**
			\brief Add a new error message to the log with the current time stamp.
			\param[in] errorMsg New error message that should be appended
		*/
		void appendErrorMsg(const string& errorMsg);

		/**
			\brief Change output folder for the log file and reinit.
			\note Default: logs-folder within the folder of the executable using the CLibCollisionDetection library
			\param[in] folder new log file folder
		*/
		void changeLogFileFolder(const string& folder);

		/**
			\brief Converts a ascii/ansi/utf8 string to a wide string (windows stuff..)
			\param[in] s standard c++ string
			\returns wide string
		*/
		static std::wstring s2ws(const string& s);

		/**
			\brief Converts a wide string (windows stuff..) to a ascii/ansi/utf8 string.
			\param[in] ws wide string
			\returns standard c++ string
		*/
		static string ws2s(const std::wstring& ws);

	};

/* Use this macro to log any string-typed output from errors, exceptions, etc.*/
#define LOG_ERROR(X) (ErrorLogger::instance().appendErrorMsg(X))
}


