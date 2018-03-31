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

		/* String stream object used to store error messages. */
		stringstream errorLog;

		/**
			\brief Initiates a new error log upon construction with the current time in µSeconds as file name.
		*/
		ErrorLogger();

		/**
			\brief Helper method to get the path of the executable using the CLibCollisionDetection-library.
			\returns Directory of the executable using the CLibCollisionDetection-library.
		*/
		const string getLogFilePath() const;

	public:

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
			\brief Get the current contained error messages as string
			\returns Current error log as a formatted string.
		*/
		const string toString() const;

		/**
			\brief Call to save the current log to file and mark end of logging.
		*/
		void saveToLogToFile();
	};

/* Use this macro to log any string-typed output from errors, exceptions, etc.*/
#define LOG_ERROR(X) (ErrorLogger::instance().appendErrorMsg(X))
}


