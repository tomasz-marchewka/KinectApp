#pragma once
#include <ctime>
#include <stdio.h>
#include <QString>
#include <qthread.h>

class Logger : public QThread
{
	Q_OBJECT
public:
	static Logger& getInstance()
	{
		static Logger instance;
		return instance;
	}
	void log(const char* message)
	{
		log(QString(message));
	}

	void log(QString message)
	{
		time_t t = time(0);
		struct tm *now = localtime(&t);
		char timeFormat[21];
		sprintf(timeFormat, "%04d-%02d-%02d %02d:%02d:%02d ", now->tm_year + 1900, now->tm_mon, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
		message.prepend(timeFormat);
		emit(logMessage(message));
	}

signals:
	void logMessage(QString message);
private:
	Logger() {}
	Logger(Logger const&);
	void operator=(Logger const&);
};

