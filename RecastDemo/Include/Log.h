# ifndef LOG_SYS_H
# define LOG_SYS_H

# include <iostream>
# include <fstream>
# include <string>
# include <time.h>
# include <stdio.h>
# include <stdlib.h>

#define  PLATFORM_LINUX 0
#define  PLATFORM_WINDOWS 0
const int  MaxBufferSize = 1000;
using std::cout;
using std::string;
using std::endl;
using std::to_string;
using std::ios;

class Logger;

enum log_level
{
	debug, info, warning, error
};// 日志等级
enum log_target
{
	file, terminal, file_and_terminal
};// 日志输出目标


extern string currTime();
extern string BaseFileName(const string& FullName);
extern string BaseFunctionName(const string& FullName);


extern Logger g_log;
extern string LogPath;

#define  LogDebug( format, ...)  RecastLog(debug, "[(Time:%s) %s:%d %s]  " format, currTime().c_str(), __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define  LogInfo( format, ...)  RecastLog(info, "[(Time:%s) %s:%d %s]  " format, currTime().c_str(), __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define  LogWarn( format, ...)  RecastLog(warning, "[(Time:%s) %s:%d %s]  " format, currTime().c_str(), __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define  LogError( format, ...)  RecastLog(error, "[(Time:%s) %s:%d %s]  " format, currTime().c_str(), __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#define RecastLog(priority, format, ...)   \
do \
{ \
char MessageBuffer[MaxBufferSize]; \
int ret = snprintf(MessageBuffer, MaxBufferSize, format, __VA_ARGS__ ); \
string mes = MessageBuffer; \
g_log.output(mes, priority); \
 \
} while (0);




class Logger
{
public:
	Logger();  // 默认构造函数
	Logger(log_target target, log_level level, string path);
	~Logger();
	void Debug(string text);
	void Info(string text);
	void Warn(string text);
	void Error(string text);
	void output(string text, log_level act_level);            // 输出行为

private:
	std::ofstream outfile;    // 将日志输出到文件的流对象
	log_target target;        // 日志输出目标
	string path;              // 日志文件路径
	log_level level;          // 日志等级
};

# endif

