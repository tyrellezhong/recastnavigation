#include "Log.h"
//string LogPath = "navmesh.log";
string LogPath = "C:/recastnavigation_z/Recast.log";
//Logger g_log;
Logger g_log(log_target::file_and_terminal, log_level::debug, LogPath);

Logger::Logger()
{
	// 默认构造函数
	path = "C:/recastnavigation/Recast.log";
	target = log_target::file_and_terminal;
	level = log_level::debug;
	cout << "[WELCOME] " << __FILE__ << " " << __DATE__ << __TIME__ <<" : " << "=== Start logging ===" << endl;
}

Logger::Logger(log_target target, log_level level, string path)
{
	this->target = target;
	this->path = path;
	this->level = level;
	string tmp = "";  // 双引号下的常量不能直接相加，所以用一个string类型做转换
	string welcome_dialog = tmp + "[Welcome] " + __FILE__ + " " + currTime() + " : " + "=== Start logging === " + __DATE__ + " "+__TIME__;
	if (target != log_target::terminal)
	{
		this->outfile.open(path, ios::out | ios::app);   // 打开输出文件
		this->outfile << welcome_dialog << std::endl;
		outfile.flush();
	}
	if (target != log_target::file)
	{
		// 如果日志对象不是仅文件
		cout << welcome_dialog;
	}
}

void Logger::Debug(string text)
{
	this->output(text, log_level::debug);
}

void Logger::output(string text, log_level act_level)
{
	string prefix;
	prefix += currTime();
	if (act_level == log_level::debug) prefix = "[DEBUG] ";
	else if (act_level == log_level::info) prefix = "[INFO] ";
	else if (act_level == log_level::warning) prefix = "[WARNING] ";
	else if (act_level == log_level::error) prefix = "[ERROR] ";
	else prefix = "";

	string output_content = prefix + text + "\n";
	if (this->level <= act_level && this->target != log_target::file)
	{
		// 当前等级设定的等级才会显示在终端，且不能是只文件模式
		cout << output_content;
	}
	if (this->target != log_target::terminal)
	{
		outfile << output_content;
		outfile.flush();
	}
}

void Logger::Info(string text)
{
	this->output(text, log_level::info);
}

void Logger::Warn(string text)
{
	this->output(text, log_level::warning);
}

void Logger::Error(string text)
{
	this->output(text, log_level::error);
}

extern string currTime()
{
	time_t timep;
	time(&timep);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
	return tmp;
}

extern string BaseFileName(const string& FullName)
{
	string FileName;
//#if PLATFORM_LINUX
//	FullName.Split(TEXT("/"), NULL, &FileName, ESearchCase::IgnoreCase, ESearchDir::FromEnd);
//#elif PLATFORM_WINDOWS
//	//FullName.Split(TEXT("/"), NULL, &FileName, ESearchCase::IgnoreCase, ESearchDir::FromEnd); //UE4.25
//	FullName.Split(TEXT("\\"), NULL, &FileName, ESearchCase::IgnoreCase, ESearchDir::FromEnd); //UE4.24
//#else
//	FullName.Split(TEXT("\\"), NULL, &FileName, ESearchCase::IgnoreCase, ESearchDir::FromEnd);
//#endif
//	return FileName;
	return FileName;
}

extern string BaseFunctionName(const string& FullName)
{
	string functionName;
	return functionName;
}
