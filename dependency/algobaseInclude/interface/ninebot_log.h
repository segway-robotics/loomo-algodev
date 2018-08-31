/*! define the header for AprAlgo log
 * Filename: ninebot_log.h
 * Version: 0.60
 * Author(s): Jianzhu Huai, Yuan Zhang, Jianming Li
 * DO NOT CHANGE THIS FILE unless discussed with author
 * Algo team, Ninebot Inc., 2018
 */

#ifndef NINEBOT_LOG_NINEBOT_LOG_H
#define NINEBOT_LOG_NINEBOT_LOG_H

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <atomic>
#include <string>
#include <thread>

#if defined(_MSC_VER)
#define NINEBOT_LOG_EXPORT __declspec(dllexport)
#else /* compiler doesn't support __declspec() */
#define NINEBOT_LOG_EXPORT __attribute__ ((visibility("default")))
#endif

/*
 * log priority values, in ascending priority order.
 * This definition is not enclosed in ninebot_algo namespace in order to (1) agree with android/log.h
 * where no namespace wraps android_LogPriority, and (2) to avoid putting the namespace name in
 * macros like ALOGD
 */

/***
#define PRAGMA_STR1(x)  #x
#define PRAGMA_STR2(x)  PRAGMA_STR1 (x)
#pragma message(__FILE__ "(" PRAGMA_STR2(__LINE__) ") windows " PRAGMA_STR2(_WIN32) \
        " unix " PRAGMA_STR2(__unix__) " apple " PRAGMA_STR2(__APPLE__) " mach " PRAGMA_STR2(__MACH__) \
        " android " PRAGMA_STR2(ANDROID) " __android__ " PRAGMA_STR2(__ANDROID__))
***/

#if defined(ANDROID) || defined(__ANDROID__)
#include <android/log.h>
typedef android_LogPriority LEVEL_TYPE;
#else
typedef enum LEVEL_TYPE
{
	ANDROID_LOG_UNKNOWN = 0,
    ANDROID_LOG_DEFAULT,    /* only for SetMinPriority() */
    ANDROID_LOG_VERBOSE,
    ANDROID_LOG_DEBUG,
    ANDROID_LOG_INFO,
    ANDROID_LOG_WARN,
    ANDROID_LOG_ERROR,
    ANDROID_LOG_FATAL,
    ANDROID_LOG_SILENT, /* only for SetMinPriority(); must be last */
}LEVEL_TYPE;
#endif

/**********************************************************************
 How to configure log level for the project that uses ninebot_log.h?
 (1) To begin with, create a setting xml file, (2) push that file to the device path specified at
 LOG_CONFIG_FILE.
 The following is the content of a sample setting xml file:

<root>
    <LEVEL>3</LEVEL>
</root>

 Set level >=8 or <=0 to disable logging, set it larger numbers to log only higher level messages.
 E.g., set level as 3 to log for both debug and warn messages, set ot as 5 to only log warnings.
 The corresponding enumerate log levels for these numbers are given below.

 Optionally, an output file argument, LOG_FILENAME, can be provided in the xml file as below. It
 specifies the destination file to sink the log stream. If not specified, the log stream only goes
 to the monitor.

 <root>
     <!-- This is a sample comment -->
     <LEVEL>3</LEVEL>
     <LOG_FILENAME>/sdcard/data/log.txt</LOG_FILENAME>
 </root>

  Optionally, an log tag filter argument, TAG_FILTER, can be provided in the xml file as below. It
 specifies the tag filter so that only logs with the tag is printed (ANDROID_LOG_WARN, ANDROID_LOG_ERROR will not be filtered).
 If not specified, then all logs are printed.
 When using ALOGx, tag will not be provided by the user, and the default tag DEFAULT_LOG_TAG=AprAlgo is used.
 When using ALOGTAGx, tag is provided by the user specificially.

 In each application, you can also redefine DEFAULT_LOG_TAG, so that ALOGD can still be used with alternative log tag, e.g.,
 	#undef DEFAULT_LOG_TAG
 	#define DEFAULT_LOG_TAG "LocalMapping"

 <root>
     <!-- This is a sample comment -->
     <LEVEL>3</LEVEL>
     <TAG_FILTER>AprAlgo</TAG_FILTER>
 </root>

 (2) Once you create such a file, say, logAprAlgo.xml, push it to a robot device via the commands
 (suppose the path given by LOG_CONFIG_FILE is  "/sdcard/logAprAlgo.xml")

 adb push logAprAlgo.xml /sdcard

Now you can throttle the verbosity by varying the level value in the specified xml file.

 In this log module, the engine for writing to a log file is unbuffered file
   stream by fprintf which is unamimously supported by major OSs.
 Since then, it has come to my notice that the UNIX system call, mmap, may
   boost efficiency for logging to files as evidenced by log4a(https://github.com/pqpo/Log4a) and wechat xlog.
 However, further reading shows that mmap may not be so impressive. see
   https://stackoverflow.com/questions/35891525/mmap-for-writing-sequential-log-file-for-speed/35891885
 There are also good C++ logging libraries, see https://kjellkod.wordpress.com/2015/06/30/the-worlds-fastest-logger-vs-g3log/
   but for now I have not looked closely at them.

***********************************************************/
namespace ninebot_algo {

#define DEFAULT_LOG_TAG		"AprAlgo"


#define ALOGD(FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_DEBUG, DEFAULT_LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)
#define ALOGI(FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_INFO, DEFAULT_LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)
#define ALOGW(FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_WARN, DEFAULT_LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)
#define ALOGE(FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_ERROR, DEFAULT_LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)

#define ALOGOLDI(...) __android_log_print (ANDROID_LOG_INFO, DEFAULT_LOG_TAG, __VA_ARGS__)
#define ALOGOLDD(...) __android_log_print (ANDROID_LOG_DEBUG, DEFAULT_LOG_TAG, __VA_ARGS__)
#define ALOGOLDE(...) __android_log_print (ANDROID_LOG_ERROR, DEFAULT_LOG_TAG, __VA_ARGS__)
#define ALOGOLDW(...) __android_log_print (ANDROID_LOG_WARN, DEFAULT_LOG_TAG, __VA_ARGS__)

// the macros from the twp below groups are identical in effect
#define ALOGTAGD(LOG_TAG, FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_DEBUG, LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)
#define ALOGTAGI(LOG_TAG, FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_INFO, LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)
#define ALOGTAGW(LOG_TAG, FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_WARN, LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)
#define ALOGTAGE(LOG_TAG, FMT, ...) AprAlgoLog::instance_.WriteLog(ANDROID_LOG_ERROR, LOG_TAG, "%s:%d " FMT, __FILE__, __LINE__, ##__VA_ARGS__)

#define ALOGTAGDT(LOG_TAG, ...) AprAlgoLog::instance_.WriteLog2(ANDROID_LOG_DEBUG, LOG_TAG, __FILE__, __LINE__, __VA_ARGS__)
#define ALOGTAGIT(LOG_TAG, ...) AprAlgoLog::instance_.WriteLog2(ANDROID_LOG_INFO, LOG_TAG, __FILE__, __LINE__, __VA_ARGS__)
#define ALOGTAGWT(LOG_TAG, ...) AprAlgoLog::instance_.WriteLog2(ANDROID_LOG_WARN, LOG_TAG, __FILE__, __LINE__, __VA_ARGS__)
#define ALOGTAGET(LOG_TAG, ...) AprAlgoLog::instance_.WriteLog2(ANDROID_LOG_ERROR, LOG_TAG, __FILE__, __LINE__, __VA_ARGS__)

#define ALOG AprAlgoLog::GetInstance()->WriteLog

NINEBOT_LOG_EXPORT std::string GetStdoutFromCommand(std::string cmd);

class NINEBOT_LOG_EXPORT AprAlgoLog {
public:
	static AprAlgoLog instance_;

	~AprAlgoLog();
	void WriteLog(LEVEL_TYPE level, const char *log_tag,
				  const char *format, ...);
	void WriteLog2(LEVEL_TYPE level, const char *log_tag, const char *file,
				   const int lineNumber, const char *format, ...);
private: 
    AprAlgoLog();
    AprAlgoLog(const AprAlgoLog& other);
    AprAlgoLog & operator=(const AprAlgoLog & other);
    
    void PeekLogSettings();

    LEVEL_TYPE m_iLevel;
    FILE *m_file; // logs file
    char *m_logtag; // log tag filter
    std::thread m_loglevelmonitor; // a thread every so often peeks the log setting file to update m_iLevel
    std::atomic_bool m_cancel;
    static int instanceCount;
    const static int peekIntervalSeconds;
};
} // NINEBOT_LOG_NINEBOT_LOG_H
#endif
