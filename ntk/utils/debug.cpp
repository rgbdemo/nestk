
#include "debug.h"
#include "xml_serializable.h"

#include <iostream>
#include <QStringList>

#include <QtGlobal>
#include <QMutex>
#include <QMutexLocker>
#include <QFile>

#include <cstdio>

#include <iostream>

typedef const char* CString;

namespace ntk
{

QString logFileName;
QMutex logFileLock;
bool hasLogFileName = false;
FILE* logFileHandle = 0;

void setLogFileName (const std::string& logfile)
{
    QMutexLocker _ (&logFileLock);
    if (logFileHandle)
    {
        fclose (logFileHandle);
        logFileHandle = 0;
    }

    logFileName = QString::fromStdString(logfile);
    if (logfile.empty())
    {
        hasLogFileName = false;
    }
    else
    {
        logFileHandle = fopen (logFileName.toLatin1(), "a");
        hasLogFileName = true;
    }
}

std::string getLogFileName ()
{
    QMutexLocker _ (&logFileLock);
    return logFileName.toStdString();
}

}

#if _WIN32
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRA_LEAN
#include <windows.h>
void printWindowsDebugOutputLine (CString prefix, CString message)
{
    std::string line(prefix);
    line += message;
    line += "\r\n";
    OutputDebugString(line.c_str());
}

void printStandardLine (CString prefix, CString message)
{
    printWindowsDebugOutputLine(prefix, message);
    // std::cout << prefix << message << std::endl;
}

void printErrorLine (CString prefix, CString message)
{
    printWindowsDebugOutputLine(prefix, message);
    // std::cerr << prefix << message << std::endl;
}

void printLogLine (CString prefix, CString message)
{
    printWindowsDebugOutputLine(prefix, message);
    // std::clog << prefix << message << std::endl;
}
#else
#include <iostream>
void printStandardLine (CString prefix, CString message)
{
    std::cout << prefix << message << std::endl;
}
void printErrorLine (CString prefix, CString message)
{
    std::cerr << prefix << message << std::endl;
}
void printLogLine (CString prefix, CString message)
{
    std::clog << prefix << message << std::endl;
}
#endif

#if QT_VERSION < 0x050000
void handleMsg (QtMsgType type, CString msg)
{
#else
void handleMsg (QtMsgType type, const QMessageLogContext& ctx, const QString& str)
{
    const QByteArray utf8 = str.toUtf8();
    const CString msg = utf8.constData();
    // FIXME: Log context.
#endif
    switch (type)
    {
        case QtDebugMsg:
            printLogLine("Debug: ", msg);
            break;
        case QtWarningMsg:
            printErrorLine("Warning: ", msg);
            break;
        case QtCriticalMsg:
            printErrorLine("Critical: ", msg);
            break;
        case QtFatalMsg:
            printStandardLine("Fatal: ", msg);
            abort();
    }
}

struct MsgHandler
{
    static void init ()
    {
#if QT_VERSION < 0x050000
        qInstallMsgHandler(handleMsg);
#else
        qInstallMessageHandler(handleMsg);
#endif
    }

    static void quit ()
    {
#if QT_VERSION < 0x050000
        qInstallMsgHandler(0);
#else
        qInstallMessageHandler(0);
#endif
    }


    MsgHandler ()
    {
        init();
    }

    ~MsgHandler ()
    {
        quit();
    }

    static QMutex mutex;

    static void use ()
    {
        const QMutexLocker _(&mutex);

        static const MsgHandler instance;
    }
};

QMutex MsgHandler::mutex;

namespace ntk
{
  int ntk_debug_level = 0;
  int ntk_log_level = 2;

  extern QTextStream qErr;
  extern QTextStream qOut;

  void assert_failure(const char* where, const char* what, const char* cond)
  {
    std::cerr << "ASSERT failure in " << where << ": " << what
        << " [" << cond << "]" << std::endl;
    abort();
  }

  void fatal_error(const char* what, int code)
  {
    std::cerr << "FATAL failure: " << what << std::endl;
    exit(code);
  }

}

const NtkDebug& operator<<(const NtkDebug& d, const std::string& rhs)
{
  d << rhs.c_str();
  return d;
}

const NtkDebug& operator<<(const NtkDebug& d, const ntk::XmlSerializable& rhs)
{
  ntk::XMLNode e = ntk::XMLNode::createXMLTopNode("debug");
  rhs.fillXmlElement(e);
  d.stringPtr()->append(e.createXMLString(true));
  return d;
}

const NtkDebug& operator<<(const NtkDebug& d, const QStringList& rhs)
{
        QStringList::ConstIterator i   = rhs.begin();
  const QStringList::ConstIterator end = rhs.end();

  if (i == end)
    return d;

  d << *i++;

  for (; i != end; ++i)
    d << ", " << *i;

  return d;
}

NtkDebug :: ~NtkDebug()
{
    MsgHandler::use();

    qDebug() << "[DBG]" << s;
}

namespace ntk
{

void print_log (const int level, const char* prefix, const char* fmt, ...)
{
    if (level > ntk::ntk_log_level)
        return;

    ntk::logFileLock.lock();
    FILE* handle;

    if (0 != ntk::logFileHandle)
    {
        handle = ntk::logFileHandle;
    }
    else
    {
        ntk::logFileLock.unlock();
        if (level == 0)
            handle = stderr;
        else
            handle = stdout;
    }

    fprintf (handle, prefix);
    va_list args;
    va_start(args, fmt);
    vfprintf (handle, fmt, args);
    va_end (args);

    fflush (handle);

    logFileLock.unlock();
}

}
