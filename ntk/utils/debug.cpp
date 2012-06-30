/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "debug.h"
#include "xml_serializable.h"

#include <iostream>
#include <QStringList>

#include <QtGlobal>
#include <QMutex>
#include <QMutexLocker>

typedef const char* CString;

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
    std::cout << prefix << message << std::endl;
}

void printErrorLine (CString prefix, CString message)
{
    printWindowsDebugOutputLine(prefix, message);
    std::cerr << prefix << message << std::endl;
}

void printLogLine (CString prefix, CString message)
{
    printWindowsDebugOutputLine(prefix, message);
    std::clog << prefix << message << std::endl;
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

void handleMsg (QtMsgType type, CString msg)
{
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
        qInstallMsgHandler(handleMsg);
    }

    static void quit ()
    {
        qInstallMsgHandler(0);
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
