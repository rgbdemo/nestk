#ifndef   	NTK_UTILS_DEBUG_H_
# define   	NTK_UTILS_DEBUG_H_

# include <ntk/core.h>

# include <QString>
# include <QTextStream>
# include <QDebug>

namespace ntk
{
  extern int ntk_debug_level;
  extern int ntk_log_level;

  void setLogFileName (const std::string& logfile);
  std::string getLogFileName ();

  class XmlSerializable;
}

class QStringList;

class NtkDebug
{
  public:
    virtual ~NtkDebug();
    // { qDebug() << "[DBG]" << s; }

  public:
    void print(const ntk::XmlSerializable& rhs) const;

  public:
    QString* stringPtr() const { return &s; }

  private:
    mutable QString s;
};

#define NTK_DECLARE_DEBUG_OPERATOR(Type) \
inline const NtkDebug& operator<<(const NtkDebug& d, Type rhs) \
{ QTextStream stream(d.stringPtr()); stream << rhs; return d; }

NTK_DECLARE_DEBUG_OPERATOR(bool)
NTK_DECLARE_DEBUG_OPERATOR(short)
NTK_DECLARE_DEBUG_OPERATOR(unsigned short)
NTK_DECLARE_DEBUG_OPERATOR(int)
NTK_DECLARE_DEBUG_OPERATOR(unsigned)
NTK_DECLARE_DEBUG_OPERATOR(long)
NTK_DECLARE_DEBUG_OPERATOR(unsigned long)
NTK_DECLARE_DEBUG_OPERATOR(long long)
NTK_DECLARE_DEBUG_OPERATOR(unsigned long long)
NTK_DECLARE_DEBUG_OPERATOR(double)
NTK_DECLARE_DEBUG_OPERATOR(const QString&)
NTK_DECLARE_DEBUG_OPERATOR(const char*)

const NtkDebug& operator<<(const NtkDebug& d, const std::string& rhs);

const NtkDebug& operator<<(const NtkDebug& d, const ntk::XmlSerializable& rhs);

const NtkDebug& operator<<(const NtkDebug& d, const QStringList& rhs);

#ifndef NDEBUG
# define ntk_dbg(level) if (level <= ntk::ntk_debug_level) NtkDebug()
#else
# define ntk_dbg(level) if (0) NtkDebug()
#endif // ndef NDEBUG

# define ntk_log() NtkDebug()
# define ntk_log_error() NtkDebug() << "Error: "

#ifndef NDEBUG
# define ntk_dbg_print(value, level) \
  ntk_dbg(level) << #value": " << value;
#else
# define ntk_dbg_print(value, level)
#endif // ndef NDEBUG

namespace ntk {

  void assert_failure(const char* where, const char* what, const char* cond);
  void fatal_error(const char* what, int code=1);

}

#ifdef __GNUC__
# define PRETTY_FUNCTION __PRETTY_FUNCTION__
#else
# define PRETTY_FUNCTION ""
#endif // __GNUC__

#ifndef NDEBUG
# define ntk_assert(cond,what) if (!(cond)) ntk::assert_failure(PRETTY_FUNCTION, what, #cond);
#else
# define ntk_assert(cond,what)
#endif // ndef NDEBUG

#define ntk_ensure(cond,what) if (!(cond)) { ntk::assert_failure(PRETTY_FUNCTION, what, #cond); \
                                             ntk::fatal_error(what); }

#define ntk_dbg_enter_function(level) \
  ntk_dbg(level) << "<Entering> " << PRETTY_FUNCTION \
                 << " (" << (void*)this << ")" \
                 << " (" << cv::getThreadNum() << ")";

#define ntk_dbg_leave_function(level) \
  ntk_dbg(level) << "</Leaving> " << PRETTY_FUNCTION \
                 << " (" << this << ")" \
                 << " (" << cv::getThreadNum() << ")";

namespace ntk
{

void print_log (const int level, const char* prefix, const char* fmt, ...);

} // ntk

#define   ntk_error(...) ntk::print_log(0,   "ERROR: ", __VA_ARGS__)
#define    ntk_warn(...) ntk::print_log(1, "WARNING: ", __VA_ARGS__)
#define    ntk_info(...) ntk::print_log(2,    "INFO: ", __VA_ARGS__)
#define ntk_verbose(...) ntk::print_log(3, "VERBOSE: ", __VA_ARGS__)

#endif 	    /* !NTK_UTILS_DEBUG_H_ */
