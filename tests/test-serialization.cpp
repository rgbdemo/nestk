
#include <ntk/core.h>
#include <ntk/utils/serializable.h>
#include <ntk/utils/debug.h>

#include <QTextStream>
#include <QDataStream>
#include <QBuffer>
#include <QDebug>

struct Foo
{
  int a, b;
};

template <class StreamType>
StreamType &	operator<< (StreamType& stream, const Foo& f)
{
  stream << f.a << ntk::sep() << f.b;
  return stream;
}

template <class StreamType>
StreamType &	operator>> (StreamType& stream, Foo& f)
{
  stream >> f.a >> f.b;
  return stream;
}

QDebug operator << (QDebug stream, const Foo& f)
{
  stream << f.a << f.b;
  return stream;
}

int main()
{
  Foo f; f.a = 42; f.b = 51;
  QString s;

  {
    QTextStream textstream(&s);
    textstream << f;
    Foo g;
    textstream >> g;
    ntk_assert(g.a == 42 && g.b == 51, "Bad textstream.");
    qDebug() << g;
  }

  QBuffer buffer; buffer.open(QIODevice::ReadWrite);
  {
    QDataStream datastream(&buffer);
    datastream << f;
  }
  qDebug() << buffer.data();
  buffer.seek(0);

  {
    QDataStream datastream(&buffer);
    Foo g;
    datastream >> g;
    ntk_assert(g.a == 42 && g.b == 51, "Bad datastream.");
    qDebug() << g;
  }

  return 0;
}
