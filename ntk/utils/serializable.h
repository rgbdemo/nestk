/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#ifndef NTK_utils_serializable_H
# define NTK_utils_serializable_H

# include <ntk/core.h>
# include <ntk/thread/utils.h>
# include <ntk/utils/debug.h>

# include <list>
# include <vector>

# include <QStringList>
# include <QString>
# include <QTextStream>
# include <QDataStream>
# include <QFileInfo>
# include <QDir>

namespace ntk
{

class SerializingDevice
{
public:
    enum DeviceType { NONE, DATASTREAM, BYTEARRAY, FILEINFO, DIRECTORY};

public:
    SerializingDevice (QDataStream& stream)
        : stream (&stream)
        , byte_array (0)
        , file_info ()
        , dir ()
        , device_type (DATASTREAM)
    {}

    SerializingDevice (QByteArray& array)
        : stream (0)
        , byte_array (&array)
        , file_info ()
        , dir ()
        , device_type (BYTEARRAY)
    {}

    SerializingDevice (const QFileInfo& file_info)
        : stream (0)
        , byte_array (0)
        , file_info (file_info)
        , dir ()
        , device_type (FILEINFO)
    {}

    SerializingDevice (const QDir& dir)
        : stream (0)
        , byte_array (0)
        , file_info ()
        , dir (dir)
        , device_type (DIRECTORY)
    {}

    SerializingDevice ()
        : stream (0)
        , byte_array (0)
        , file_info ()
        , dir ()
        , device_type (NONE)
    {}

public:
    QDataStream* stream;
    QByteArray* byte_array;
    QFileInfo file_info;
    QDir dir;
    DeviceType device_type;
};

template <class T>
class Serializer
{
public:
    typedef T PtrT;

public:
    virtual PtrT read (const SerializingDevice& device) { return PtrT(); }

    virtual bool canReadFromDataStream () const { return false; }
    virtual bool     canReadFromString () const { return false; }
    virtual bool       canReadFromFile () const { return false; }
    virtual bool        canReadFromDir () const { return false; }

    virtual void write (const SerializingDevice& device, const PtrT value) { abort (); }

    virtual bool canWriteToDataStream () const { return false; }
    virtual bool     canWriteToString () const { return false; }
    virtual bool       canWriteToFile () const { return false; }
    virtual bool        canWriteToDir () const { return false; }
};

} // ntk

namespace ntk
{
  class XmlContext
  {
    public:
      virtual ~XmlContext() {}
  };


  struct QStreamSeparator {};
  inline QStreamSeparator sep() { return QStreamSeparator(); }

  template <class StreamType>
  StreamType &	operator<< (StreamType& stream, const QStreamSeparator& sep)
  { return stream; }

  inline QTextStream &	operator<< (QTextStream& stream, const QStreamSeparator& sep)
  { stream << " "; return stream; }

}

namespace ntk
{

class StringStream
{
  public:
    StringStream(const QString& s)
    { 
      QMutexLocker locker(&m_critical_section);
      m_data = s.split(QRegExp("\\s+"), QString::SkipEmptyParts);
      m_corrupted = false;
    }
    
    StringStream() : m_corrupted(false)
    {}
    
  public:
    QString getQString()
    {
      QMutexLocker locker(&m_critical_section);
      return m_data.join(" ");
    }
    
#define defineInputStringStreamInputOperator(Type, Method) \
    StringStream& operator>>(Type& value) \
    { \
      QMutexLocker locker(&m_critical_section); \
      if (!isExhausted()) \
      { \
        value = m_data.front()Method; \
        m_data.pop_front(); \
      } \
      else \
      { \
        m_corrupted = true; \
        ntk_throw_exception("Corrupted stream."); \
      } \
      return *this; \
    }
    defineInputStringStreamInputOperator(QString, )
    defineInputStringStreamInputOperator(bool, .toInt())
    
    defineInputStringStreamInputOperator(int, .toInt())
    defineInputStringStreamInputOperator(unsigned int, .toInt())
    
    defineInputStringStreamInputOperator(unsigned char, .toInt())
    defineInputStringStreamInputOperator(char, .toInt())

    defineInputStringStreamInputOperator(float, .toFloat())
    defineInputStringStreamInputOperator(double, .toDouble())
#undef defineInputStringStreamInputOperator
    
#define defineOutputStringStreamInputOperator(Type) \
    StringStream& operator<<(const Type& value) \
    { \
      QMutexLocker locker(&m_critical_section); \
      m_data.push_back(QString("%1").arg(value)); \
      return *this; \
    }
    defineOutputStringStreamInputOperator(QString)
    defineOutputStringStreamInputOperator(bool)
        
    defineOutputStringStreamInputOperator(int)
    defineOutputStringStreamInputOperator(unsigned int)
        
    defineOutputStringStreamInputOperator(char)
    defineOutputStringStreamInputOperator(unsigned char)
        
    defineOutputStringStreamInputOperator(float)
    defineOutputStringStreamInputOperator(double)
#undef defineOutputStringStreamInputOperator    
    
    bool isExhausted() const
    { 
      QMutexLocker locker(&m_critical_section);
      return m_data.size() == 0; 
    }

    bool isCorrupted() const
    {
      QMutexLocker locker(&m_critical_section);
      return m_corrupted;
    }
    
  private:
    mutable RecursiveQMutex m_critical_section;
    QStringList m_data;
    bool m_corrupted;
};

inline bool isStreamCorrupted(const QTextStream& stream)
{
  return stream.status() != QTextStream::Ok;
}

inline bool isStreamCorrupted(const QDataStream& stream)
{
  return stream.status() != QDataStream::Ok;
}

inline bool isStreamCorrupted(const StringStream& stream)
{
  return stream.isCorrupted();
}

template < typename StreamType, typename Type>
StreamType& operator>>(StreamType& input, std::vector<Type>& array)
{
  int size = 0;
  input >> size;
  array.resize(size);
  for (int i = 0; i < size; ++i)
  {
    if (isStreamCorrupted(input)) ntk_throw_exception("Corrupted stream.");
    input >> array[i];
  }
  return input;
};

template < typename StreamType, typename Type>
StreamType& operator<<(StreamType& output, const std::vector<Type>& array)
{
  output << (int)array.size() << ntk::sep();
  for (int i = 0; i < (int)array.size(); ++i)
    output << array[i] << ntk::sep();
  if (isStreamCorrupted(output))
    ntk_throw_exception("Corrupted stream");
  return output;
};

template < typename StreamType >
StreamType& operator>>(StreamType& input, std::vector<unsigned char>& array)
{
  int size = 0;
  input >> size;
  array.resize(size);
  for (int i = 0; i < size; ++i)
  {
    if (isStreamCorrupted(input)) ntk_throw_exception("Corrupted stream.");
    int k;
    input >> k;
    array[i] = k;
  }
  return input;
};

template < typename StreamType >
StreamType& operator<<(StreamType& output, const std::vector<unsigned char>& array)
{
  output << (int)array.size() << ntk::sep();
  for (int i = 0; i < (int)array.size(); ++i)
    output << (int)array[i] << ntk::sep();
  if (isStreamCorrupted(output))
    ntk_throw_exception("Corrupted stream");
  return output;
};

template < typename StreamType, typename Type>
StreamType& operator<<(StreamType& output, const std::vector<Type*>& array)
{
  output << (int)array.size() << ntk::sep();
  for (int i = 0; i < (int)array.size(); ++i)
    output << *array[i] << ntk::sep();
  if (isStreamCorrupted(output))
    ntk_throw_exception("Corrupted stream");
  return output;
};

template < typename StreamType, typename Type>
StreamType& operator>>(StreamType& input, std::vector<Type*>& array)
{
  int size = 0;
  input >> size;
  array.resize(size);
  for (int i = 0; i < size; ++i)
  {
    if (isStreamCorrupted(input)) ntk_throw_exception("Corrupted stream.");
    array[i] = new Type();
    input >> *array[i];
  }
  return input;
};

} // end of ntk

#endif // ndef NTK_utils_serializable_H
