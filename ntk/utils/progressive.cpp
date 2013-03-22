
#include "progressive.h"
#include "debug.h"

void ntk::ProgressiveImpl::progress(const char* message, float percent, const char* id) const
{
    emit progressChanged(QString(message), percent, id);
}
