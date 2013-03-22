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

#ifndef NTK_UTILS_PROGRESSIVE_H
#define NTK_UTILS_PROGRESSIVE_H

#include <ntk/core.h>

#include <QObject>

namespace ntk
{

class ProgressiveImpl : public QObject
{
Q_OBJECT
public:
    //! Send a progress message, percent and optionally give an identifier.
    virtual void progress(const char* message, float percent, const char* id = "unknown") const;

signals:
    void progressChanged(QString, float, QString) const;
};

/*!
 * Objects whose progress can be followed. This class does not inherit directly
 * from QObject to avoid multiple inheritance issues with QObject.
 */
class Progressive
{
public:
    ProgressiveImpl* progressObject() const { return &impl; }

    virtual void progress(const char* message, float percent, const char* id = "unknown") const
    {
        progressObject()->progress(message, percent, id);
    }

private:
    mutable ProgressiveImpl impl;
};

}  // ntk

#endif // NTK_UTILS_PROGRESSIVE_H
