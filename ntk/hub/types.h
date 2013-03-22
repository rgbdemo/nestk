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
 * Author: Nicolas Tisserand <nicolas.tisserand@manctl.com>
 */

#ifndef NTK_HUB_TYPES_H
# define NTK_HUB_TYPES_H

#include "ntk/mesh/meshfwd.h"
#include <QtGlobal>
#include <QMetaType>

class QString;
class QStringList;
class QImage;

namespace cv { class Mat; }

//------------------------------------------------------------------------------

// Types

namespace ntk { namespace hub {

typedef             qreal Real;
typedef           QString String;
typedef       QStringList Strings;
typedef            QImage Image;
typedef           cv::Mat Matrix;
typedef         ntk::Mesh Mesh;
typedef ntk::MeshConstPtr MeshConstPtr;

//------------------------------------------------------------------------------

// Traits

template < typename Type >
struct TypeTraits
{
    // const char* Name;
    // const char* name;
    // typedef Arg;
    // typedef Ret;
    // typedef Val;
};

#define HUB_TYPE_NAME(Type) TypeTraits<Type>::name
#define HUB_TYPE_ARG(Type)  TypeTraits<Type>::Arg
#define HUB_TYPE_RET(Type)  TypeTraits<Type>::Ret
#define HUB_TYPE_VAL(Type)  TypeTraits<Type>::Val

//------------------------------------------------------------------------------

// Preprocessor Registry

/* Usage:
 *
 * #define HUB_TYPE(Type, type, Arg, Ret, Val) SomethingWith##Type##type##Arg##Ret##Val
 *         HUB_TYPES()
 * #undef  HUB_TYPE
 */

#define HUB_TYPES()                                                                  \
        HUB_TYPE(Real   , real   , Real               , Real         , Real)         \
        HUB_TYPE(String , string , const String&      , String       , String)       \
        HUB_TYPE(Strings, strings, const Strings&     , Strings      , Strings)      \
        HUB_TYPE(Image  , image  , const Image&       , Image        , Image)        \
        HUB_TYPE(Mesh   , mesh   , const MeshConstPtr&, MeshConstPtr , MeshConstPtr)
//      HUB_TYPE(Matrix , matrix , const Matrix&      , Matrix       , Matrix)

//------------------------------------------------------------------------------

// Traits Registration

#define HUB_TYPE(Type, type, Arg_, Ret_, Val_) \
template <>                                    \
struct TypeTraits<Type>                        \
{                                              \
    static const char* Name;                   \
    static const char* name;                   \
    typedef Arg_ Arg;                          \
    typedef Ret_ Ret;                          \
    typedef Val_ Val;                          \
};
        HUB_TYPES()
#undef  HUB_TYPE

//------------------------------------------------------------------------------

// Runtime Registration

/*
 * Call registerTypes() before using any hub type in signals & slots.
 */

void registerTypes ();

} }

#endif // !NTK_HUB_TYPES_H
