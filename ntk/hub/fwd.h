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

// This file was intentionally written using some very wide text lines.
// Please keep it this way.

#ifndef NTK_HUB_FWD_H
# define NTK_HUB_FWD_H

// C Object
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Direct
#define FWD_C_OBJ_0(   Fwd, CObj, Ret, CFunction                                        ) Ret Fwd (                                                                )    { return CFunction(CObj                                    ); }
#define FWD_C_OBJ_1(   Fwd, CObj, Ret, CFunction, Arg0                                  ) Ret Fwd (Arg0 arg0                                                       )    { return CFunction(CObj, arg0                              ); }
#define FWD_C_OBJ_2(   Fwd, CObj, Ret, CFunction, Arg0, Arg1                            ) Ret Fwd (Arg0 arg0, Arg1 arg1                                            )    { return CFunction(CObj, arg0, arg1                        ); }
#define FWD_C_OBJ_3(   Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2                      ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2                                 )    { return CFunction(CObj, arg0, arg1, arg2                  ); }
#define FWD_C_OBJ_4(   Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2, Arg3                ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3                      )    { return CFunction(CObj, arg0, arg1, arg2, arg3            ); }
#define FWD_C_OBJ_5(   Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4          ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4           )    { return CFunction(CObj, arg0, arg1, arg2, arg3, arg4      ); }
#define FWD_C_OBJ_6(   Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5    ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5)    { return CFunction(CObj, arg0, arg1, arg2, arg3, arg4, arg5); }
#define FWD_C_OBJ_0_CV(Fwd, CObj, Ret, CFunction                                    , Cv) Ret Fwd (                                                                ) Cv { return CFunction(CObj                                    ); }
#define FWD_C_OBJ_1_CV(Fwd, CObj, Ret, CFunction, Arg0                              , Cv) Ret Fwd (Arg0 arg0                                                       ) Cv { return CFunction(CObj, arg0                              ); }
#define FWD_C_OBJ_2_CV(Fwd, CObj, Ret, CFunction, Arg0, Arg1                        , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1                                            ) Cv { return CFunction(CObj, arg0, arg1                        ); }
#define FWD_C_OBJ_3_CV(Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2                  , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2                                 ) Cv { return CFunction(CObj, arg0, arg1, arg2                  ); }
#define FWD_C_OBJ_4_CV(Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2, Arg3            , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3                      ) Cv { return CFunction(CObj, arg0, arg1, arg2, arg3            ); }
#define FWD_C_OBJ_5_CV(Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4      , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4           ) Cv { return CFunction(CObj, arg0, arg1, arg2, arg3, arg4      ); }
#define FWD_C_OBJ_6_CV(Fwd, CObj, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5) Cv { return CFunction(CObj, arg0, arg1, arg2, arg3, arg4, arg5); }

// Bind 0
#define FWD_C_OBJ_1_BIND_0(   Fwd, CObj, Ret, CFunction, Bind0                                        ) Ret Fwd (                                                                )    { return CFunction(CObj, Bind0                                    ); }
#define FWD_C_OBJ_2_BIND_0(   Fwd, CObj, Ret, CFunction, Bind0, Arg1                                  ) Ret Fwd (Arg1 arg1                                                       )    { return CFunction(CObj, Bind0, arg1                              ); }
#define FWD_C_OBJ_3_BIND_0(   Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2                            ) Ret Fwd (Arg1 arg1, Arg2 arg2                                            )    { return CFunction(CObj, Bind0, arg1, arg2                        ); }
#define FWD_C_OBJ_4_BIND_0(   Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3                      ) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3                                 )    { return CFunction(CObj, Bind0, arg1, arg2, arg3                  ); }
#define FWD_C_OBJ_5_BIND_0(   Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4                ) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4                      )    { return CFunction(CObj, Bind0, arg1, arg2, arg3, arg4            ); }
#define FWD_C_OBJ_6_BIND_0(   Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5          ) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5           )    { return CFunction(CObj, Bind0, arg1, arg2, arg3, arg4, arg5      ); }
#define FWD_C_OBJ_7_BIND_0(   Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6    ) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5, Arg6 arg6)    { return CFunction(CObj, Bind0, arg1, arg2, arg3, arg4, arg5, arg6); }
#define FWD_C_OBJ_1_CV_BIND_0(Fwd, CObj, Ret, CFunction, Bind0                                    , Cv) Ret Fwd (                                                                ) Cv { return CFunction(CObj, Bind0                                    ); }
#define FWD_C_OBJ_2_CV_BIND_0(Fwd, CObj, Ret, CFunction, Bind0, Arg1                              , Cv) Ret Fwd (Arg1 arg1                                                       ) Cv { return CFunction(CObj, Bind0, arg1                              ); }
#define FWD_C_OBJ_3_CV_BIND_0(Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2                        , Cv) Ret Fwd (Arg1 arg1, Arg2 arg2                                            ) Cv { return CFunction(CObj, Bind0, arg1, arg2                        ); }
#define FWD_C_OBJ_4_CV_BIND_0(Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3                  , Cv) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3                                 ) Cv { return CFunction(CObj, Bind0, arg1, arg2, arg3                  ); }
#define FWD_C_OBJ_5_CV_BIND_0(Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4            , Cv) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4                      ) Cv { return CFunction(CObj, Bind0, arg1, arg2, arg3, arg4            ); }
#define FWD_C_OBJ_6_CV_BIND_0(Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5      , Cv) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5           ) Cv { return CFunction(CObj, Bind0, arg1, arg2, arg3, arg4, arg5      ); }
#define FWD_C_OBJ_7_CV_BIND_0(Fwd, CObj, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Cv) Ret Fwd (Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5, Arg6 arg6) Cv { return CFunction(CObj, Bind0, arg1, arg2, arg3, arg4, arg5, arg6); }

// C Object Data
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Direct
#define FWD_C_DATA_0(   Fwd, CType, CData, Ret, CFunction                                        ) FWD_C_OBJ_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction                                        )
#define FWD_C_DATA_1(   Fwd, CType, CData, Ret, CFunction, Arg0                                  ) FWD_C_OBJ_1(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0                                  )
#define FWD_C_DATA_2(   Fwd, CType, CData, Ret, CFunction, Arg0, Arg1                            ) FWD_C_OBJ_2(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1                            )
#define FWD_C_DATA_3(   Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2                      ) FWD_C_OBJ_3(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2                      )
#define FWD_C_DATA_4(   Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2, Arg3                ) FWD_C_OBJ_4(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2, Arg3                )
#define FWD_C_DATA_5(   Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4          ) FWD_C_OBJ_5(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4          )
#define FWD_C_DATA_6(   Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5    ) FWD_C_OBJ_6(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5    )
#define FWD_C_DATA_0_CV(Fwd, CType, CData, Ret, CFunction                                    , Cv) FWD_C_OBJ_0_CV(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction                                    , Cv)
#define FWD_C_DATA_1_CV(Fwd, CType, CData, Ret, CFunction, Arg0                              , Cv) FWD_C_OBJ_1_CV(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0                              , Cv)
#define FWD_C_DATA_2_CV(Fwd, CType, CData, Ret, CFunction, Arg0, Arg1                        , Cv) FWD_C_OBJ_2_CV(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1                        , Cv)
#define FWD_C_DATA_3_CV(Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2                  , Cv) FWD_C_OBJ_3_CV(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2                  , Cv)
#define FWD_C_DATA_4_CV(Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2, Arg3            , Cv) FWD_C_OBJ_4_CV(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2, Arg3            , Cv)
#define FWD_C_DATA_5_CV(Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4      , Cv) FWD_C_OBJ_5_CV(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4      , Cv)
#define FWD_C_DATA_6_CV(Fwd, CType, CData, Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Cv) FWD_C_OBJ_6_CV(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Cv)

// Bind 0
#define FWD_C_DATA_1_BIND_0(   Fwd, CType, CData, Ret, CFunction, Bind0                                        ) FWD_C_OBJ_1_BIND_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0                                        )
#define FWD_C_DATA_2_BIND_0(   Fwd, CType, CData, Ret, CFunction, Bind0, Arg1                                  ) FWD_C_OBJ_2_BIND_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1                                  )
#define FWD_C_DATA_3_BIND_0(   Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2                            ) FWD_C_OBJ_3_BIND_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2                            )
#define FWD_C_DATA_4_BIND_0(   Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3                      ) FWD_C_OBJ_4_BIND_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3                      )
#define FWD_C_DATA_5_BIND_0(   Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4                ) FWD_C_OBJ_5_BIND_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4                )
#define FWD_C_DATA_6_BIND_0(   Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5          ) FWD_C_OBJ_6_BIND_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5          )
#define FWD_C_DATA_7_BIND_0(   Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6    ) FWD_C_OBJ_7_BIND_0(   Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6    )
#define FWD_C_DATA_1_CV_BIND_0(Fwd, CType, CData, Ret, CFunction, Bind0                                    , Cv) FWD_C_OBJ_1_CV_BIND_0(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0                                    , Cv)
#define FWD_C_DATA_2_CV_BIND_0(Fwd, CType, CData, Ret, CFunction, Bind0, Arg1                              , Cv) FWD_C_OBJ_2_CV_BIND_0(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1                              , Cv)
#define FWD_C_DATA_3_CV_BIND_0(Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2                        , Cv) FWD_C_OBJ_3_CV_BIND_0(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2                        , Cv)
#define FWD_C_DATA_4_CV_BIND_0(Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3                  , Cv) FWD_C_OBJ_4_CV_BIND_0(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3                  , Cv)
#define FWD_C_DATA_5_CV_BIND_0(Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4            , Cv) FWD_C_OBJ_5_CV_BIND_0(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4            , Cv)
#define FWD_C_DATA_6_CV_BIND_0(Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5      , Cv) FWD_C_OBJ_6_CV_BIND_0(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5      , Cv)
#define FWD_C_DATA_7_CV_BIND_0(Fwd, CType, CData, Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Cv) FWD_C_OBJ_7_CV_BIND_0(Fwd, reinterpret_cast<CType>(CData), Ret, CFunction, Bind0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6, Cv)

// C++ Object
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define FWD_OBJ_0(   Fwd, Obj, Ret, Method                                        ) Ret Fwd (                                                                )    { return Obj->Method(                                  ); }
#define FWD_OBJ_1(   Fwd, Obj, Ret, Method, Arg0                                  ) Ret Fwd (Arg0 arg0                                                       )    { return Obj->Method(arg0                              ); }
#define FWD_OBJ_2(   Fwd, Obj, Ret, Method, Arg0, Arg1                            ) Ret Fwd (Arg0 arg0, Arg1 arg1                                            )    { return Obj->Method(arg0, arg1                        ); }
#define FWD_OBJ_3(   Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2                      ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2                                 )    { return Obj->Method(arg0, arg1, arg2                  ); }
#define FWD_OBJ_4(   Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2, Arg3                ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3                      )    { return Obj->Method(arg0, arg1, arg2, arg3            ); }
#define FWD_OBJ_5(   Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4          ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4           )    { return Obj->Method(arg0, arg1, arg2, arg3, arg4      ); }
#define FWD_OBJ_6(   Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5    ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5)    { return Obj->Method(arg0, arg1, arg2, arg3, arg4, arg5); }
#define FWD_OBJ_0_CV(Fwd, Obj, Ret, Method                                    , Cv) Ret Fwd (                                                                ) Cv { return Obj->Method(                                  ); }
#define FWD_OBJ_1_CV(Fwd, Obj, Ret, Method, Arg0                              , Cv) Ret Fwd (Arg0 arg0                                                       ) Cv { return Obj->Method(arg0                              ); }
#define FWD_OBJ_2_CV(Fwd, Obj, Ret, Method, Arg0, Arg1                        , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1                                            ) Cv { return Obj->Method(arg0, arg1                        ); }
#define FWD_OBJ_3_CV(Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2                  , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2                                 ) Cv { return Obj->Method(arg0, arg1, arg2                  ); }
#define FWD_OBJ_4_CV(Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2, Arg3            , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3                      ) Cv { return Obj->Method(arg0, arg1, arg2, arg3            ); }
#define FWD_OBJ_5_CV(Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4      , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4           ) Cv { return Obj->Method(arg0, arg1, arg2, arg3, arg4      ); }
#define FWD_OBJ_6_CV(Fwd, Obj, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5) Cv { return Obj->Method(arg0, arg1, arg2, arg3, arg4, arg5); }

// C++ Member
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define FWD_0(   Fwd, Ref, Ret, Method                                        ) Ret Fwd (                                                                )    { return Ref.Method(                                  ); }
#define FWD_1(   Fwd, Ref, Ret, Method, Arg0                                  ) Ret Fwd (Arg0 arg0                                                       )    { return Ref.Method(arg0                              ); }
#define FWD_2(   Fwd, Ref, Ret, Method, Arg0, Arg1                            ) Ret Fwd (Arg0 arg0, Arg1 arg1                                            )    { return Ref.Method(arg0, arg1                        ); }
#define FWD_3(   Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2                      ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2                                 )    { return Ref.Method(arg0, arg1, arg2                  ); }
#define FWD_4(   Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2, Arg3                ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3                      )    { return Ref.Method(arg0, arg1, arg2, arg3            ); }
#define FWD_5(   Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4          ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4           )    { return Ref.Method(arg0, arg1, arg2, arg3, arg4      ); }
#define FWD_6(   Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5    ) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5)    { return Ref.Method(arg0, arg1, arg2, arg3, arg4, arg5); }
#define FWD_0_CV(Fwd, Ref, Ret, Method                                    , Cv) Ret Fwd (                                                                ) Cv { return Ref.Method(                                  ); }
#define FWD_1_CV(Fwd, Ref, Ret, Method, Arg0                              , Cv) Ret Fwd (Arg0 arg0                                                       ) Cv { return Ref.Method(arg0                              ); }
#define FWD_2_CV(Fwd, Ref, Ret, Method, Arg0, Arg1                        , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1                                            ) Cv { return Ref.Method(arg0, arg1                        ); }
#define FWD_3_CV(Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2                  , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2                                 ) Cv { return Ref.Method(arg0, arg1, arg2                  ); }
#define FWD_4_CV(Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2, Arg3            , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3                      ) Cv { return Ref.Method(arg0, arg1, arg2, arg3            ); }
#define FWD_5_CV(Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4      , Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4           ) Cv { return Ref.Method(arg0, arg1, arg2, arg3, arg4      ); }
#define FWD_6_CV(Fwd, Ref, Ret, Method, Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Cv) Ret Fwd (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5) Cv { return Ref.Method(arg0, arg1, arg2, arg3, arg4, arg5); }

// C++ Constructors
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define FWD_CONSTRUCTORS(Class, Delegate)                                                                                                                                                                                         \
                                                                                                      Class (                                                                ) : Delegate(                                  ) { } \
template < typename Arg0                                                                            > Class (Arg0 arg0                                                       ) : Delegate(arg0                              ) { } \
template < typename Arg0, typename Arg1                                                             > Class (Arg0 arg0, Arg1 arg1                                            ) : Delegate(arg0, arg1                        ) { } \
template < typename Arg0, typename Arg1, typename Arg2                                              > Class (Arg0 arg0, Arg1 arg1, Arg2 arg2                                 ) : Delegate(arg0, arg1, arg2                  ) { } \
template < typename Arg0, typename Arg1, typename Arg2, typename Arg3                               > Class (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3                      ) : Delegate(arg0, arg1, arg2, arg3            ) { } \
template < typename Arg0, typename Arg1, typename Arg2, typename Arg3, typename Arg4                > Class (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4           ) : Delegate(arg0, arg1, arg2, arg3, arg4      ) { } \
template < typename Arg0, typename Arg1, typename Arg2, typename Arg3, typename Arg4, typename Arg5 > Class (Arg0 arg0, Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5) : Delegate(arg0, arg1, arg2, arg3, arg4, arg5) { }

#endif // !NTK_HUB_FWD_H
