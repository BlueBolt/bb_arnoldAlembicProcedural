##-*****************************************************************************
##
## Copyright (c) 2009-2011, Industrial Light & Magic,
##   a division of Lucasfilm Entertainment Company Ltd.
##
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:
## *       Redistributions of source code must retain the above copyright
## notice, this list of conditions and the following disclaimer.
## *       Redistributions in binary form must reproduce the above
## copyright notice, this list of conditions and the following disclaimer
## in the documentation and/or other materials provided with the
## distribution.
## *       Neither the name of Industrial Light & Magic nor the names of
## its contributors may be used to endorse or promote products derived
## from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
##-*****************************************************************************

PROJECT( ALEMBIC_ARNOLD_PROCEDURAL_MODULE )


IF( ${ALEMBIC_ARNOLD_FOUND} )

SET( H_FILES
  ArbGeomParams.h
  PathUtil.h
  ProcArgs.h
  SampleUtil.h
  WriteGeo.h
  json/json.h
  pystring.h
  )

SET( CXX_FILES
  ArbGeomParams.cpp
  PathUtil.cpp
  ProcArgs.cpp
  ProcMain.cpp
  SampleUtil.cpp
  WriteGeo.cpp
  jsoncpp.cpp
  pystring.cpp
)

SET( CORE_LIBS
  AlembicAbcGeom
  AlembicAbc
  AlembicAbcCoreHDF5
  AlembicAbcCoreOgawa
  AlembicAbcCoreFactory
  AlembicAbcCoreAbstract
  AlembicOgawa
  AlembicUtil )

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DJSON_IS_AMALGAMATION " ) # for the json lib to compile
#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DJSON_IS_AMALGAMATION -std=c++0x" ) # added for c++11 stad lib access

MESSAGE( STATUS "Boost_REGEX_LIBRARY: ${Boost_REGEX_LIBRARY}" )

ADD_ARNOLD_CXX_PLUGIN( bb_AlembicArnoldProcedural ProcMain.cpp ${H_FILES} ${CXX_FILES} )
TARGET_LINK_LIBRARIES( bb_AlembicArnoldProcedural
  ${CORE_LIBS}
  ${ALEMBIC_HDF5_LIBS}
  ${ALEMBIC_ILMBASE_LIBS}
  ${Boost_REGEX_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${ZLIB_LIBRARIES}
)

INSTALL( TARGETS bb_AlembicArnoldProcedural
         DESTINATION arnold )



ENDIF()
