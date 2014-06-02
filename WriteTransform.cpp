//-*****************************************************************************
//
// Copyright (c) 2009-2011,
//  Sony Pictures Imageworks Inc. and
//  Industrial Light & Magic, a division of Lucasfilm Entertainment Company Ltd.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of Sony Pictures Imageworks, nor
// Industrial Light & Magic, nor the names of their contributors may be used
// to endorse or promote products derived from this software without specific
// prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//-*****************************************************************************

#include "WriteTransform.h"
#include "ArbGeomParams.h"
#include "PathUtil.h"
//#include "ArbAttrUtil.h"

#include <ai.h>
#include <sstream>

#include "json/json.h"

#include <boost/thread.hpp>



//-*****************************************************************************

#if AI_VERSION_ARCH_NUM == 3
    #if AI_VERSION_MAJOR_NUM < 4
        #define AiNodeGetNodeEntry(node)   ((node)->base_node)
    #endif
#endif



//-*****************************************************************************
void printMatrix(AtMatrix matrix)
{
   AiMsgInfo("CurMattrix");
   AiMsgInfo("%f %f %f %f", matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3]);
   AiMsgInfo("%f %f %f %f", matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3]);
   AiMsgInfo("%f %f %f %f", matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3]);
   AiMsgInfo("%f %f %f %f", matrix[3][0], matrix[3][1], matrix[3][2], matrix[3][3]);
}

bool nodeHasParameter( struct AtNode * node, const std::string & paramName)
{
    return AiNodeEntryLookUpParameter( AiNodeGetNodeEntry( node ),
            paramName.c_str() ) != NULL;
}

//-*****************************************************************************

void ApplyTransformation( struct AtNode * node,
        MatrixSampleMap * xformSamples, ProcArgs &args )
{
    if ( !node || !xformSamples || xformSamples->empty() )
    {
        return;
    }
   
    // confirm that this node has a parameter
    if ( !nodeHasParameter( node, "matrix" ) )
    {
        return;
    }
   
    // check to see that we're not a single identity matrix
    if (xformSamples->size() == 1 &&
            xformSamples->begin()->second == Imath::M44d())
    {
        return;
    }
   
   
    std::vector<float> sampleTimes;
    sampleTimes.reserve(xformSamples->size());
   
    std::vector<float> mlist;
    mlist.reserve( 16* xformSamples->size() );
   
    for ( MatrixSampleMap::iterator I = xformSamples->begin();
            I != xformSamples->end(); ++I )
    {
        // build up a vector of relative sample times to feed to
        // "transform_time_samples" or "time_samples"
        sampleTimes.push_back( GetRelativeSampleTime(args, (*I).first) );
       
       
        for (int i = 0; i < 16; i++)
        {
            mlist.push_back( (*I).second.getValue()[i] );
        }
    }
   
    AiNodeSetArray(node, "matrix",
                ArrayConvert(1, xformSamples->size(),
                        AI_TYPE_MATRIX, &mlist[0]));
   
   
    if ( sampleTimes.size() > 1 )
    {
        // persp_camera calls it time_samples while the primitives call it
        // transform_time_samples
        if ( nodeHasParameter( node, "transform_time_samples" ) )
        {
            AiNodeSetArray(node, "transform_time_samples",
                            ArrayConvert(sampleTimes.size(), 1,
                                    AI_TYPE_FLOAT, &sampleTimes[0]));
        }
        else if ( nodeHasParameter( node, "time_samples" ) )
        {
            AiNodeSetArray(node, "time_samples",
                            ArrayConvert(sampleTimes.size(), 1,
                                    AI_TYPE_FLOAT, &sampleTimes[0]));
        }
        else
        {
            //TODO, warn if neither is present? Should be there in all
            //commercial versions of arnold by now.
        }
    }
}




