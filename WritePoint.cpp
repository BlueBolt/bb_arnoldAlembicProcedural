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

#include "WritePoint.h"
#include "WriteTransform.h"
#include "WriteOverrides.h"

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


template <typename geomParamT>
void ProcessIndexedBuiltinParam(
        geomParamT & param,
        const SampleTimeSet & sampleTimes,
        std::vector<float> & values,
        std::vector<unsigned int> & idxs,
        size_t elementSize)
{
    if ( !param.valid() ) { return; }
    
    bool isFirstSample = true;
    for ( SampleTimeSet::iterator I = sampleTimes.begin();
          I != sampleTimes.end(); ++I, isFirstSample = false)
    {
        ISampleSelector sampleSelector( *I );
        
        
        switch ( param.getScope() )
        {
        case kVaryingScope:
        case kVertexScope:
        {
            // a value per-point, idxs should be the same as vidxs
            // so we'll leave it empty
            
            // we'll get the expanded form here
            typename geomParamT::Sample sample = param.getExpandedValue(
                    sampleSelector);
            
            size_t footprint = sample.getVals()->size() * elementSize;
            
            values.reserve( values.size() + footprint );
            values.insert( values.end(),
                    (float32_t*) sample.getVals()->get(),
                    ((float32_t*) sample.getVals()->get()) + footprint );
            
            break;
        }
        case kFacevaryingScope:
        {
            // get the indexed form and feed to nidxs
            
            typename geomParamT::Sample sample = param.getIndexedValue(
                    sampleSelector);
            
            if ( isFirstSample )
            {
                idxs.reserve( sample.getIndices()->size() );
                idxs.insert( idxs.end(),
                        sample.getIndices()->get(),
                        sample.getIndices()->get() +
                                sample.getIndices()->size() );
            }
            
            size_t footprint = sample.getVals()->size() * elementSize;
            values.reserve( values.size() + footprint );
            values.insert( values.end(),
                    (const float32_t*) sample.getVals()->get(),
                    ((const float32_t*) sample.getVals()->get()) + footprint );
            
            break;
        }
        default:
            break;
        }
        
        
    }
    
    
}

//-*****************************************************************************

namespace
{
    // Arnold scene build is single-threaded so we don't have to lock around
    // access to this for now.
    typedef std::map<std::string, AtNode *> NodeCache;
    NodeCache g_meshCache;
}


AtNode * ProcessPointsBase(
        IPoints & prim, ProcArgs & args,
        SampleTimeSet & sampleTimes,
        std::vector<AtPoint> & vidxs,
		std::vector<float> & radius,
		MatrixSampleMap * xformSamples )
{
    if ( !prim.valid() )
    {
        return NULL;
    }
    
    Alembic::AbcGeom::IPointsSchema  &ps = prim.getSchema();
    TimeSamplingPtr ts = ps.getTimeSampling();
    
	sampleTimes.insert( ts->getFloorIndex(args.frame / args.fps, ps.getNumSamples()).second );
    
    std::string name = args.nameprefix + prim.getFullName();
    
    // check if this meshes name matches the search pattern in the arguments

    if ( !matchPattern(name,args.pattern) && args.pattern != "*" && args.pattern != "" )
    {
        return NULL;
    }

    // check if this mesh matches the exclude pattern

    if ( matchPattern(name,args.excludePattern) && args.excludePattern != "" )
    {
        return NULL;
    }

    AtNode * instanceNode = NULL;
    
    std::string cacheId;
    
    SampleTimeSet singleSampleTimes;
    singleSampleTimes.insert( ts->getFloorIndex(args.frame / args.fps, ps.getNumSamples()).second );

	ICompoundProperty arbGeomParams = ps.getArbGeomParams();
	ISampleSelector frameSelector( *singleSampleTimes.begin() );
	std::vector<std::string> tags;

	//get tags
	if ( arbGeomParams != NULL && arbGeomParams.valid() )
	{
		if (arbGeomParams.getPropertyHeader("mtoa_constant_tags") != NULL)
		{
			const PropertyHeader * tagsHeader = arbGeomParams.getPropertyHeader("mtoa_constant_tags");
			if (IStringGeomParam::matches( *tagsHeader ))
			{
				IStringGeomParam param( arbGeomParams,  "mtoa_constant_tags" );
				if ( param.valid() )
				{
					IStringGeomParam::prop_type::sample_ptr_type valueSample =
									param.getExpandedValue( frameSelector ).getVals();

					if ( param.getScope() == kConstantScope || param.getScope() == kUnknownScope)
					{
						Json::Value jtags;
						Json::Reader reader;
						if(reader.parse(valueSample->get()[0], jtags))
							for( Json::ValueIterator itr = jtags.begin() ; itr != jtags.end() ; itr++ )
							{
								tags.push_back(jtags[itr.key().asUInt()].asString());
							}
					}
				}
			}
		}
	}

    if ( args.makeInstance )
    {
        std::ostringstream buffer;
        AbcA::ArraySampleKey sampleKey;
        
        
        for ( SampleTimeSet::iterator I = sampleTimes.begin();
                I != sampleTimes.end(); ++I )
        {
            ISampleSelector sampleSelector( *I );
            ps.getPositionsProperty().getKey(sampleKey, sampleSelector);
            
            buffer << GetRelativeSampleTime( args, (*I) ) << ":";
            sampleKey.digest.print(buffer);
            buffer << ":";
        }
        
        cacheId = buffer.str();
        
        instanceNode = AiNode( "ginstance" );
        AiNodeSetStr( instanceNode, "name", name.c_str() );
		args.createdNodes.push_back(instanceNode);

        if ( args.proceduralNode )
        {
            AiNodeSetByte( instanceNode, "visibility",
                    AiNodeGetByte( args.proceduralNode, "visibility" ) );
        
        }
        else
        {
            AiNodeSetByte( instanceNode, "visibility", AI_RAY_ALL );
        }

		ApplyTransformation( instanceNode, xformSamples, args );

		NodeCache::iterator I = g_meshCache.find(cacheId);

		// parameters overrides
		if(args.linkOverride)
			ApplyOverrides(name, instanceNode, tags, args);

		// shader assignation
		if (nodeHasParameter( instanceNode, "shader" ) )
		{
			if(args.linkShader)
			{
				ApplyShaders(name, instanceNode, tags, args);
			}
			else
			{
				AtArray* shaders = AiNodeGetArray(args.proceduralNode, "shader");
				if (shaders->nelements != 0)
				   AiNodeSetArray(instanceNode, "shader", AiArrayCopy(shaders));
			}
		}

        if ( I != g_meshCache.end() )
        {
            AiNodeSetPtr(instanceNode, "node", (*I).second );	
			return NULL;
        }
    }
    

    bool isFirstSample = true;

	float radiusPoint = 0.1f;
	if (AiNodeLookUpUserParameter(args.proceduralNode, "radiusPoint") !=NULL )
		radiusPoint = AiNodeGetFlt(args.proceduralNode, "radiusPoint");
	
	

	bool useVelocities = false;
	if ((sampleTimes.size() == 1) && (args.shutterOpen != args.shutterClose))
	{
		// no sample, and motion blur needed, let's try to get velocities.
		if(ps.getVelocitiesProperty().valid())
			useVelocities = true;
	}

	for ( SampleTimeSet::iterator I = sampleTimes.begin();
          I != sampleTimes.end(); ++I, isFirstSample = false)
    {
        ISampleSelector sampleSelector( *I );
        Alembic::AbcGeom::IPointsSchema::Sample sample = ps.getValue( sampleSelector );

		Alembic::Abc::P3fArraySamplePtr v3ptr = sample.getPositions();
		size_t pSize = sample.getPositions()->size(); 

		if(useVelocities && isFirstSample)
		{
			float scaleVelocity = 1.0f;
			if (AiNodeLookUpUserParameter(args.proceduralNode, "scaleVelocity") !=NULL )
				scaleVelocity = AiNodeGetFlt(args.proceduralNode, "scaleVelocity");

			vidxs.resize(pSize*2);
			Alembic::Abc::V3fArraySamplePtr velptr = sample.getVelocities();

			float timeoffset = ((args.frame / args.fps) - ts->getFloorIndex((*I), ps.getNumSamples()).second) * args.fps;

			for ( size_t pId = 0; pId < pSize; ++pId ) 
			{
				Alembic::Abc::V3f posAtOpen = ((*v3ptr)[pId] + (*velptr)[pId] * scaleVelocity *-timeoffset);			
				AtPoint pos1;
				pos1.x = posAtOpen.x;
				pos1.y = posAtOpen.y;
				pos1.z = posAtOpen.z;
				vidxs[pId]= pos1;

				Alembic::Abc::V3f posAtEnd = ((*v3ptr)[pId] + (*velptr)[pId]* scaleVelocity *(1.0f-timeoffset));
				AtPoint pos2;
				pos2.x = posAtEnd.x;
				pos2.y = posAtEnd.y;
				pos2.z = posAtEnd.z;
				vidxs[pId+pSize]= pos2;
				
				radius.push_back(radiusPoint);	
			}
		}
		else
			// not motion blur or correctly sampled particles
		{
			for ( size_t pId = 0; pId < pSize; ++pId ) 
			{
				AtPoint pos;
				pos.x = (*v3ptr)[pId].x;
				pos.y = (*v3ptr)[pId].y;
				pos.z = (*v3ptr)[pId].z;
				vidxs.push_back(pos);
				radius.push_back(radiusPoint);
			}
		}
	}
    
    AtNode* pointsNode = AiNode( "points" );
    
    if (!pointsNode)
    {
        AiMsgError("Failed to make points node for %s",
                prim.getFullName().c_str());
        return NULL;
    }
    

    args.createdNodes.push_back(pointsNode);
    if ( instanceNode != NULL)
    {
        AiNodeSetStr( pointsNode, "name", (name + ":src").c_str() );
    }
    else
    {
        AiNodeSetStr( pointsNode, "name", name.c_str() );
    }
    
    if(!useVelocities)
	{
		AiNodeSetArray(pointsNode, "points",
				AiArrayConvert( vidxs.size() / sampleTimes.size(), 
						sampleTimes.size(), AI_TYPE_POINT, (void*)(&(vidxs[0]))
								));
		AiNodeSetArray(pointsNode, "radius",
				AiArrayConvert( vidxs.size() / sampleTimes.size(), 
						sampleTimes.size(), AI_TYPE_FLOAT, (void*)(&(radius[0]))
								));

		if ( sampleTimes.size() > 1 )
		{
			std::vector<float> relativeSampleTimes;
			relativeSampleTimes.reserve( sampleTimes.size() );
        
			for (SampleTimeSet::const_iterator I = sampleTimes.begin();
					I != sampleTimes.end(); ++I )
			{
			   chrono_t sampleTime = GetRelativeSampleTime( args, (*I) );

				relativeSampleTimes.push_back(sampleTime);
                    
			}
        
			AiNodeSetArray( pointsNode, "deform_time_samples",
					AiArrayConvert(relativeSampleTimes.size(), 1,
							AI_TYPE_FLOAT, &relativeSampleTimes[0]));
		}
	}
	else
	{
		AiNodeSetArray(pointsNode, "points",
				AiArrayConvert( vidxs.size() / 2, 
						2, AI_TYPE_POINT, (void*)(&(vidxs[0]))
								));
		AiNodeSetArray(pointsNode, "radius",
				AiArrayConvert( vidxs.size() /2 / sampleTimes.size(), 
						sampleTimes.size(), AI_TYPE_FLOAT, (void*)(&(radius[0]))
								));		
		
		AiNodeSetArray( pointsNode, "deform_time_samples",
					AiArray(2, 1, AI_TYPE_FLOAT, 0.f, 1.f));

	}

   AddArbitraryGeomParams( arbGeomParams, frameSelector, pointsNode );
    
    if ( instanceNode == NULL )
	{
        if ( xformSamples )
        {
            ApplyTransformation( pointsNode, xformSamples, args );
        }
        
        return pointsNode;
	}
    else
    {
        AiNodeSetByte( pointsNode, "visibility", 0 );

		  AiNodeSetInt( pointsNode, "mode", 1 );
        
        AiNodeSetPtr(instanceNode, "node", pointsNode );
        g_meshCache[cacheId] = pointsNode;
        return pointsNode;
    }
    
}



void ProcessPoint( IPoints &points, ProcArgs &args,
					MatrixSampleMap * xformSamples)
{
	SampleTimeSet sampleTimes;
    std::vector<AtPoint> vidxs;
	std::vector<float> radius;
    
    AtNode * pointsNode = ProcessPointsBase(
            points, args, sampleTimes, vidxs, radius, xformSamples);
    
    // This is a valid condition for the second instance onward and just
    // means that we don't need to do anything further.
    if ( !pointsNode )
    {
        return;
    }
    
    IPointsSchema &ps = points.getSchema();

    
}

