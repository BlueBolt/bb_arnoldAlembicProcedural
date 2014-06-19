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


//

#include "WriteGeo.h"
#include "WriteTransform.h"
#include "WriteOverrides.h"
#include "ArbGeomParams.h"
#include "PathUtil.h"

#include <ai.h>
#include <sstream>

#include <boost/regex.hpp>
// #include <boost/thread.hpp>
//-*****************************************************************************

#if AI_VERSION_ARCH_NUM == 3
    #if AI_VERSION_MAJOR_NUM < 4
        #define AiNodeGetNodeEntry(node)   ((node)->base_node)
    #endif
#endif

//-*****************************************************************************
std::string GetPrmanScopeString( GeometryScope scope )
{
    switch (scope)
    {
    case kUniformScope:
        return "uniform";
    case kVaryingScope:
        return "varying";
    case kVertexScope:
        return "vertex";
    case kFacevaryingScope:
        return "facevarying";
    case kConstantScope:
    default:
        return "constant";
    }
}

template <typename geomParamT>
void ProcessIndexedBuiltinParam(
        geomParamT param,
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


// The return value is the curves node. If instanced, it will be returned
// for the first created instance only.
AtNode * ProcessCurvesBase(
        ICurves & prim, ProcArgs & args,
        SampleTimeSet & sampleTimes,
        std::vector<AtPoint> & vidxs,
        std::vector<float> & radius,
        MatrixSampleMap * xformSamples)
{
    if ( !prim.valid() )
    {
        return NULL;
    }
    

    // Get the tiome samples for this geo

    Alembic::AbcGeom::ICurvesSchema  &ps = prim.getSchema();
    TimeSamplingPtr ts = ps.getTimeSampling();
   
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


    // do custom attributes and assignments

    
    sampleTimes.insert( ts->getFloorIndex(args.frame / args.fps, ps.getNumSamples()).second );
        
    AtNode * instanceNode = NULL;
    
    std::string cacheId;
    
    SampleTimeSet singleSampleTimes;
    singleSampleTimes.insert( ts->getFloorIndex(args.frame / args.fps, ps.getNumSamples()).second );

    ICompoundProperty arbGeomParams = ps.getArbGeomParams();
    ISampleSelector frameSelector( *singleSampleTimes.begin() );
    std::vector<std::string> tags;

    // get tags
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
    
    // overrides that can't be applied on instances
    // we create a hash from that.
    std::string hashAttributes("@");
    Json::FastWriter writer;
    Json::Value rootEncode;

    if(args.linkOverride)
    {
      bool foundInPath = false;
      for(std::vector<std::string>::iterator it=args.overrides.begin(); it!=args.overrides.end(); ++it)
      {
        Json::Value overrides;
        if(it->find("/") != string::npos)
        {
          if(name.find(*it) != string::npos)
          {
            overrides = args.overrideRoot[*it];
            foundInPath = true;
          }

        }
        else if(matchPattern(name,*it)) // based on wildcard expression
        {
            overrides = args.overrideRoot[*it];
            foundInPath = true;
        }
        else if(foundInPath == false)
        {
          if (std::find(tags.begin(), tags.end(), *it) != tags.end())
          {
            overrides = args.overrideRoot[*it];
          }
        }

        if(overrides.size() > 0)
        {
          for( Json::ValueIterator itr = overrides.begin() ; itr != overrides.end() ; itr++ ) 
          {
            std::string attribute = itr.key().asString();

            if (attribute=="smoothing" 
              || attribute=="opaque" 
              || attribute=="basis"
              || attribute=="mode"                
              || attribute=="min_pixel_width"                
              || attribute=="max_subdivs" 
              || attribute=="invert_normals")
            {
              Json::Value val = args.overrideRoot[*it][itr.key().asString()];
              rootEncode[attribute]=val;
            }
          }
        }
      }
    }

    hashAttributes += writer.write(rootEncode);
   
    if ( args.makeInstance  )
    {
        AiMsgInfo("[ABC] Making Instance for shape %s", name.c_str());
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
        
        buffer << "@" << hash(hashAttributes);
        
        cacheId = buffer.str();
        
        instanceNode = AiNode( "ginstance" );
        AiNodeSetStr( instanceNode, "name", name.c_str() );
        args.createdNodes.push_back(instanceNode);
        
        AiNodeSetBool( instanceNode, "inherit_xform", false );

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
        
        // adding arbitary parameters

        AddArbitraryGeomParams( arbGeomParams, frameSelector, instanceNode );

        NodeCache::iterator I = g_meshCache.find(cacheId);

        // start param overrides on instance
        if(args.linkOverride)
        {
            ApplyOverrides(name, instanceNode, tags, args );
        }   

        if (args.linkUserAttributes)
        {
            ApplyUserAttributes(name, instanceNode, tags,  args);
        }

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
        } // end shader assignment
        
        if ( I != g_meshCache.end() ) 
        {
           AiNodeSetPtr(instanceNode, "node", (*I).second );
           return NULL;
        }

    } // end makeinstance
    
    size_t numSampleTimes = sampleTimes.size();
    size_t numCurves;
    size_t pSize;

    Int32ArraySamplePtr nVertices;

    bool isFirstSample = true;

    float radiusCurve = 0.1f;
    if (AiNodeLookUpUserParameter(args.proceduralNode, "radiusCurve") !=NULL )
      radiusCurve = AiNodeGetFlt(args.proceduralNode, "radiusCurve");

    const char * modeCurve = "ribbon";
    if (AiNodeLookUpUserParameter(args.proceduralNode, "modeCurve") !=NULL )
      modeCurve = AiNodeGetStr(args.proceduralNode, "modeCurve");
    
    const char * basis = NULL;

    for ( SampleTimeSet::iterator I = sampleTimes.begin();
        I != sampleTimes.end(); ++I, isFirstSample = false)
    {
        ISampleSelector sampleSelector( *I );
        Alembic::AbcGeom::ICurvesSchema::Sample sample = ps.getValue( sampleSelector );

        if ( isFirstSample )
        {                   

            numCurves = sample.getNumCurves();  
            size_t vidxSize = sample.getCurvesNumVertices()->size();
            nVertices = sample.getCurvesNumVertices();                 
            vidxs.reserve( vidxSize );

            BasisType basisType = sample.getBasis();
            if ( basisType != kNoBasis )
            {
                
                switch ( basisType )
                {
                case kBezierBasis:
                    basis = "bezier";
                    break;
                case kBsplineBasis:
                    basis = "b-spline";
                    break;
                case kCatmullromBasis:
                    basis = "catmull-rom";
                    break;
                case kHermiteBasis:
                    basis = "hermite";
                    break;
                case kPowerBasis:
                    basis = "power";
                    break;
                default:
                    break;
                }
            }
        }
        
        pSize = sample.getPositions()->size(); 


        Alembic::Abc::V3fArraySamplePtr velptr = sample.getVelocities();
        Alembic::Abc::P3fArraySamplePtr v3ptr = sample.getPositions();
        
        if(numSampleTimes == 1 && (args.shutterOpen != args.shutterClose) && (ps.getVelocitiesProperty().valid()) && isFirstSample )
        {
            float scaleVelocity = 1.0f;
            if (AiNodeLookUpUserParameter(args.proceduralNode, "scaleVelocity") !=NULL )
                scaleVelocity = AiNodeGetFlt(args.proceduralNode, "scaleVelocity");
            vidxs.resize(pSize*3*2);
            numSampleTimes = 2;

            float timeoffset = ((args.frame / args.fps) - ts->getFloorIndex((*I), ps.getNumSamples()).second) * args.fps;

            for ( size_t vId = 0; vId < pSize; ++vId )
            {
                
                Alembic::Abc::V3f posAtOpen = ((*v3ptr)[vId] + (*velptr)[vId] * scaleVelocity *-timeoffset);
                AtPoint pos1;
                pos1.x = posAtOpen.x;
                pos1.y = posAtOpen.y;
                pos1.z = posAtOpen.z;
                vidxs[vId]= pos1;


                Alembic::Abc::V3f posAtEnd = ((*v3ptr)[vId] + (*velptr)[vId] * scaleVelocity *(1.0f-timeoffset));
                AtPoint pos2;
                pos2.x = posAtEnd.x;
                pos2.y = posAtEnd.y;
                pos2.z = posAtEnd.z;          
                vidxs[vId+pSize]= pos1;

                // radius.push_back(radiusCurve);  
            }
        }
        else
        {
          for ( size_t pId = 0; pId < pSize; ++pId ) 
          {
            AtPoint pos;
            pos.x = (*v3ptr)[pId].x;
            pos.y = (*v3ptr)[pId].y;
            pos.z = (*v3ptr)[pId].z;
            vidxs.push_back(pos);
            // radius.push_back(radiusCurve);
          }
        }
    }


    std::vector<float> fullradlist;
    std::vector<float> radlist;
    std::vector<unsigned int> radidxs;

    ProcessIndexedBuiltinParam(
            ps.getWidthsParam(),
            singleSampleTimes,
            fullradlist,
            radidxs,
            1);   



    // need to pop out the values from radius so 
    // they make up 2 less values for each curve

    // get the points per curve
    AtArray* curveNumPoints = AiArrayAllocate( numCurves , 1, AI_TYPE_UINT);  

    unsigned int w_end = 0;

    for ( size_t currentCurve = 0; currentCurve < numCurves ;
          ++currentCurve )
    {
        unsigned int c_verts = nVertices->get()[currentCurve];

        AiArraySetUInt(curveNumPoints, currentCurve, c_verts);

        // as splines require two less vtx widths per curve we crop out the 
        // second and second to last width before outputing to radlist vector
        if ( !fullradlist.empty() )
        {
          unsigned int w_start = w_end;
          w_end = (currentCurve+1)*c_verts;

          std::vector<float> this_range(fullradlist.begin() + w_start,
                                        fullradlist.begin() + w_end);

          for ( size_t r=0; r < this_range.size(); ++r )
          {
              if ( r != 1 && r !=this_range.size()-2 )
                radlist.push_back(this_range[r]);
          }
        }

    }

    AtArray* curveWidths;

    if ( !radlist.empty() )
    {

      curveWidths = AiArrayAllocate(static_cast<unsigned int>( radlist.size() ),
                                           1, AI_TYPE_FLOAT); 
      for (unsigned int i = 0; i < radlist.size() ; ++i)
      {
          AiArraySetFlt(curveWidths, i, radlist[i]);
      }

    }
    else
    {
      // write out as uniform values, need to be more clever 
      // with this, at the moment we assume all the curves are 
      // the same number of verts
      curveWidths = AiArrayAllocate(static_cast<unsigned int>( pSize-(numCurves*2) ),
                                           1, AI_TYPE_FLOAT); 

      for ( size_t PId = 0; PId < pSize-(numCurves*2) ; ++PId ) 
      {
          AiArraySetFlt(curveWidths, PId, radiusCurve);
      }
    }


    // TODO suface->curve UVs

    // std::vector<float> uvlist;
    // std::vector<unsigned int> uvidxs;

    // ProcessIndexedBuiltinParam(
    //         cs.getUVsParam(), // getUVsPAram looks for "uvs" as a float2 array (V2f), currently not exported from AbcExport
    //         singleSampleTimes,
    //         uvlist,
    //         uvidxs,
    //         2);   

    AtNode* curvesNode = AiNode( "curves" );

    if (!curvesNode)
    {
      AiMsgError("Failed to make curves node for %s",
              prim.getFullName().c_str());
      return NULL;
    }

    args.createdNodes.push_back(curvesNode);

    // the basis of the curve (bezier,catmull-rom etc)
    if ( basis != NULL )
    {
      AiNodeSetStr(curvesNode, "basis", basis);
    }

    // the mode of the curve (ribbon,thick,oriented)
    if ( modeCurve != NULL )
    {
      AiNodeSetStr(curvesNode, "mode", modeCurve);
    }

    // Attribute overrides. We assume instance mode all the time here.
    if(args.linkOverride)
    {
    for(std::vector<std::string>::iterator it=args.overrides.begin(); it!=args.overrides.end(); ++it)
    {
      if(name.find(*it) != string::npos || std::find(tags.begin(), tags.end(), *it) != tags.end() || matchPattern(name,*it))
      {
        const Json::Value overrides = args.overrideRoot[*it];
        if(overrides.size() > 0)
        {
          for( Json::ValueIterator itr = overrides.begin() ; itr != overrides.end() ; itr++ ) 
          {
            std::string attribute = itr.key().asString();
            AiMsgDebug("[ABC] Checking attribute %s for shape %s", attribute.c_str(), name.c_str());

            if (attribute=="smoothing" 
              || attribute=="opaque" 
              || attribute=="basis"
              || attribute=="mode"                
              || attribute=="min_pixel_width"                
              || attribute=="max_subdivs" 
              || attribute=="invert_normals")
            {
              // check if the attribute exists ...
              const AtNodeEntry* nodeEntry = AiNodeGetNodeEntry(curvesNode);
              const AtParamEntry* paramEntry = AiNodeEntryLookUpParameter(nodeEntry, attribute.c_str());

              if ( paramEntry != NULL)
              {
                AiMsgDebug("[ABC] attribute %s exists on shape", attribute.c_str());
                Json::Value val = args.overrideRoot[*it][itr.key().asString()];
                if( val.isString() ) 
                  AiNodeSetStr(curvesNode, attribute.c_str(), val.asCString());
                else if( val.isBool() ) 
                  AiNodeSetBool(curvesNode, attribute.c_str(), val.asBool());
                else if( val.isInt() ) 
                {
                  //make the difference between Byte & int!
                  int typeEntry = AiParamGetType(paramEntry);
                  if(typeEntry == AI_TYPE_BYTE)
                    AiNodeSetByte(curvesNode, attribute.c_str(), val.asInt());
                  else 
                    AiNodeSetInt(curvesNode, attribute.c_str(), val.asInt());
                }
                else if( val.isUInt() ) 
                  AiNodeSetUInt(curvesNode, attribute.c_str(), val.asUInt());
                else if( val.isDouble() ) 
                  AiNodeSetFlt(curvesNode, attribute.c_str(), val.asDouble());
              }
            }
          }
        }
      }
    }
    }

    if ( instanceNode != NULL)
    {
      AiNodeSetStr( curvesNode, "name", (name + ":src").c_str() );
    }
    else
    {
      AiNodeSetStr( curvesNode, "name", name.c_str() );
    }


    // number of points per curves
    AiNodeSetArray(curvesNode, "num_points", curveNumPoints);

    // the point positions for the curves
    AiNodeSetArray(curvesNode, "points",
          AiArrayConvert(vidxs.size(), 1, AI_TYPE_POINT,
                  (void*)&vidxs[0]));

    // radius
    AiNodeSetArray(curvesNode, "radius",curveWidths);

    if ( sampleTimes.size() > 1 )
    {
      std::vector<float> relativeSampleTimes;
      relativeSampleTimes.reserve( sampleTimes.size() );

      for (SampleTimeSet::const_iterator I = sampleTimes.begin();
              I != sampleTimes.end(); ++I )
      {
          relativeSampleTimes.push_back(
                  GetRelativeSampleTime( args, (*I) ) );

      }

      AiNodeSetArray( curvesNode, "deform_time_samples",
              AiArrayConvert(relativeSampleTimes.size(), 1,
                      AI_TYPE_FLOAT, &relativeSampleTimes[0]));
    }
    else if(numSampleTimes == 2)
    {
        AiNodeSetArray( curvesNode, "deform_time_samples",
                AiArray(2, 1, AI_TYPE_FLOAT, 0.f, 1.f));
    }

    {
        ICompoundProperty arbGeomParams = ps.getArbGeomParams();
        ISampleSelector frameSelector( *singleSampleTimes.begin() );

        /* add user attributs to the current object */
        AddArbitraryGeomParams( arbGeomParams, frameSelector, curvesNode );
    }

    // add as switch
    if ( args.invertNormals )
    {
        AiNodeSetBool( curvesNode, "invert_normals", args.invertNormals );
    }

    if ( instanceNode == NULL )
    {
      if ( xformSamples )
      {
          ApplyTransformation( curvesNode, xformSamples, args );
      }
              // shader assignation
      if (nodeHasParameter( curvesNode, "shader" ) )
      {
        if(args.linkShader)
        {
          ApplyShaders(name, curvesNode, tags, args);
        }
        else
        {
          AtArray* shaders = AiNodeGetArray(args.proceduralNode, "shader");
          if (shaders->nelements != 0)
             AiNodeSetArray(curvesNode, "shader", AiArrayCopy(shaders));
        }
      } // end shader assignment

      return curvesNode;
    }
    else
    {
      AiNodeSetByte( curvesNode, "visibility", 0 ); // had original node that is being referenced

      AiNodeSetPtr(instanceNode, "node", curvesNode );
      g_meshCache[cacheId] = curvesNode;
      return curvesNode;

    }
    // }
    
}

//-*************************************************************************

void ProcessCurves( ICurves &curves, ProcArgs &args,
        MatrixSampleMap * xformSamples)
{
    SampleTimeSet sampleTimes;
    std::vector<AtPoint> vidxs;
    std::vector<float> radius;
    
    AtNode * curvesNode = ProcessCurvesBase(
            curves, args, sampleTimes, vidxs, radius, xformSamples);
    
    // This is a valid condition for the second instance onward and just
    // means that we don't need to do anything further.
    if ( !curvesNode )
    {
        return;
    }
}

