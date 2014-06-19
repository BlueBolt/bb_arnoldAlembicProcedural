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

void getInstanceName(char *buf, size_t bufsize, const char* name)
{
   int index = 0;
   while(1){
       sprintf(buf,"%s_instance_%05d", name, index);
       AtNode *tmp = AiNodeLookUpByName(buf);
       if(!tmp){
           return;
       }
       index++;
   }
}

//-*****************************************************************************


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


//-*************************************************************************
// This is templated to handle shared behavior of IPolyMesh and ISubD

// We send in our empty sampleTimes and vidxs because polymesh needs those
// for processing animated normal.


// The return value is the polymesh node. If instanced, it will be returned
// for the first created instance only.
template <typename primT>
AtNode * ProcessPolyMeshBase(
        primT & prim, ProcArgs & args,
        SampleTimeSet & sampleTimes,
        std::vector<AtUInt32> & vidxs,
        int subdiv_iterations,
        MatrixSampleMap * xformSamples, 
        const std::string & facesetName = "" )
{
    if ( !prim.valid() )
    {
        return NULL;
    }
    

    // Get the tiome samples for this geo

    typename primT::schema_type  &ps = prim.getSchema();
    TimeSamplingPtr ts = ps.getTimeSampling();
    
    if ( ps.getTopologyVariance() != kHeterogenousTopology )
    {
        GetRelevantSampleTimes( args, ts, ps.getNumSamples(), sampleTimes );
    }
    else
    {
        sampleTimes.insert( ( args.frame + args.frameOffset ) / args.fps );
    }
    
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

    std::string cacheId;
    
    SampleTimeSet singleSampleTimes;
    singleSampleTimes.insert( ( args.frame + args.frameOffset ) / args.fps );

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

    // displacement stuff
    AtNode* appliedDisplacement = NULL;
    if(args.linkDisplacement)
    {
      bool foundInPath = false;
      for(std::map<std::string, AtNode*>::iterator it = args.displacements.begin(); it != args.displacements.end(); ++it) 
      {
        //check both path & tag
        if(it->first.find("/") != string::npos)
        {
          if(name.find(it->first) != string::npos)
          {
            appliedDisplacement = it->second;
            foundInPath = true;
          }

        }
        else if(matchPattern(name,it->first)) // based on wildcard expression
        {
            appliedDisplacement = it->second;
            foundInPath = true;
        }
        else if(foundInPath == false)
        {
          if (std::find(tags.begin(), tags.end(), it->first) != tags.end())
          {
            appliedDisplacement = it->second;
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
              || attribute=="subdiv_iterations" 
              || attribute=="subdiv_type"
              || attribute=="subdiv_adaptive_metric"
              || attribute=="subdiv_uv_smoothing"
              || attribute=="subdiv_pixel_error"
              || attribute=="disp_height"
              || attribute=="disp_padding"
              || attribute=="disp_zero_value"
              || attribute=="disp_autobump"
              || attribute=="invert_normals")
            {
              Json::Value val = args.overrideRoot[*it][itr.key().asString()];
              rootEncode[attribute]=val;
            }
          }
        }
      }
    }

    if(appliedDisplacement != NULL)
    {
      rootEncode["disp_shader"] = std::string(AiNodeGetName(appliedDisplacement));

    }

    hashAttributes += writer.write(rootEncode);

    AtNode * instanceNode = NULL;
    
    if ( args.makeInstance  )
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
        
        buffer << "@" << hash(hashAttributes);
        buffer << "@" << facesetName;
        
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
    
    std::vector<AtByte> nsides;
    std::vector<float> vlist;

    std::vector<float> uvlist;
    std::vector<unsigned int> uvidxs;

    // POTENTIAL OPTIMIZATIONS LEFT TO THE READER
    // 1) vlist needn't be copied if it's a single sample

    size_t numSampleTimes = sampleTimes.size();
    bool isFirstSample = true;
    for ( SampleTimeSet::iterator I = sampleTimes.begin();
        I != sampleTimes.end(); ++I, isFirstSample = false)
    {
        ISampleSelector sampleSelector( *I );
        typename primT::schema_type::Sample sample = ps.getValue( sampleSelector );

        if ( isFirstSample )
        {
            size_t numPolys = sample.getFaceCounts()->size();
            nsides.reserve( sample.getFaceCounts()->size() );
            for ( size_t i = 0; i < numPolys; ++i )
            {
              int32_t n = sample.getFaceCounts()->get()[i];

              if ( n > 255 )
              {
                  // TODO, warning about unsupported face
                  return NULL;
              }

              nsides.push_back( (AtByte) n );
            }

            size_t vidxSize = sample.getFaceIndices()->size();
            vidxs.reserve( vidxSize );

            unsigned int facePointIndex = 0;
            unsigned int base = 0;

            for (unsigned int i = 0; i < numPolys; ++i)
            {
               // reverse the order of the faces
               int curNum = nsides[i];
               for (int j = 0; j < curNum; ++j, ++facePointIndex)
               {
                  vidxs.push_back((*sample.getFaceIndices())[base+curNum-j-1]);

               }
               base += curNum;
            }
        }
  
        if(numSampleTimes == 1 && (args.shutterOpen != args.shutterClose) && (ps.getVelocitiesProperty().valid()) && isFirstSample )
        {
            float scaleVelocity = 1.0f;
            if (AiNodeLookUpUserParameter(args.proceduralNode, "scaleVelocity") !=NULL )
                scaleVelocity = AiNodeGetFlt(args.proceduralNode, "scaleVelocity");

            Alembic::Abc::V3fArraySamplePtr velptr = sample.getVelocities();
            Alembic::Abc::P3fArraySamplePtr v3ptr = sample.getPositions();
            size_t pSize = sample.getPositions()->size(); 
            vlist.resize(pSize*3*2);
            numSampleTimes = 2;
            float timeoffset = ((args.frame / args.fps) - ts->getFloorIndex((*I), ps.getNumSamples()).second) * args.fps;
            for ( size_t vId = 0; vId < pSize; ++vId )
            {
                
                Alembic::Abc::V3f posAtOpen = ((*v3ptr)[vId] + (*velptr)[vId] * scaleVelocity *-timeoffset);
                vlist[3*vId + 0] = posAtOpen.x;
                vlist[3*vId + 1] = posAtOpen.y;
                vlist[3*vId + 2] = posAtOpen.z;

                Alembic::Abc::V3f posAtEnd = ((*v3ptr)[vId] + (*velptr)[vId] * scaleVelocity *(1.0f-timeoffset));
                vlist[3*vId + 3*pSize + 0] = posAtEnd.x;
                vlist[3*vId + 3*pSize + 1] = posAtEnd.y;
                vlist[3*vId + 3*pSize + 2] = posAtEnd.z;            
            }
        }
        else
        {
            vlist.reserve( vlist.size() + sample.getPositions()->size() * 3);
            vlist.insert( vlist.end(),
                    (const float32_t*) sample.getPositions()->get(),
                    ((const float32_t*) sample.getPositions()->get()) +
                            sample.getPositions()->size() * 3 );
        }
    }

    /* Apply uvs to geo */
    /* TODO add uv archive as an input */
    ProcessIndexedBuiltinParam(
          ps.getUVsParam(),
          singleSampleTimes,
          uvlist,
          uvidxs,
          2);

    AtNode* meshNode = AiNode( "polymesh" );

    if (!meshNode)
    {
      AiMsgError("Failed to make polymesh node for %s",
              prim.getFullName().c_str());
      return NULL;
    }

    args.createdNodes.push_back(meshNode);

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
              || attribute=="subdiv_iterations" 
              || attribute=="subdiv_type"
              || attribute=="subdiv_adaptive_metric"
              || attribute=="subdiv_uv_smoothing"
              || attribute=="subdiv_pixel_error"
              || attribute=="disp_height"
              || attribute=="disp_padding"
              || attribute=="disp_zero_value"
              || attribute=="disp_autobump"
              || attribute=="invert_normals")
            {
              // check if the attribute exists ...
              const AtNodeEntry* nodeEntry = AiNodeGetNodeEntry(meshNode);
              const AtParamEntry* paramEntry = AiNodeEntryLookUpParameter(nodeEntry, attribute.c_str());

              if ( paramEntry != NULL)
              {
                AiMsgDebug("[ABC] attribute %s exists on shape", attribute.c_str());
                Json::Value val = args.overrideRoot[*it][itr.key().asString()];
                if( val.isString() ) 
                  AiNodeSetStr(meshNode, attribute.c_str(), val.asCString());
                else if( val.isBool() ) 
                  AiNodeSetBool(meshNode, attribute.c_str(), val.asBool());
                else if( val.isInt() ) 
                {
                  //make the difference between Byte & int!
                  int typeEntry = AiParamGetType(paramEntry);
                  if(typeEntry == AI_TYPE_BYTE)
                    AiNodeSetByte(meshNode, attribute.c_str(), val.asInt());
                  else 
                    AiNodeSetInt(meshNode, attribute.c_str(), val.asInt());
                }
                else if( val.isUInt() ) 
                  AiNodeSetUInt(meshNode, attribute.c_str(), val.asUInt());
                else if( val.isDouble() ) 
                  AiNodeSetFlt(meshNode, attribute.c_str(), val.asDouble());
              }
            }
          }
        }
      }
    }
    }

    // displaces assignation

    if(appliedDisplacement!= NULL)
    AiNodeSetPtr(meshNode, "disp_map", appliedDisplacement);

    if ( instanceNode != NULL)
    {
      AiNodeSetStr( meshNode, "name", (name + ":src").c_str() );
    }
    else
    {
      AiNodeSetStr( meshNode, "name", name.c_str() );
    }

    AiNodeSetArray(meshNode, "vidxs",
          AiArrayConvert(vidxs.size(), 1, AI_TYPE_UINT,
                  (void*)&vidxs[0]));

    AiNodeSetArray(meshNode, "nsides",
          AiArrayConvert(nsides.size(), 1, AI_TYPE_BYTE,
                  &(nsides[0])));

    AiNodeSetArray(meshNode, "vlist",
          AiArrayConvert( vlist.size() / sampleTimes.size(),
                  sampleTimes.size(), AI_TYPE_FLOAT, (void*)(&(vlist[0]))));

    if ( !uvlist.empty() )
    {
     AiMsgDebug( "[ABC] Flipping in V %s",args.flipv ? "true" : "false"); 
     if (args.flipv)
     {

        for (size_t i = 1, e = uvlist.size(); i < e; i += 2)
        {
          uvlist[i] = 1.0 - uvlist[i];
        }            
     }

     AiMsgDebug( "[ABC] assigning UVs %s",name.c_str()); 

     // realocate the uvs to a AiArrayFlt array

     AtArray* a_uvlist = AiArrayAllocate( uvlist.size() , 1, AI_TYPE_FLOAT);

     for (unsigned int i = 0; i < uvlist.size() ; ++i)
     {
      AiArraySetFlt(a_uvlist, i, uvlist[i]);
     }

     AiNodeSetArray(meshNode, "uvlist", a_uvlist);

     if ( !uvidxs.empty() )
     {
       // we must invert the idxs

       unsigned int facePointIndex = 0;
       unsigned int base = 0;

       AtArray* uvidxReversed = AiArrayAllocate(uvidxs.size(), 1, AI_TYPE_UINT);

            //AiMsgInfo("sampleTimes.size() %i", sampleTimes.size());

       for (unsigned int i = 0; i < nsides.size() ; ++i)
       {
          int curNum = nsides[i];
          for (int j = 0; j < curNum; ++j, ++facePointIndex)
             AiArraySetUInt(uvidxReversed, facePointIndex, uvidxs[base+curNum-j-1]);


          base += curNum;
       }

        AiNodeSetArray(meshNode, "uvidxs",
              uvidxReversed);
     }
     else
     {
       AiNodeSetArray(meshNode, "uvidxs",
               AiArrayConvert(vidxs.size(), 1, AI_TYPE_UINT,
                       &(vidxs[0])));
     }
    }

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

      AiNodeSetArray( meshNode, "deform_time_samples",
              AiArrayConvert(relativeSampleTimes.size(), 1,
                      AI_TYPE_FLOAT, &relativeSampleTimes[0]));
    }
    else if(numSampleTimes == 2)
    {
        AiNodeSetArray( meshNode, "deform_time_samples",
                AiArray(2, 1, AI_TYPE_FLOAT, 0.f, 1.f));
    }

    // faceset visibility array
    if ( !facesetName.empty() )
    {
      if ( ps.hasFaceSet( facesetName ) )
      {
          ISampleSelector frameSelector( *singleSampleTimes.begin() );


          IFaceSet faceSet = ps.getFaceSet( facesetName );
          IFaceSetSchema::Sample faceSetSample =
                  faceSet.getSchema().getValue( frameSelector );

          std::set<int> facesToKeep;


          facesToKeep.insert( faceSetSample.getFaces()->get(),
                  faceSetSample.getFaces()->get() +
                          faceSetSample.getFaces()->size() );

          std::vector<bool> faceVisArray;
          faceVisArray.reserve( nsides.size() );

          for ( int i = 0; i < (int) nsides.size(); ++i )
          {
              faceVisArray.push_back(
                      facesToKeep.find( i ) != facesToKeep.end() );
          }

          if ( AiNodeDeclare( meshNode, "face_visibility", "uniform BOOL" ) )
          {
              AiNodeSetArray( meshNode, "face_visibility",
                      AiArrayConvert( faceVisArray.size(), 1, AI_TYPE_BOOLEAN,
                              (void *) &faceVisArray[0] ) );
          }
      }
    }

    {
        ICompoundProperty arbGeomParams = ps.getArbGeomParams();
        ISampleSelector frameSelector( *singleSampleTimes.begin() );

        /* add user attributs to the current object */
        AddArbitraryGeomParams( arbGeomParams, frameSelector, meshNode );
    }

    // AiNodeSetBool( meshNode, "smoothing", true ); // disbled to allow the mesh to have hard edges

    if (subdiv_iterations > 0)
    {
        AiNodeSetStr( meshNode, "subdiv_type", "catclark" );
        AiNodeSetInt( meshNode, "subdiv_iterations", args.subdIterations );
        AiNodeSetStr( meshNode, "subdiv_uv_smoothing", args.subdUVSmoothing.c_str() );
    }


    // add as switch
    if ( args.invertNormals )
    {
        AiNodeSetBool( meshNode, "invert_normals", args.invertNormals );
    }

    if ( args.disp_map != "" )
    {
        AtNode* disp_node = AiNodeLookUpByName(args.disp_map.c_str());
        AiNodeSetPtr( meshNode, "disp_map", disp_node );
    }

    if ( instanceNode == NULL )
    {
      if ( xformSamples )
      {
          ApplyTransformation( meshNode, xformSamples, args );
      }
              // shader assignation
      if (nodeHasParameter( meshNode, "shader" ) )
      {
        if(args.linkShader)
        {
          ApplyShaders(name, meshNode, tags, args);
        }
        else
        {
          AtArray* shaders = AiNodeGetArray(args.proceduralNode, "shader");
          if (shaders->nelements != 0)
             AiNodeSetArray(meshNode, "shader", AiArrayCopy(shaders));
        }
      } // end shader assignment

      return meshNode;
    }
    else
    {
      AiNodeSetByte( meshNode, "visibility", 0 ); // had original node that is being referenced

      AiNodeSetPtr(instanceNode, "node", meshNode );
      g_meshCache[cacheId] = meshNode;
      return meshNode;

    }
    // }
    
}

//-*************************************************************************

void ProcessPolyMesh( IPolyMesh &polymesh, ProcArgs &args,
        MatrixSampleMap * xformSamples, const std::string & facesetName )
{
    SampleTimeSet sampleTimes;
    std::vector<AtUInt32> vidxs;
    
    AtNode * meshNode = ProcessPolyMeshBase(
            polymesh, args, sampleTimes, vidxs, 0, xformSamples,
                    facesetName );
    
    // This is a valid condition for the second instance onward and just
    // means that we don't need to do anything further.
    if ( !meshNode )
    {
        return;
    }

  
    IPolyMeshSchema &ps = polymesh.getSchema();

    std::vector<float> nlist;
    std::vector<unsigned int> nidxs;

    // AiNodeSetBool(meshNode, "smoothing", true); // Disabled for now so meshes can have hard edges

    //TODO: better check
    if (AiNodeGetArray(meshNode, "vlist")->nkeys == sampleTimes.size())
    {
        ProcessIndexedBuiltinParam(
                ps.getNormalsParam(),
                sampleTimes,
                nlist,
                nidxs,
                3);
    
    }

    // check that the meshNode has normals and is not  a 
    if ( !nlist.empty() && AiNodeGetStr(meshNode, "subdiv_type") == "none" )
    {
        AiNodeSetArray(meshNode, "nlist",
            AiArrayConvert( nlist.size() / sampleTimes.size(),
                    sampleTimes.size(), AI_TYPE_FLOAT, (void*)(&(nlist[0]))));

        if ( !nidxs.empty() )
        {

           // we must invert the idxs
           //unsigned int facePointIndex = 0;
           unsigned int base = 0;
           AtArray* nsides = AiNodeGetArray(meshNode, "nsides");
           std::vector<unsigned int> nvidxReversed;
           for (unsigned int i = 0; i < nsides->nelements / nsides->nkeys; ++i)
           {
              int curNum = AiArrayGetUInt(nsides ,i);
              
              for (int j = 0; j < curNum; ++j)
              {
                  nvidxReversed.push_back(nidxs[base+curNum-j-1]);
              }
              base += curNum;
           }
            AiNodeSetArray(meshNode, "nidxs", AiArrayConvert(nvidxReversed.size(), 1, AI_TYPE_UINT, (void*)&nvidxReversed[0]));
        }
        else
        {
            AiNodeSetArray(meshNode, "nidxs",
                    AiArrayConvert(vidxs.size(), 1, AI_TYPE_UINT,
                            &(vidxs[0])));
        }
    }
}

//-*************************************************************************

void ProcessSubD( ISubD &subd, ProcArgs &args,
        MatrixSampleMap * xformSamples, const std::string & facesetName )
{
    SampleTimeSet sampleTimes;
    std::vector<AtUInt32> vidxs;
    
    AtNode * meshNode = ProcessPolyMeshBase(
            subd, args, sampleTimes, vidxs, args.subdIterations,
                    xformSamples, facesetName );
    
    // This is a valid condition for the second instance onward and just
    // means that we don't need to do anything further.
    if ( !meshNode )
    {
        return;
    }

    AiNodeSetStr( meshNode, "subdiv_type", "catclark" ); // quick override

}
