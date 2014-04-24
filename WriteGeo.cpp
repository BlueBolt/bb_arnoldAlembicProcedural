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
#include "ArbGeomParams.h"

#include <ai.h>
#include <sstream>
#include <algorithm> 

#include <boost/regex.hpp>
//-*****************************************************************************

const size_t hashStr( std::string const& s )
{
    size_t result = 2166136261U ;
    std::string::const_iterator end = s.end() ;
    for ( std::string::const_iterator iter = s.begin() ;
            iter != end ;
            ++ iter ) 
  {
        result = 127 * result
                + static_cast< unsigned char >( *iter ) ;
    }
    return result ;
 }


#if AI_VERSION_ARCH_NUM == 3
    #if AI_VERSION_MAJOR_NUM < 4
        #define AiNodeGetNodeEntry(node)   ((node)->base_node)
    #endif
#endif

bool nodeHasParameter( struct AtNode * node, const std::string & paramName)
{
    return AiNodeEntryLookUpParameter( AiNodeGetNodeEntry( node ),
            paramName.c_str() ) != NULL;
}

/*
 * Return a new string with all occurrences of 'from' replaced with 'to'
 */
std::string replace_all(const std::string &str, const char *from, const char *to)
{
    std::string result(str);
    std::string::size_type
        index = 0,
        from_len = strlen(from),
        to_len = strlen(to);
    while ((index = result.find(from, index)) != std::string::npos) {
        result.replace(index, from_len, to);
        index += to_len;
    }
    return result;
}

/*
 * Translate a shell pattern into a regular expression
 * This is a direct translation of the algorithm defined in fnmatch.py.
 */
static std::string translate(const char *pattern)
{
    int i = 0, n = strlen(pattern);
    std::string result;
 
    while (i < n) {
        char c = pattern[i];
        ++i;
 
        if (c == '*') {
            result += ".*";
        } else if (c == '?') {
            result += '.';
        } else if (c == '[') {
            int j = i;
            /*
             * The following two statements check if the sequence we stumbled
             * upon is '[]' or '[!]' because those are not valid character
             * classes.
             */
            if (j < n && pattern[j] == '!')
                ++j;
            if (j < n && pattern[j] == ']')
                ++j;
            /*
             * Look for the closing ']' right off the bat. If one is not found,
             * escape the opening '[' and continue.  If it is found, process
             * the contents of '[...]'.
             */
            while (j < n && pattern[j] != ']')
                ++j;
            if (j >= n) {
                result += "\\[";
            } else {
                std::string stuff = replace_all(std::string(&pattern[i], j - i), "\\", "\\\\");
                char first_char = pattern[i];
                i = j + 1;
                result += "[";
                if (first_char == '!') {
                    result += "^" + stuff.substr(1);
                } else if (first_char == '^') {
                    result += "\\" + stuff;
                } else {
                    result += stuff;
                }
                result += "]";
            }
        } else {
            if (isalnum(c)) {
                result += c;
            } else {
                result += "\\";
                result += c;
            }
        }
    }
    /*
     * Make the expression multi-line and make the dot match any character at all.
     */
    return result + "\\Z(?ms)";
}
 


bool matchPattern(std::string str, std::string pat)
{
    // given a path name and a pattern see if the path string matches the pattern
    bool found = false;
    std::vector<std::string> parts;
    std::string temp;
    if (pat == "*") // pattern is * 
        return true;
    if (pat.size() - 1 > str.size()) // pattern string is bigger then input string
        return false;

    std::string::const_iterator it = pat.begin(), end = pat.end();
    size_t counter = 0;
    while (it != end)
    {
        if (*it != '*')
            temp += *it;
        if (*it == '*')
        {
            parts.push_back(temp);
            temp = "";
        }
        it++;
    }
    parts.push_back(temp);
    std::vector<std::string>::const_iterator vecIt = parts.begin(), vecEnd = parts.end();
    if (!parts[0].empty())
    {
        if (parts[0] != str.substr(0, parts[0].size()))
            return false;
    }
    else
        vecIt++;
    size_t size = str.size();
    size_t pos = 0;
    size_t tempSize;
    while (vecIt != vecEnd)
    {
        temp = *vecIt;
        tempSize = temp.size();
        if (temp.empty())
            return true;
        while (pos + tempSize < size)
        {
            if (temp == str.substr(pos, temp.size()))
            {
                str.erase(0, pos + temp.size());
                found = true;
                break;
            }
            pos++;
        }
        if (found == false)
            return false;
        pos = 0;
        vecIt++;
    }
    return true;
}

bool matchPattern2(std::string str, std::string pat)
{
    boost::regex rx (translate(pat.c_str()).c_str());
    bool result = boost::regex_search(str,rx);
    return result;
}

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

    if ( !matchPattern2(name,args.pattern) && args.pattern != "*" && args.pattern != "" )
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
        else if(matchPattern2(name,it->first)) // based on wildcard expression
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
      for(std::vector<std::string>::iterator it=args.overrides.begin(); it!=args.overrides.end(); ++it)
      {
        Json::Value overrides;
        if(it->find("/") != string::npos)
        {
          if(name.find(*it) != string::npos)
          {
            overrides = args.overrideRoot[*it];
          }

        }
        else if(matchPattern2(name,*it)) // based on wildcard expression
        {
            overrides = args.overrideRoot[*it];
        }
        else
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
              || attribute=="subdiv_iterations" 
              || attribute=="subdiv_type"
              || attribute=="subdiv_adaptive_metric"
              || attribute=="subdiv_uv_smoothing"
              || attribute=="subdiv_pixel_error"
              || attribute=="disp_height"
              || attribute=="disp_padding"
              || attribute=="disp_zero_value"
              || attribute=="disp_autobump")
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
    
    // detect if this node is already in the scene if so generate a ginstance and apply the transform to it

    AtNode *masterProc = AiNodeLookUpByName(name.c_str());
    // if(masterProc)
    // {

    //   instanceNode = AiNode( "ginstance" );

    //   // get the name for this instance
      
    //   char name_buff[1024]; // name buffer
      
    //   getInstanceName(name_buff, sizeof(name_buff), name.c_str());

    //   AiNodeSetStr( instanceNode, "name", name_buff );
    //   AiNodeSetPtr(instanceNode, "node", masterProc);
    //   AiNodeSetBool(instanceNode, "inherit_xform", false);
      
    //   AtMatrix matrix;
    //   AtVector scaleVector;
    //   AiV3Create(scaleVector, 1, 1, 1);
    //   AiM4Scaling(matrix, &scaleVector);
    //   AiNodeSetMatrix(instanceNode, "matrix", matrix);

    //   if ( args.proceduralNode )
    //   {
    //       AiNodeSetByte( instanceNode, "visibility",
    //       AiNodeGetByte( args.proceduralNode, "visibility" ) );
    //   }
    //   else
    //   {
    //       AiNodeSetByte( instanceNode, "visibility", AI_RAY_ALL );
    //   }

    //   //apply the transform to this instance
    //   ApplyTransformation( instanceNode, xformSamples, args );

    //   // add this guy to the list of created nodes
    //   args.createdNodes.push_back(instanceNode);

    //   // return instanceNode;
    // }

    // } else {

      // We may want to move this out of Write Geo for polymesh 
      // so we can use it for the other geo types.  
      
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
        
        buffer << "@" << hashStr(hashAttributes);
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

        AddArbitraryGeomParams(
            arbGeomParams,
            frameSelector,
                instanceNode );

        NodeCache::iterator I = g_meshCache.find(cacheId);

        // start param overrides on instance
        if(args.linkOverride)
        {
          bool foundInPath = false;
          for(std::vector<std::string>::iterator it=args.overrides.begin(); it!=args.overrides.end(); ++it)
          {
            Json::Value overrides;                        
            if(it->find("/") != string::npos) // Based on path
            {
              if(name.find(*it) != string::npos)
              {
                overrides = args.overrideRoot[*it];
                foundInPath = true;
              }
            } 
            else if(matchPattern2(name,*it)) // based on wildcard expression
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
                const AtNodeEntry* nodeEntry = AiNodeGetNodeEntry(instanceNode);
                const AtParamEntry* paramEntry = AiNodeEntryLookUpParameter(nodeEntry, attribute.c_str());

                if ( paramEntry != NULL)
                {
                  Json::Value val = args.overrideRoot[*it][itr.key().asString()];
                  if( val.isString() ) 
                    AiNodeSetStr(instanceNode, attribute.c_str(), val.asCString());
                  else if( val.isBool() ) 
                    AiNodeSetBool(instanceNode, attribute.c_str(), val.asBool());
                  else if( val.isInt() ) 
                  {
                    //make the difference between Byte & int!
                    int typeEntry = AiParamGetType(paramEntry);
                    if(typeEntry == AI_TYPE_BYTE)
                    { 
                      if(attribute=="visibility")
                      {
                        AtByte attrViz = val.asInt();
                        // special case, we must determine it against the general viz.
                        AtByte procViz = AiNodeGetByte( args.proceduralNode, "visibility" );
                        AtByte compViz = AI_RAY_ALL;
                        {
                          compViz &= ~AI_RAY_GLOSSY;
                          if(procViz > compViz)
                            procViz &= ~AI_RAY_GLOSSY;
                          else
                            attrViz &= ~AI_RAY_GLOSSY;
                          compViz &= ~AI_RAY_DIFFUSE;
                          if(procViz > compViz)
                            procViz &= ~AI_RAY_DIFFUSE;
                          else
                            attrViz &= ~AI_RAY_DIFFUSE;
                          compViz &= ~AI_RAY_REFRACTED;
                          if(procViz > compViz)
                            procViz &= ~AI_RAY_REFRACTED;
                          else
                            attrViz &= ~AI_RAY_REFRACTED;
                          compViz &= ~AI_RAY_REFLECTED;
                          if(procViz > compViz)
                            procViz &= ~AI_RAY_REFLECTED;
                          else
                            attrViz &= ~AI_RAY_REFLECTED;
                          compViz &= ~AI_RAY_SHADOW;
                          if(procViz > compViz)
                            procViz &= ~AI_RAY_SHADOW;
                          else
                            attrViz &= ~AI_RAY_SHADOW;
                          compViz &= ~AI_RAY_CAMERA;
                          if(procViz > compViz)
                            procViz &= ~AI_RAY_CAMERA;
                          else
                            attrViz &= ~AI_RAY_CAMERA;
                        }

                        AiNodeSetByte(instanceNode, attribute.c_str(), attrViz);
                      }
                      else
                        AiNodeSetByte(instanceNode, attribute.c_str(), val.asInt());
                    }
                    else 
                      AiNodeSetInt(instanceNode, attribute.c_str(), val.asInt());
                  }
                  else if( val.isUInt() ) 
                    AiNodeSetUInt(instanceNode, attribute.c_str(), val.asUInt());
                  else if( val.isDouble() ) 
                    AiNodeSetFlt(instanceNode, attribute.c_str(), val.asDouble());
                }
              }
            }

          }
        } // end param overrides


        // shader assignation
        if (nodeHasParameter( instanceNode, "shader" ) )
        {
          if(args.linkShader)
          {
            bool foundInPath = false;
            AtNode* appliedShader = NULL;
            for(std::map<std::string, AtNode*>::iterator it = args.shaders.begin(); it != args.shaders.end(); ++it) 
            {

              //check both path & tag
              if(it->first.find("/") != string::npos)
              {
                if(name.find(it->first) != string::npos)
                {
                  appliedShader = it->second;
                  foundInPath = true;
                }
              }
              else if(matchPattern2(name,it->first)) // based on wildcard expression
              {

                 AiMsgDebug("[ABC] Shader pattern '%s' matched %s",it->first.c_str(), name.c_str());
                 appliedShader = it->second;
                 foundInPath = true;
              }
              else if(foundInPath == false)
              {
                if (std::find(tags.begin(), tags.end(), it->first) != tags.end())
                  appliedShader = it->second;
              }
            }

            if(appliedShader != NULL)
            {
              AiMsgDebug("[ABC] Assigning shader  %s to %s", AiNodeGetName(appliedShader), AiNodeGetName(instanceNode));
              AtArray* shaders = AiArrayAllocate( 1 , 1, AI_TYPE_NODE);
              AiArraySetPtr(shaders, 0, appliedShader);
              AiNodeSetArray(instanceNode, "shader", shaders);
            }
            else
            {
              AtArray* shaders = AiNodeGetArray(args.proceduralNode, "shader");
              if (shaders->nelements != 0)
                 AiNodeSetArray(instanceNode, "shader", AiArrayCopy(shaders));
            }

          }
          else
          {
            AtArray* shaders = AiNodeGetArray(args.proceduralNode, "shader");
            if (shaders->nelements != 0)
               AiNodeSetArray(instanceNode, "shader", AiArrayCopy(shaders));
          }
        } // end shader assignment

        if ( masterProc )
        {
           AiNodeSetPtr(instanceNode, "node", masterProc );
           return NULL;
        } else if ( I != g_meshCache.end() ) {
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
              vidxs.insert( vidxs.end(), sample.getFaceIndices()->get(),
                      sample.getFaceIndices()->get() + vidxSize );
          }


          vlist.reserve( vlist.size() + sample.getPositions()->size() * 3);
          vlist.insert( vlist.end(),
                  (const float32_t*) sample.getPositions()->get(),
                  ((const float32_t*) sample.getPositions()->get()) +
                          sample.getPositions()->size() * 3 );
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
          if(name.find(*it) != string::npos || std::find(tags.begin(), tags.end(), *it) != tags.end())
          {
            const Json::Value overrides = args.overrideRoot[*it];
            if(overrides.size() > 0)
            {
              for( Json::ValueIterator itr = overrides.begin() ; itr != overrides.end() ; itr++ ) 
              {
                std::string attribute = itr.key().asString();

                if (attribute=="smoothing" 
                  || attribute=="subdiv_iterations" 
                  || attribute=="subdiv_type"
                  || attribute=="subdiv_adaptive_metric"
                  || attribute=="subdiv_uv_smoothing"
                  || attribute=="subdiv_pixel_error"
                  || attribute=="disp_height"
                  || attribute=="disp_padding"
                  || attribute=="disp_zero_value"
                  || attribute=="disp_autobump")
                {
                  AiMsgDebug("Checking attribute %s for shape %s", attribute.c_str(), name.c_str());
                  // check if the attribute exists ...
                  const AtNodeEntry* nodeEntry = AiNodeGetNodeEntry(meshNode);
                  const AtParamEntry* paramEntry = AiNodeEntryLookUpParameter(nodeEntry, attribute.c_str());

                  if ( paramEntry != NULL)
                  {
                    AiMsgDebug("attribute %s exists on shape", attribute.c_str());
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
           AiNodeSetArray(meshNode, "uvidxs",
                   AiArrayConvert(uvidxs.size(), 1, AI_TYPE_UINT,
                           &(uvidxs[0])));
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

      AiNodeSetBool( meshNode, "smoothing", true );

      if (subdiv_iterations > 0)
        {

          AiNodeSetStr( meshNode, "subdiv_type", "catclark" );
          AiNodeSetInt( meshNode, "subdiv_iterations", args.subdIterations );
          AiNodeSetStr( meshNode, "subdiv_uv_smoothing", args.subdUVSmoothing.c_str() );
        }


      // add as switch
      AiNodeSetBool( meshNode, "invert_normals", args.invertNormals );

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

    AiNodeSetBool(meshNode, "smoothing", true);
  
    ProcessIndexedBuiltinParam(
            ps.getNormalsParam(),
            sampleTimes,
            nlist,
            nidxs,
            3);
    


    // if ( !nlist.empty() )
    // {
    //     AiNodeSetArray(meshNode, "nlist",
    //         AiArrayConvert( nlist.size() / sampleTimes.size(),
    //                 sampleTimes.size(), AI_TYPE_FLOAT, (void*)(&(nlist[0]))));

    //     if ( !nidxs.empty() )
    //     {

    //        // we must invert the idxs
    //        //unsigned int facePointIndex = 0;
    //        unsigned int base = 0;
    //        AtArray* nsides = AiNodeGetArray(meshNode, "nsides");
    //    std::vector<unsigned int> nvidxReversed;
    //        for (unsigned int i = 0; i < nsides->nelements / nsides->nkeys; ++i)
    //        {
    //           int curNum = AiArrayGetUInt(nsides ,i);
        
    //           for (int j = 0; j < curNum; ++j)
    //     {
    //       nvidxReversed.push_back(nidxs[base+curNum-j-1]);
    //     }
    //           base += curNum;
    //        }
    //         AiNodeSetArray(meshNode, "nidxs", AiArrayConvert(nvidxReversed.size(), 1, AI_TYPE_UINT, (void*)&nvidxReversed[0]));
    //     }
    //     else
    //     {
    //         AiNodeSetArray(meshNode, "nidxs",
    //                 AiArrayConvert(vidxs.size(), 1, AI_TYPE_UINT,
    //                         &(vidxs[0])));
    //     }
    // }    

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


void ProcessPoints( IPoints &points, ProcArgs &args)
{
    SampleTimeSet sampleTimes;
    std::vector<AtUInt32> pts;
        
    // This is a valid condition for the second instance onward and just
    // means that we don't need to do anything further.
    
    return;
}



void ProcessCurves( IPoints &points, ProcArgs &args)
{
    SampleTimeSet sampleTimes;
    std::vector<AtUInt32> pts;
        
    // This is a valid condition for the second instance onward and just
    // means that we don't need to do anything further.
    
    return;
}

