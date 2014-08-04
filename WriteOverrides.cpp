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

#include "PathUtil.h"
#include "WriteOverrides.h"
#include "ArbGeomParams.h"
#include <ai.h>
#include <sstream>

#include "json/json.h"



void ApplyOverrides(std::string name, AtNode* node, std::vector<std::string> tags, ProcArgs & args)
{
   bool foundInPath = false;
   for(std::vector<std::string>::iterator it=args.overrides.begin(); it!=args.overrides.end(); ++it)
   {
      Json::Value overrides;                        
      if(it->find("/") != std::string::npos) // Based on path
      {
        if(name.find(*it) != std::string::npos)
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

          const AtNodeEntry* nodeEntry = AiNodeGetNodeEntry(node);
          const AtParamEntry* paramEntry = AiNodeEntryLookUpParameter(nodeEntry, attribute.c_str());

          if ( paramEntry != NULL && attribute!="invert_normals" || attribute == "matte")
          {

            Json::Value val = args.overrideRoot[*it][itr.key().asString()];
            if( val.isString() ) 
              AiNodeSetStr(node, attribute.c_str(), val.asCString());
            else if( val.isBool() )
            {
              if(attribute == "matte")
              {
                  AiMsgDebug("[ABC] adding enable_matte to %s", AiNodeGetName(node));
                  AddUserGeomParams(node,"enable_matte",AI_TYPE_BOOLEAN);
                  AiNodeSetBool(node,"enable_matte", val.asBool());
              }
              else
              {
                  AiNodeSetBool(node, attribute.c_str(), val.asBool());
              }
            }
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

                  AiNodeSetByte(node, attribute.c_str(), attrViz);
                }
                else
                  AiNodeSetByte(node, attribute.c_str(), val.asInt());
              }
              else 
                AiNodeSetInt(node, attribute.c_str(), val.asInt());
            }
            else if( val.isUInt() ) 
              AiNodeSetUInt(node, attribute.c_str(), val.asUInt());
            else if( val.isDouble() ) 
              AiNodeSetFlt(node, attribute.c_str(), val.asDouble());
          }
        }
      }

   }
}

void ApplyUserAttributes(std::string name, AtNode* node,std::vector<std::string> tags,ProcArgs & args)
{

   bool foundInPath = false;
   for(std::vector<std::string>::iterator it=args.userAttributes.begin(); it!=args.userAttributes.end(); ++it)
   {
      Json::Value userAttributes;                        
      if(it->find("/") != std::string::npos) // Based on path
      {
        if(name.find(*it) != std::string::npos)
        {
          userAttributes = args.userAttributesRoot[*it];
          foundInPath = true;
        }
      } 
      else if(matchPattern(name,*it)) // based on wildcard expression
      {
          userAttributes = args.userAttributesRoot[*it];
          foundInPath = true;
      }
      else if(foundInPath == false)
      {
        if (std::find(tags.begin(), tags.end(), *it) != tags.end())
        {
          userAttributes = args.userAttributesRoot[*it];
        }
      }

      if(userAttributes.size() > 0)
      {
        for( Json::ValueIterator itr = userAttributes.begin() ; itr != userAttributes.end() ; itr++ ) 
        {
            std::string attribute = itr.key().asString();

            if( AiNodeLookUpUserParameter(node,attribute.c_str()))
              continue;

            const AtNodeEntry* nodeEntry = AiNodeGetNodeEntry(node);
            Json::Value val = args.userAttributesRoot[*it][attribute];
            if( val.isString() ) 
            {
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_STRING);
              AiNodeSetStr(node, attribute.c_str(), val.asCString());
            } 
            else if( val.isBool() ) 
            {
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_BOOLEAN);
              AiNodeSetBool(node, attribute.c_str(), val.asBool());
            }
            else if( val.isInt() ) 
            {
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_INT);
              AiNodeSetInt(node, attribute.c_str(), val.asInt());
            }
            else if( val.isUInt() ) 
            {
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_UINT);
              AiNodeSetUInt(node, attribute.c_str(), val.asUInt());
            }
            else if(val.isDouble())
            {
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_FLOAT);
              AiNodeSetFlt(node, attribute.c_str(), val.asDouble());
            }
            else if(val.isArray()) 
            {              

              // in the future we will convert to an arnold array type for now lets just 
              // write out a json string

              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_STRING);
              Json::FastWriter writer;
              AiNodeSetStr(node, attribute.c_str(), writer.write(val).c_str());
              
              // AddUserGeomParams(node,attribute.c_str(),AI_TYPE_ARRAY );

              // // get the type of the first entry, this will be our key as to
              // // what type of data is in this array
              // Json::Value firstValue = val[0];
              // if (firstValue.isString())
              // {
              //   AtArray* arrayValues = AiArrayAllocate( val.size() , 1, AI_TYPE_STRING);

              //   for( uint idx = 0 ; idx != val.size() ; idx++ ) 
              //   {
              //     AiMsgInfo("[ABC] adding string %s to user array attribute '%s'",val[idx].asCString(),attribute.c_str());
              //     AiArraySetStr(arrayValues,idx,val[idx].asCString());
              //   }

              //   AiNodeSetArray(node, attribute.c_str(), arrayValues);         
              // }
     
            }

            // TODO color, matrix, vector

         }
      }
   }
}

void ApplyShaders(std::string name, AtNode* node, std::vector<std::string> tags, ProcArgs & args)
{
   bool foundInPath = false;
   AtNode* appliedShader = NULL;
   for(std::map<std::string, AtNode*>::iterator it = args.shaders.begin(); it != args.shaders.end(); ++it) 
   {

     //check both path & tag
     if(it->first.find("/") != std::string::npos)
     {
       if(name.find(it->first) != std::string::npos)
       {
         appliedShader = it->second;
         foundInPath = true;
       }
     }
     else if(matchPattern(name,it->first)) // based on wildcard expression
     {

        AiMsgDebug("[ABC] Shader pattern '%s' matched %s",it->first.c_str(), name.c_str());
        appliedShader = it->second;
        foundInPath = true;
     }
     else if(foundInPath == false)
     {
       if (std::find(tags.begin(), tags.end(), it->first) != tags.end())
       {
         AiMsgDebug("[ABC] Shader tag '%s' matched tag on %s",it->first.c_str(), name.c_str());
         appliedShader = it->second;
       }
     }
   }

   if(appliedShader != NULL)
   {
     std::string newName = std::string(AiNodeGetName(appliedShader)) + std::string("_") + name;
     AtArray* shaders = AiArrayAllocate( 1 , 1, AI_TYPE_NODE);
     
     AiMsgDebug("[ABC] Assigning shader  %s to %s", AiNodeGetName(appliedShader), AiNodeGetName(node));
     AiArraySetPtr(shaders, 0, appliedShader);
     
     AiNodeSetArray(node, "shader", shaders);
   }
   else
   {
     AtArray* shaders = AiNodeGetArray(args.proceduralNode, "shader");
     if (shaders->nelements != 0)
        AiNodeSetArray(node, "shader", AiArrayCopy(shaders));
   }
}

void ApplyDisplacement(std::string name, AtNode* node, std::vector<std::string> tags, ProcArgs & args)
{
   bool foundInPath = false;
   AtNode* appliedDisplacement = NULL;
   for(std::map<std::string, AtNode*>::iterator it = args.displacements.begin(); it != args.displacements.end(); ++it) 
   {

     //check both path & tag
     if(it->first.find("/") != std::string::npos)
     {
       if(name.find(it->first) != std::string::npos)
       {
         appliedDisplacement = it->second;
         foundInPath = true;
       }
     }
     else if(matchPattern(name,it->first)) // based on wildcard expression
     {

        AiMsgDebug("[ABC] Displacement pattern '%s' matched %s",it->first.c_str(), name.c_str());
        appliedDisplacement = it->second;
        foundInPath = true;
     }
     else if(foundInPath == false)
     {
       if (std::find(tags.begin(), tags.end(), it->first) != tags.end())
       {
         AiMsgDebug("[ABC] Displacement tag '%s' matched tag on %s",it->first.c_str(), name.c_str());
         appliedDisplacement = it->second;
       }
     }
   }

   if(appliedDisplacement != NULL)
   {
     std::string newName = std::string(AiNodeGetName(appliedDisplacement)) + std::string("_") + name;
     AtArray* shaders = AiArrayAllocate( 1 , 1, AI_TYPE_NODE);
     
     AiMsgDebug("[ABC] Assigning Displacement  %s to %s", AiNodeGetName(appliedDisplacement), AiNodeGetName(node));
     AiArraySetPtr(shaders, 0, appliedDisplacement);
     
     AiNodeSetArray(node, "disp_map", shaders);
   }
}

