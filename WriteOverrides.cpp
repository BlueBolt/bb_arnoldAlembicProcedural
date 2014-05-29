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
#include <ai.h>
#include <sstream>

#include "json/json.h"



void ApplyOverrides(std::string name, AtNode* node, std::vector<std::string> tags, ProcArgs & args)
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

       if ( paramEntry != NULL && attribute!="invert_normals")
       {

         Json::Value val = args.overrideRoot[*it][itr.key().asString()];
         if( val.isString() ) 
           AiNodeSetStr(node, attribute.c_str(), val.asCString());
         else if( val.isBool() ) 
           AiNodeSetBool(node, attribute.c_str(), val.asBool());
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
      if(it->find("/") != string::npos) // Based on path
      {
        if(name.find(*it) != string::npos)
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

            const AtNodeEntry* nodeEntry = AiNodeGetNodeEntry(node);

            Json::Value val = args.userAttributesRoot[*it][attribute];
            if( val.isString() ) 
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_STRING);
              AiNodeSetStr(node, attribute.c_str(), val.asCString());
            else if( val.isBool() ) 
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_BOOLEAN);
              AiNodeSetBool(node, attribute.c_str(), val.asBool());
            else if( val.isInt() ) 
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_INT);
              AiNodeSetInt(node, attribute.c_str(), val.asInt());
            else if( val.isUInt() ) 
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_UINT);
              AiNodeSetUInt(node, attribute.c_str(), val.asUInt());
            else if(val.isDouble())
              AddUserGeomParams(node,attribute.c_str(),AI_TYPE_FLOAT);
              AiNodeSetInt(node, attribute.c_str(), val.asDouble());
            // TODO color, matrix, vector

         }
      }
   }
}

void ApplyShaders(std::string name, AtNode* node, std::vector<std::string> tags, ProcArgs & args, bool matte)
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
     else if(matchPattern(name,it->first)) // based on wildcard expression
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
     AiMsgDebug("[ABC] Assigning shader  %s to %s", AiNodeGetName(appliedShader), AiNodeGetName(node));
     AtArray* shaders = AiArrayAllocate( 1 , 1, AI_TYPE_NODE);
     
     if(matte)
     {
       std::string shaderName =  newName + std::string("_BlackHole");
       AtNode* matteShader = AiNode ("Blackhole");
       AiNodeSetStr(matteShader, "name", shaderName.c_str());
       AiNodeLink (appliedShader, "input", matteShader);
       AiArraySetPtr(shaders, 0, matteShader);
     }
     else
     {
       AiArraySetPtr(shaders, 0, appliedShader);
     }  

     AiNodeSetArray(node, "shader", shaders);
   }
   else
   {
     AtArray* shaders = AiNodeGetArray(args.proceduralNode, "shader");
     if (shaders->nelements != 0)
        AiNodeSetArray(node, "shader", AiArrayCopy(shaders));
   }
}