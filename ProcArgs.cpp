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
#include "ProcArgs.h"

#include <boost/tokenizer.hpp>

#include <vector>
#include <algorithm>
#include <iostream>

//-*****************************************************************************
//INSERT YOUR OWN TOKENIZATION CODE AND STYLE HERE
ProcArgs::ProcArgs( const char * paramStr )
  : frame(0.0)
  , frameOffset(0.0)
  , fps(24.0)
  , shutterOpen(0)
  , shutterClose(0)
  , excludeXform(false)
  , subdIterations(0)
  , subdUVSmoothing("pin_corners")
  , pattern("*")
  , disp_padding(-AI_BIG)
  , proceduralNode(0)
  , flipv(false)
  , invertNormals(false)  
  , linkShader(false)
  , linkDisplacement(false)
  , linkOverride(false)
  , linkUserAttributes(false)
{    
    // Grab the shutter a camera attached to AiUniverse if present

    AtNode* camera = AiUniverseGetCamera();
    shutterOpen = AiNodeGetFlt(camera, "shutter_start");
    shutterClose = AiNodeGetFlt(camera, "shutter_end");

    typedef boost::char_separator<char> Separator;
    typedef boost::tokenizer<Separator> Tokenizer;
    
    std::vector<std::string> tokens;
    std::string params( paramStr );

    Tokenizer tokenizer( params, Separator(" ") );
    for ( Tokenizer::iterator iter = tokenizer.begin(); iter != tokenizer.end() ;
          ++iter )
    {
        if ( (*iter).empty() ) { continue; }

        tokens.push_back( *iter );
    }

    for ( size_t i = 0; i < tokens.size(); ++i )
    {
        std::string token = tokens[i];
        std::transform( token.begin(), token.end(), token.begin(), ::tolower );

        if ( token == "-frame" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                frame = atof( tokens[i].c_str() );
            }
        }
        else if ( token == "-frameoffset" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                frameOffset = atof( tokens[i].c_str() );
            }
        }
        else if ( token == "-fps" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                fps = atof( tokens[i].c_str() );
            }
        }
        else if ( token == "-shutteropen" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                shutterOpen = atof( tokens[i].c_str() );
            }
        }
        else if ( token == "-shutterclose" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                shutterClose = atof( tokens[i].c_str() );
            }
        }
        else if ( token == "-filename" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                filename = tokens[i];
            }
        }
        else if ( token == "-nameprefix" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                nameprefix = tokens[i];
            }
        }
        else if ( token == "-objectpath" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                objectpath = tokens[i];
            }
        }
        else if ( token == "-pattern" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                pattern = tokens[i];
            }
        }
        else if ( token == "-excludexform" )
        {
            excludeXform = true;
            
        }
        else if ( token == "-subditerations" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                subdIterations = atoi( tokens[i].c_str() );
            }
        }
        else if ( token == "-subduvsmoothing" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                subdUVSmoothing = tokens[i];
            }
        }
        else if ( token == "-disp_map" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                disp_map = tokens[i];
            }
        }
        else if ( token == "-disp_padding" )
        {
            ++i;
            if ( i < tokens.size() )
            {
                disp_padding = atof( tokens[i].c_str() );
            }
        }
        else if ( token == "-makeinstance" )
        {
            makeInstance = true;
        }
        else if ( token == "-flipv" )
        {
            flipv = true;
        }
        else if ( token == "-invertNormals" )
        {
            invertNormals = true;
        }
        
    }
}


void ProcArgs::usage()
{
    std::cerr << "bb_AlembicArnoldProcedural 0.9.1 usage:" << std::endl;
    std::cerr << std::endl;
    
    
    std::cerr << "-filename /path/to/some/archive.abc" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "This is the only required argument. "
                 "It has no default value." << std::endl;
    std::cerr << std::endl;
    

    std::cerr << "-frame 42" << std::endl;
    std::cerr << std::endl;

    std::cerr << "The frame number to load from within the archive. "
                 "The default value is 0. This is combined with -fps to map "
                 "to Alembic time units (double-precision seconds).";
    
    std::cerr << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "-fps 24" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "Combined with -frame above. The default value is 24.0.";
    std::cerr << std::endl;
    std::cerr << std::endl;
    
    
    std::cerr << "-shutteropen 0.0" << std::endl;
    std::cerr << "-shutterclose 0.5" << std::endl;
    std::cerr << std::endl;


    std::cerr << "These are frame-relative values which specify the shutter "
                 "window. The procedural will include all samples present in "
                 "the archive which are relevant to the shutter window. "
                 "The default value of both is 0.0 (no motion blur).";
    std::cerr << std::endl;
    std::cerr << std::endl;


    std::cerr << "-objectpath /assetroot/characters" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "If specified, only objects at or below the provided path "
                 "(within the archive) will be emitted. When combined with "
                 "-excludexform, this can also be used to load individual "
                 "leaf locations within an externally defined hierarchy. Be "
                 "aware that in that case, you'd need to set the \"matrix\" "
                 "and \"inherit_xform\" parameters on the procedural node "
                 "itself. If the path points to a single \"faceset\" object "
                 "directly beneath a polymesh or subdivision mesh, it'll add "
                 "a \"face_visibility\" user data array.";
    std::cerr << std::endl;
    std::cerr << std::endl;
    
    std::cerr << std::endl;    std::cerr << "-pattern \"*\" " << std::endl;

    std::cerr << "If given will only load geometry that mtches the given pattern.";
    std::cerr << std::endl;
    std::cerr << std::endl;

    std::cerr << "-excludexform" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "If specified, the \"matrix\" parameter will not be set on "
                 "the resulting primitive nodes." << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "-subditerations 2" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "For AbcGeom::ISubD objects, this option specifies the "
                 "\"subdiv_iterations\" value. It currently has no effect for "
                 "other primitive types. The default value is 0.";
    std::cerr << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "-nameprefix some_prefix__" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "Because node names are unique scene-wide in arnold, this "
                 "allows you control potential name clashes when loading or "
                 "instancing an archive (or multiple equivalently named "
                 "archives) multiple times. The default name of each node is "
                 "its full path within the alembic archive.";
    std::cerr << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "-flipv" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "Flip the uvs of any polymesh/subd in v direction";
    
    std::cerr << std::endl;

    std::cerr << "-makeinstance" << std::endl;
    std::cerr << std::endl;
    
    std::cerr << "This behavior is disabled by default. If enabled, the "
                 "procedural will attempt to identify identical primitives "
                 "(using Alembic's per-array-property hash keys) and create "
                 "corresponding \"ginstance\" nodes. Two primitives are "
                 "considered equivalent if the keys of their relevant point "
                 "position samples match along with any specified "
                 "subdivision values. This works across multiple archives or "
                 "invokations of the procedural. It currently does not write "
                 "unique user data per instance but will likely do so "
                 "automatically (when necessary) in a future release. "
                 "The ray visibility of the source primitive will be set to "
                 "AI_RAY_NONE and the \"ginstance\" node's will be set to "
                 "that of the calling \"procedural\" node.";
    
    std::cerr << std::endl;

 
    std::cerr <<  "======="
                  ""
                  "User Properties"
                  ""
                  "======="
                  ""
                  "There are key user attributes that can be added to the procedural"
                  "that affect the contense of the generated geometry:"
                  ""
                  "The procedural will detect various custom user attributes as" 
                  "json dictionaries."
                  "Where ever a target node is given it can be given as a alembic"
                  "object path (\"/root/path/to/object\"), a wildcard regualr expression"
                  "(\"*match*\"),or a key of an exported geometry constant \"tag\""
                  "attribute with key:value pairs.";

    std::cerr << std::endl;


    std::cerr <<   "* overrides <string>"
                   "* overridesfile <string>"
                   ""
                   "This is either a JSON dictionary containgin key, dictionary pairs :"
                   ""
                   "{\"target\":{\"attr\":value}}" 
                   ""
                   "or a path to a file containig key:dictionary pairs with in a larger"
                   "dictionary key named \"overrides\"." 
                   ""
                   "{\"overrides\":{\"target\":{\"attr\":value}}}"
                   ""
                   "These attriubtes need to match the available attreributes for the"
                   "type of geometry that you are overiddeing. To find the aproppriate"
                   "attributes use kicks info tools e.g. kick -info polymesh will give"
                   "the attributes available on a polymesh. In addition to these attributes"
                   "you can also use a \"matte\" boolean attribute; set it to true and"
                   "the procedural will add a usr attribute called \"enable_matte\" to"
                   "the target object(s)" ;

    std::cerr << std::endl;

    std::cerr <<   "* userAttributes <string>"
                   "* userAttributesfile <string>"
                   ""
                   "These attributes are simaler to the overrides, however they only add user"
                   "attributes, usefull for linking to procedural shader operations and using for"
                   "userdata operations."
                   ""
                   "{\"userAttributes\":{\"target\":{\"attr\":value}}}";

    std::cerr << std::endl;

    std::cerr <<   "* shaderAssignation <string>"
                   "* displacementAssignation <string>"
                   "* shaderAssignationfile <string>"
                   ""
                   "Shader and displacement assignment is done from a shader:list pattern, so you"
                   "can choose a shader and assign it to multiple objects within the heirarchy:"
                   ""
                   "{\"shaderName\":[\"/path/to/object\",\"*match*\",\"tag\"]}"
                   ""
                   "They can also be assigned using a JSON file under the two keys \"shaders\""
                   "and \"displacement\"";

    std::cerr << std::endl;

    std::cerr <<   "* assShaders <string>"
                   ""
                   "This is the optional path to a ass file containig shaders that you wish to load"
                   "in to arnold, these shaders may be ones you a referenceing to in the shaders"
                   "attribute above.";

    std::cerr << std::endl;


}
