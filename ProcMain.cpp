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

#include <cstring>
#include <memory>
#include "ProcArgs.h"
#include "PathUtil.h"
#include "SampleUtil.h"
#include "WriteGeo.h"
#include "Overrides.h"
#include "json/json.h"
#include "pystring.h"

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreHDF5/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcCoreFactory/All.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm> 

namespace
{

using namespace Alembic::Abc;
using namespace Alembic::AbcGeom;
using namespace Alembic::AbcCoreFactory;

typedef std::map<std::string, IObject> FileCache;
FileCache g_fileCache;

typedef std::vector<std::string> LoadedAss;
LoadedAss g_loadedAss;

void WalkObject( IObject parent, const ObjectHeader &ohead, ProcArgs &args,
             PathList::const_iterator I, PathList::const_iterator E,
                    MatrixSampleMap * xformSamples)
{
    /* accumulate transformation samples and pass along as an argument */
    /* to WalkObject */
    
    IObject nextParentObject;
    
    std::auto_ptr<MatrixSampleMap> concatenatedXformSamples;
    
    if ( IXform::matches( ohead ) )
    {
        if ( args.excludeXform )
        {
            nextParentObject = IObject( parent, ohead.getName() );
        }
        else
        {
            IXform xform( parent, ohead.getName() );
            
            IXformSchema &xs = xform.getSchema();
            
            if ( xs.getNumOps() > 0 )
            { 
                TimeSamplingPtr ts = xs.getTimeSampling();
                size_t numSamples = xs.getNumSamples();
                
                SampleTimeSet sampleTimes;
                GetRelevantSampleTimes( args, ts, numSamples, sampleTimes,
                        xformSamples);
                
                MatrixSampleMap localXformSamples;
                
                MatrixSampleMap * localXformSamplesToFill = 0;
                
                concatenatedXformSamples.reset(new MatrixSampleMap);
                
                if ( !xformSamples )
                {
                    // If we don't have parent xform samples, we can fill
                    // in the map directly.
                    localXformSamplesToFill = concatenatedXformSamples.get();
                }
                else
                {
                    //otherwise we need to fill in a temporary map
                    localXformSamplesToFill = &localXformSamples;
                }
                
                
                for (SampleTimeSet::iterator I = sampleTimes.begin();
                        I != sampleTimes.end(); ++I)
                {
                    XformSample sample = xform.getSchema().getValue(
                            Abc::ISampleSelector(*I));
                    (*localXformSamplesToFill)[(*I)] = sample.getMatrix();
                }
                
                if ( xformSamples )
                {
                    ConcatenateXformSamples(args,
                            *xformSamples,
                            localXformSamples,
                            *concatenatedXformSamples.get());
                }
                
                
                xformSamples = concatenatedXformSamples.get();
                
            }
            
            nextParentObject = xform;
        }
    }
    else if ( ISubD::matches( ohead ) )
    {
        std::string faceSetName;

        ISubD subd( parent, ohead.getName() );
        
        //if we haven't reached the end of a specified -objectpath,
        //check to see if the next token is a faceset name.
        //If it is, send the name to ProcessSubD for addition of
        //"face_visibility" tags for the non-matching faces
        if ( I != E )
        {
            if ( subd.getSchema().hasFaceSet( *I ) )
            {
                faceSetName = *I;
            }
        }
        
        ProcessSubD( subd, args, xformSamples, faceSetName );
        
        //if we found a matching faceset, don't traverse below
        if ( faceSetName.empty() )
        {
            nextParentObject = subd;
        }
    }
    else if ( IPolyMesh::matches( ohead ) )
    {
        std::string faceSetName;
        
        IPolyMesh polymesh( parent, ohead.getName() );
        
        //if we haven't reached the end of a specified -objectpath,
        //check to see if the next token is a faceset name.
        //If it is, send the name to ProcessSubD for addition of
        //"face_visibility" tags for the non-matching faces
        if ( I != E )
        {
            if ( polymesh.getSchema().hasFaceSet( *I ) )
            {
                faceSetName = *I;
            }
        }
        
        ProcessPolyMesh( polymesh, args, xformSamples, faceSetName );
        
        //if we found a matching faceset, don't traverse below
        if ( faceSetName.empty() )
        {
            nextParentObject = polymesh;
        }
    }
    else if ( INuPatch::matches( ohead ) )
    {
        INuPatch patch( parent, ohead.getName() );
        // TODO ProcessNuPatch( patch, args );
        
        nextParentObject = patch;
    }
    else if ( IPoints::matches( ohead ) )
    {
        IPoints points( parent, ohead.getName() );
        // TODO ProcessPoints( points, args );
        
        nextParentObject = points;
    }
    else if ( ICurves::matches( ohead ) )
    {
        ICurves curves( parent, ohead.getName() );
        // TODO ProcessCurves( curves, args );
        
        nextParentObject = curves;
    }
    else if ( IFaceSet::matches( ohead ) )
    {
        // don't complain about discovering a faceset upon traversal
    }
    else
    {
        std::cerr << "could not determine type of " << ohead.getName()
                  << std::endl;
        
        std::cerr << ohead.getName() << " has MetaData: "
                  << ohead.getMetaData().serialize() << std::endl;
        
        nextParentObject = parent.getChild(ohead.getName());
    }
    
    if ( nextParentObject.valid() )
    {
        //std::cerr << nextParentObject.getFullName() << std::endl;
        
        if ( I == E )
        {
            for ( size_t i = 0; i < nextParentObject.getNumChildren() ; ++i )
            {
                WalkObject( nextParentObject,
                            nextParentObject.getChildHeader( i ),
                            args, I, E, xformSamples);
            }
        }
        else
        {
            const ObjectHeader *nextChildHeader =
                nextParentObject.getChildHeader( *I );
            
            if ( nextChildHeader != NULL )
            {
                WalkObject( nextParentObject, *nextChildHeader, args, I+1, E,
                    xformSamples);
            }
        }
    }
    
    
    
}

//-*************************************************************************

int ProcInit( struct AtNode *node, void **user_ptr )
{
    ProcArgs * args = new ProcArgs( AiNodeGetStr( node, "data" ) );
    args->proceduralNode = node;
    *user_ptr = args;

    if ( args->filename.empty() )
    {
        args->usage();
        return 1;
    }

    #if (AI_VERSION_ARCH_NUM == 3 && AI_VERSION_MAJOR_NUM < 3) || AI_VERSION_ARCH_NUM < 3
        #error Arnold version 3.3+ required for AlembicArnoldProcedural
    #endif
    
    if (!AiCheckAPIVersion(AI_VERSION_ARCH, AI_VERSION_MAJOR, AI_VERSION_MINOR))
    {
        std::cout << "AlembicArnoldProcedural compiled with arnold-"
                  << AI_VERSION
                  << " but is running with incompatible arnold-"
                  << AiGetVersion(NULL, NULL, NULL, NULL) << std::endl;
        return 1;
    } 

    /* Load shaders file*/
    if (AiNodeLookUpUserParameter(node, "assShaders") !=NULL )
    {
        const char* assfile = AiNodeGetStr(node, "assShaders");
        if(*assfile != 0)
        {
            // if we don't find the ass file, we can load it. This avoid multiple load of the same file.
            if(std::find(g_loadedAss.begin(), g_loadedAss.end(), std::string(assfile)) == g_loadedAss.end())
            {
                if(AiASSLoad(assfile, AI_NODE_SHADER) == 0)
                    g_loadedAss.push_back(std::string(assfile));

            }
            
        }
    }

    bool skipJson = false;
    bool skipShaders = false;
    bool skipOverrides = false;
    bool skipDisplacement = false;
    if (AiNodeLookUpUserParameter(node, "skipJson") !=NULL )
        skipJson = AiNodeGetBool(node, "skipJson");
    if (AiNodeLookUpUserParameter(node, "skipShaders") !=NULL )
        skipShaders = AiNodeGetBool(node, "skipShaders");
    if (AiNodeLookUpUserParameter(node, "skipOverrides") !=NULL )
        skipOverrides = AiNodeGetBool(node, "skipOverrides");
    if (AiNodeLookUpUserParameter(node, "skipDisplacements") !=NULL )
        skipDisplacement = AiNodeGetBool(node, "skipDisplacements");
    

    Json::Value jrootShaders;
    Json::Value jrootOverrides;
    Json::Value jrootDisplacements;
    bool parsingSuccessful = false;

    // Load attribute overides if there is a attribute present pointing to an overrides file
    if (AiNodeLookUpUserParameter(node, "overridefile") !=NULL && skipJson == false)
    {
        Json::Value jroot;
        Json::Reader reader;
        std::ifstream test(AiNodeGetStr(node, "overridefile"), std::ifstream::binary);
        parsingSuccessful = reader.parse( test, jroot, false );
        if ( parsingSuccessful )
        {
            /* OVERRIDES */
            if(skipOverrides == false)
            {
                jrootOverrides = jroot["overrides"];
                if (AiNodeLookUpUserParameter(node, "overrides") !=NULL)
                {
                    Json::Reader readerOverride;
                    Json::Value jrootOverridesOverrides;

                    if(readerOverride.parse( AiNodeGetStr(node, "overrides"), jrootOverridesOverrides))
                    {
                        for( Json::ValueIterator itr = jrootOverridesOverrides.begin() ; itr != jrootOverridesOverrides.end() ; itr++ ) 
                        {
                            const Json::Value paths = jrootOverridesOverrides[itr.key().asString()];
                            for( Json::ValueIterator overPath = paths.begin() ; overPath != paths.end() ; overPath++ ) 
                            {
                                Json::Value attr = paths[overPath.key().asString()];
                                jrootOverrides[itr.key().asString()][overPath.key().asString()] = attr;

                            }

                        }
                    }

                }
            }
        }
    }

    // Load shader assignments if there is a attribute present pointing to an shader assignments file
    if (AiNodeLookUpUserParameter(node, "shaderAssignmentfile") !=NULL && skipJson == false)
    {
        Json::Value jroot;
        Json::Reader reader;
        std::ifstream test(AiNodeGetStr(node, "shaderAssignmentfile"), std::ifstream::binary);
        parsingSuccessful = reader.parse( test, jroot, false );
        if ( parsingSuccessful )
        {

            /* SHADERS */
            if(skipShaders == false)
            {
                jrootShaders = jroot["shaders"];
                if (AiNodeLookUpUserParameter(node, "shaderAssignation") !=NULL)
                {
                    Json::Reader readerOverride;
                    Json::Value jrootShadersOverrides;
                    if(readerOverride.parse( AiNodeGetStr(node, "shaderAssignation"), jrootShadersOverrides ))
                    {
                        if(jrootShadersOverrides.size() > 0)
                        {
                            Json::Value newJrootShaders;
                            // concatenate both json string.
                            for( Json::ValueIterator itr = jrootShadersOverrides.begin() ; itr != jrootShadersOverrides.end() ; itr++ ) 
                            {
                                Json::Value tmp = jrootShaders[itr.key().asString()];
                                const Json::Value paths = jrootShadersOverrides[itr.key().asString()];
                                for( Json::ValueIterator shaderPath = paths.begin() ; shaderPath != paths.end() ; shaderPath++ ) 
                                {
                                    Json::Value val = paths[shaderPath.key().asUInt()];
                                    // now, create the new paths
                                    for( Json::ValueIterator itr2 = jrootShaders.begin() ; itr2 != jrootShaders.end() ; itr2++ ) 
                                    {
                                        const Json::Value pathsShader = jrootShaders[itr2.key().asString()];
                                        for( Json::ValueIterator shaderPathOrig = pathsShader.begin() ; shaderPathOrig != pathsShader.end() ; shaderPathOrig++ ) 
                                        {
                                            Json::Value val2 = pathsShader[shaderPathOrig.key().asUInt()];                                  
                                            if(val2.asString() != val.asString())
                                            {
                                                newJrootShaders[itr2.key().asString()].append(val2.asString());
                                            }

                                        }
                                    }
                                }
                                if(tmp.size() == 0)
                                {
                                    newJrootShaders[itr.key().asString()] = jrootShadersOverrides[itr.key().asString()];
                                }
                                else
                                {
                                    const Json::Value shaderPaths = jrootShadersOverrides[itr.key().asString()];
                                    for( Json::ValueIterator itr2 = shaderPaths.begin() ; itr2 != shaderPaths.end() ; itr2++ ) 
                                    {
                                        newJrootShaders[itr.key().asString()].append(jrootShadersOverrides[itr.key().asString()][itr2.key().asUInt()]);
                                    }
                                }

                            }
                            jrootShaders = newJrootShaders;
                        }
                    }
                    
                }
            }

            /* DISPLACEMENTS */
            if(skipDisplacement == false)
            {
                jrootDisplacements = jroot["displacement"];
                if (AiNodeLookUpUserParameter(node, "displacementAssignation") !=NULL)
                {
                    Json::Reader readerOverride;
                    Json::Value jrootDisplacementsOverrides;
                    if(readerOverride.parse( AiNodeGetStr(node, "displacementAssignation"), jrootDisplacementsOverrides ))
                    {
                        if(jrootDisplacementsOverrides.size() > 0)
                        {
                            Json::Value newJrootDisplacements;
                            // concatenate both json string.
                            for( Json::ValueIterator itr = jrootDisplacementsOverrides.begin() ; itr != jrootDisplacementsOverrides.end() ; itr++ ) 
                            {
                                Json::Value tmp = jrootDisplacements[itr.key().asString()];
                                const Json::Value paths = jrootDisplacementsOverrides[itr.key().asString()];
                                for( Json::ValueIterator shaderPath = paths.begin() ; shaderPath != paths.end() ; shaderPath++ ) 
                                {
                                    Json::Value val = paths[shaderPath.key().asUInt()];
                                    // now, create the new paths
                                    for( Json::ValueIterator itr2 = jrootDisplacements.begin() ; itr2 != jrootDisplacements.end() ; itr2++ ) 
                                    {
                                        const Json::Value pathsShader = jrootDisplacements[itr2.key().asString()];
                                        for( Json::ValueIterator shaderPathOrig = pathsShader.begin() ; shaderPathOrig != pathsShader.end() ; shaderPathOrig++ ) 
                                        {
                                            Json::Value val2 = pathsShader[shaderPathOrig.key().asUInt()];                                  
                                            if(val2.asString() != val.asString())
                                            {
                                                newJrootDisplacements[itr2.key().asString()].append(val2.asString());
                                            }
                                        }
                                    }
                                }
                                if(tmp.size() == 0)
                                {
                                    newJrootDisplacements[itr.key().asString()] = jrootDisplacementsOverrides[itr.key().asString()];
                                }
                                else
                                {
                                    const Json::Value shaderPaths = jrootDisplacementsOverrides[itr.key().asString()];
                                    for( Json::ValueIterator itr = shaderPaths.begin() ; itr != shaderPaths.end() ; itr++ ) 
                                        newJrootDisplacements[itr.key().asString()].append(jrootDisplacementsOverrides[itr.key().asString()][itr.key().asUInt()]);

                                }

                            }
                            jrootDisplacements = newJrootDisplacements;
                        }
                    }
                    
                }
            }
        }
    }
    
    // Catch if the json data is in the optional attributes shaderAssignation,overrides,displacementAssignation
    // instead of parsing the two json files
    if(!parsingSuccessful)
    {
        if (AiNodeLookUpUserParameter(node, "overrides") !=NULL  && skipOverrides == false)
        {
            Json::Reader reader;
            bool parsingSuccessful = reader.parse( AiNodeGetStr(node, "overrides"), jrootOverrides );
        }
        if (AiNodeLookUpUserParameter(node, "shaderAssignation") !=NULL && skipShaders == false)
        {
            Json::Reader reader;
            bool parsingSuccessful = reader.parse( AiNodeGetStr(node, "shaderAssignation"), jrootShaders );
        }
        if (AiNodeLookUpUserParameter(node, "displacementAssignation") !=NULL  && skipDisplacement == false)
        {
            Json::Reader reader;
            bool parsingSuccessful = reader.parse( AiNodeGetStr(node, "displacementAssignation"), jrootDisplacements );
        }
    }


    //Check displacements

    if( jrootDisplacements.size() > 0 )
    {
        args->linkDisplacement = true;
        for( Json::ValueIterator itr = jrootDisplacements.begin() ; itr != jrootDisplacements.end() ; itr++ ) 
        {
            AiMsgDebug( "[ABC] Parsing displacement shader %s", itr.key().asCString()); 
            AtNode* shaderNode = AiNodeLookUpByName(itr.key().asCString());
            if(shaderNode == NULL)
            {
                AiMsgDebug( "[ABC] Searching displacement shader %s deeper underground...", itr.key().asCString()); 
                // look for the same namespace for shaders...
                std::vector<std::string> strs;
                // boost::split(strs,args->nameprefix,boost::is_any_of(":"));
                // do split based on ':'

                // if(strs.size() > 1)
                // {
                //     strs.pop_back();
                //     strs.push_back(itr.key().asString());
                        
                //     shaderNode = AiNodeLookUpByName(boost::algorithm::join(strs, ":").c_str());
                // }
            }

            if(shaderNode != NULL)
            {
                const Json::Value paths = jrootDisplacements[itr.key().asString()];
                AiMsgDebug("[ABC] displacement Shader exists, checking paths. size = %d", paths.size());
                for( Json::ValueIterator itr2 = paths.begin() ; itr2 != paths.end() ; itr2++ ) 
                {
                    Json::Value val = paths[itr2.key().asUInt()];
                    AiMsgDebug("[ABC] Adding path %s", val.asCString());
                    args->displacements[val.asString().c_str()] = shaderNode;
                }
            }
            else
            {
                AiMsgWarning("[ABC] Can't find displacement shader %s", itr.key().asCString());
            }
        }
    }


    // Check if we can link shaders or not.
    if( jrootShaders.size() > 0 )
    {
        args->linkShader = true;
        for( Json::ValueIterator itr = jrootShaders.begin() ; itr != jrootShaders.end() ; itr++ ) 
        {
            AiMsgDebug( "[ABC] Parsing shader %s", itr.key().asCString()); 
            AtNode* shaderNode = AiNodeLookUpByName(itr.key().asCString());
            if(shaderNode == NULL)
            {
                AiMsgDebug( "[ABC] Searching shader %s deeper underground...", itr.key().asCString()); 
                // look for the same namespace for shaders...
                std::vector<std::string> strs;
                // boost::split(strs,args->nameprefix,boost::is_any_of(":"));
                // if(strs.size() > 1)
                // {
                //     strs.pop_back();
                //     strs.push_back(itr.key().asString());
                        
                //     shaderNode = AiNodeLookUpByName(boost::algorithm::join(strs, ":").c_str());
                // }
            }
            if(shaderNode != NULL)
            {
                const Json::Value paths = jrootShaders[itr.key().asString()];
                AiMsgDebug("[ABC] Shader exists, checking paths. size = %d", paths.size());
                for( Json::ValueIterator itr2 = paths.begin() ; itr2 != paths.end() ; itr2++ ) 
                {
                    Json::Value val = paths[itr2.key().asUInt()];
                    AiMsgDebug("[ABC] Adding path %s", val.asCString());
                    args->shaders[val.asString().c_str()] = shaderNode;
                }
            }
            else
            {
                AiMsgWarning("[ABC] Can't find shader %s", itr.key().asCString());
            }
        }
    }

            
    if( jrootOverrides.size() > 0 )
    {
        args->linkOverride = true;
        args->overrideRoot = jrootOverrides;
        for( Json::ValueIterator itr = jrootOverrides.begin() ; itr != jrootOverrides.end() ; itr++ ) 
        {
            std::string path = itr.key().asString();
            args->overrides.push_back(path);

        }
        std::sort(args->overrides.begin(), args->overrides.end());
    }
    IObject root;
    
    // Load the alembic file

    FileCache::iterator I = g_fileCache.find(args->filename);
    if (I != g_fileCache.end())
        root = (*I).second;

    else
    {
        IFactory factory; 
        IArchive archive = factory.getArchive(args->filename); 
        if (!archive.valid())
        {
            AiMsgError ( "Cannot read file %s", args->filename.c_str());
        }
        else 
        {
            AiMsgDebug ( "reading file %s", args->filename.c_str());
            g_fileCache[args->filename] = archive.getTop();
            root = archive.getTop();
        }
        
    }
    
    PathList path;
    TokenizePath( args->objectpath, path );

    try
    {
        if ( path.empty() ) //walk the entire scene
        {
            for ( size_t i = 0; i < root.getNumChildren(); ++i )
            {
                WalkObject( root, root.getChildHeader(i), *args,
                            path.end(), path.end(), 0 );
            }
        }
        else //walk to a location + its children
        {
            PathList::const_iterator I = path.begin();

            const ObjectHeader *nextChildHeader =
                    root.getChildHeader( *I );
            if ( nextChildHeader != NULL )
            {
                WalkObject( root, *nextChildHeader, *args, I+1,
                        path.end(), 0);
            }
        }
    }
    catch ( const std::exception &e )
    {
        AiMsgError("exception thrown during ProcInit: %s", e.what());
    }
    catch (...)
    {
        AiMsgError("exception thrown");
    }
    return 1;
}

//-*************************************************************************

int ProcCleanup( void *user_ptr )
{
    delete reinterpret_cast<ProcArgs*>( user_ptr );
    return 1;
}

//-*************************************************************************

int ProcNumNodes( void *user_ptr )
{
    ProcArgs * args = reinterpret_cast<ProcArgs*>( user_ptr );
    const char* nodeName = AiNodeGetName(args->proceduralNode);
    // AiMsgInfo("[bb_AlembicArnoldProcedural] number of nodes in %s: %d", nodeName,args->createdNodes.size());

    return (int) args->createdNodes.size();
}

//-*************************************************************************

struct AtNode* ProcGetNode(void *user_ptr, int i)
{
    ProcArgs * args = reinterpret_cast<ProcArgs*>( user_ptr );
    
    if ( i >= 0 && i < (int) args->createdNodes.size() )
    {
        const char* nodeName = AiNodeGetName(args->createdNodes[i]);
        // AiMsgInfo("[bb_AlembicArnoldProcedural] rendering internal node : %s", nodeName);

        return args->createdNodes[i];
    }
    
    return NULL;
}

} //end of namespace



extern "C"
{
    int ProcLoader(AtProcVtable* api)
    {
        api->Init        = ProcInit;
        api->Cleanup     = ProcCleanup;
        api->NumNodes    = ProcNumNodes;
        api->GetNode     = ProcGetNode;
        strcpy(api->version, AI_VERSION);
        return 1;
    }
}
