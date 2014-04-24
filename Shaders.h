
#ifndef _Alembic_Arnold_Shaders_h_
#define _Alembic_Arnold_Shaders_h_

#include <ai.h>
#include <string>
#include <vector>

struct ShaderAssignmet
{
    /* new constructor */
    ShaderOverride( const char shader, const std::vector<char> * nodes );

    /* copy constructor */
    ShaderOverride( const ShaderOverride &orl )
    : nodeName(orl.nodeName)
    ,attributeList(orl.attributeList)
    {}

    // the shader node to attach
    AtNode * shaderNode;

    // array of objects to attach to
    std::vector<char> nodeslist;

};


AtNode *createShader(const char name);

#endif