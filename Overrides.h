
#ifndef _Alembic_Arnold_OverrideList_h_
#define _Alembic_Arnold_OverrideList_h_

#include <ai.h>
#include <string>
#include <vector>

struct Overrides
{
    /* new constructor */
    Overrides( const char node, const std::vector<char> * overrides );

    /* copy constructor */
    Overrides( const Overrides &orl )
    : nodeName(orl.nodeName)
    ,attributeList(orl.attributeList)
    {}

    void applyOverride(AtNode * node);

    // this node
    std::string nodeName;

    // map of attributes and values, should we capture all attribute type here 
    // or make the procedural convert from char?

    std::map<string, char> attributeList;

};


#endif