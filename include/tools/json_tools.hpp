#ifndef __JSON_TOOLS_H__
#define __JSON_TOOLS_H__

#include "rapidjson/rapidjson.h"

namespace Common_tools
{
template < typename T >
T *get_json_array( const rapidjson::Document::Array &json_array )
{
    T *res_mat = new T[ json_array.Size() ];
    for ( size_t i = 0; i < json_array.Size(); i++ )
    {
        res_mat[ i ] = ( T ) json_array[ i ].GetDouble();
    }
    return res_mat;
};
} // namespace Common_tools

#endif