#ifndef __READER_H__
#define __READER_H__

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <stdlib.h> /* atoi */
#include <map>

#include "rapidxml_utils.hpp"

#include "types.h"

class Reader
{

public:
    Reader();

public:
    /**
     * @brief Get a reference to the loaded map
     * 
     * @return const Map
     */
    Map &GetMap() { return map_; };

private:
    Map map_;

    /**
     * @brief Read an XML file with map data (nodes and ways)
     * 
     * @param file the file path to read from
     */
    void ReadXML(const std::string &file);

    // XML Read functions
    void ReadWays(const rapidxml::xml_document<> &doc);
};

#endif // __READER_H__