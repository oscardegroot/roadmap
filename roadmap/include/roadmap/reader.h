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
#include "configuration.h"

class Reader
{

    /**
 * @brief Class for reading map files and converting it into necessary formats
 * @see types.h
 */
public:
    Reader(RoadmapConfig *config)
        : config_(config)
    {
    }

public:
    /**
     * @brief Read a map file
     * 
     * @param file_name file name relative to roadmap package (either xml or yaml files)
     */
    void Read(const std::string &file_name); // With custom name

    /**
     * @brief Read the file as defined in the config
     */
    void Read();

    /**
     * @brief Read wrapper for r values
     */
    void Read(const std::string &&file_name);

    /**
     * @brief Get a reference to the loaded map
     * @see types.h
     * @return const Map
     */
    Map &GetMap() { return map_; };

private:
    Map map_;               /** The read map */
    RoadmapConfig *config_; /** Parameters */

    /**
     * @brief Read an XML file with map data
     * 
     * @param file the file path to read from
     */
    void ReadXML(const std::string &file);

    /**
     * @brief Read a yaml file with map data 
     * 
     * @param file the file path to read from
     */
    void ReadYAML(const std::string &file);

    // XML Read function
    void ReadWays(const rapidxml::xml_document<> &doc);
};

#endif // __READER_H__