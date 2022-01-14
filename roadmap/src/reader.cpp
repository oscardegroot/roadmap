#include "reader.h"

Reader::Reader()
{
    std::string filename = ros::package::getPath("roadmap") + "/maps/test_map.xml";

    ReadXML(filename);
}

void Reader::ReadXML(const std::string &file)
{
    rapidxml::file<> xmlFile(file.c_str()); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    map_.Clear();

    ReadWays(doc);

    ROS_INFO("[Roadmap]: XML Read.");
}

void Reader::ReadWays(const rapidxml::xml_document<> &doc)
{
    // Ways connect the nodes
    map_.ways.clear();
    int id = 0;
    for (rapidxml::xml_node<> *way = doc.first_node("way"); way; way = way->next_sibling("way"))
    {
        // Create a new way and add its nodes (nodes with ref attribute)
        Way new_way;
        for (rapidxml::xml_node<> *node = way->first_node("nd"); node; node = node->next_sibling("nd"))
        {
            // int id = atoi(node->first_attribute("ref")->value());
            new_way.AddNode(Node(atof(node->first_attribute("x")->value()),
                                 atof(node->first_attribute("y")->value()),
                                 atof(node->first_attribute("theta")->value())));
        }

        for (rapidxml::xml_node<> *lane = way->first_node("lane"); lane; lane = lane->next_sibling("lane"))
        {
            // Create a lane for each lane tag
            new_way.AddLane(std::string(lane->first_attribute("type")->value()),
                            atof(lane->first_attribute("width")->value()),
                            bool(atoi(lane->first_attribute("two_way")->value())),
                            id);
        }

        map_.ways.push_back(new_way);

        // Print the result
        // for (auto &lane : new_way.lanes)
        // {
        //     std::cout << lane.id << std::endl;
        //     for (auto &node : lane.nodes)
        //         std::cout << "\tNode " << lane.id << ": " << node.x << ", " << node.y << std::endl;
        // }
    }
}