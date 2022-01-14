#include "reader.h"

Reader::Reader()
{
    std::string filename = ros::package::getPath("roadmap") + "/maps/test_map.xml";

    // std::cout << "Reading " << filename << std::endl;
    ReadXML(filename);
}

void Reader::ReadXML(const std::string &file)
{
    rapidxml::file<> xmlFile(file.c_str()); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    map_.Clear();

    ReadNodes(doc);
    ReadWays(doc);
}

void Reader::ReadNodes(const rapidxml::xml_document<> &doc)
{
    // Iterate through all "nodes"
    for (rapidxml::xml_node<> *node = doc.first_node("node"); node; node = node->next_sibling("node"))
    {
        int id = atoi(node->first_attribute("id")->value());

        map_.nodes[id] = Node(id,
                              atof(node->first_attribute("x")->value()),
                              atof(node->first_attribute("y")->value()),
                              atof(node->first_attribute("theta")->value()));
    }
}

void Reader::ReadWays(const rapidxml::xml_document<> &doc)
{
    // Ways connect the nodes
    map_.ways.clear();
    for (rapidxml::xml_node<> *way = doc.first_node("way"); way; way = way->next_sibling("way"))
    {
        int way_id = atoi(way->first_attribute("id")->value());

        // Create a new way and add its nodes (nodes with ref attribute)
        Way new_way(way_id);
        for (rapidxml::xml_node<> *node = way->first_node("nd"); node; node = node->next_sibling("nd"))
        {
            int id = atoi(node->first_attribute("ref")->value());
            new_way.AddNode(id);
        }

        // Retrieve and set the road type (tag with type attribute)
        new_way.AddType(way->first_node("tag")->first_attribute("type")->value());

        // Print the result
        // std::cout << "Way (type = " << new_way.type << "):\n";
        // for (auto &i : new_way.nodes)
        //     std::cout << "\tNode " << map_.nodes[i].id << ": " << map_.nodes[i].x << ", " << map_.nodes[i].y << std::endl;

        map_.ways.push_back(new_way);
    }
}