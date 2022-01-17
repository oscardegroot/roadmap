#include "reader.h"

void Reader::Read()
{
    std::string map_file = ros::package::getPath("roadmap") + "/" + config_->map_file_name_;
    Read(map_file);
}

void Reader::Read(const std::string &file_name)
{
    ROADMAP_INFO("Reading map file");

    // If the path does not contain the package, add it
    std::string map_file;
    if (file_name.find("//roadmap//") != std::string::npos)
        map_file = ros::package::getPath("roadmap") + "/" + file_name;
    else
        map_file = file_name;

    // Read the file with the correct file extension
    if (map_file.substr(map_file.find_last_of(".") + 1) == "xml")
        ReadXML(map_file);
    else
        ReadYAML(map_file);
}

void Reader::Read(const std::string &&file_name)
{
    std::string map_file = ros::package::getPath("roadmap") + "/" + file_name;
    Read(map_file);
}

void Reader::ReadXML(const std::string &file)
{
    rapidxml::file<> xmlFile(file.c_str()); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    map_.Clear();

    ReadWays(doc);

    ROADMAP_INFO("XML Read.");
}

void Reader::ReadYAML(const std::string &file)
{
    // std::cout << std::string("rosparam load ") + file << std::endl;
    int result = system(std::string("rosparam load " + file).c_str());
    assert(result == 0);

    // Assume one Way object as reference path
    map_.ways.clear();

    ros::NodeHandle nh;

    std::vector<double> x, y, theta;
    RoadmapConfig::retrieveParameter(nh, "global_path/x", x);
    RoadmapConfig::retrieveParameter(nh, "global_path/y", y);
    RoadmapConfig::retrieveParameter(nh, "global_path/theta", theta);
    assert(x.size() == y.size());
    assert(y.size() == theta.size());

    // Create a way object
    Way new_way;
    for (size_t i = 0; i < x.size(); i++)
        new_way.AddNode(Node(x[i], y[i], theta[i])); // Create nodes

    int id = 0;
    // Add a regular road and sidewalk by default
    new_way.AddLane("road", 4.0, true, id);
    new_way.AddLane("sidewalk", 2.0, true, id);
    map_.ways.push_back(new_way);
    ROADMAP_INFO("YAML Read.");
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
            // Optional two way parameters (default = false)
            bool two_way = false;
            if (lane->first_attribute("two_way")->value())
                two_way = bool(atoi(lane->first_attribute("two_way")->value()));

            // Create a lane for each lane tag
            new_way.AddLane(std::string(lane->first_attribute("type")->value()),
                            atof(lane->first_attribute("width")->value()),
                            two_way,
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