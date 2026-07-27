#include <fstream>
#include <string>
#include <algorithm>

std::string getRobotName()
{
    std::ifstream file("robot_name.json");

    std::string line;

    while (std::getline(file, line))
    {
        if (line.find("robot_name") != std::string::npos)
        {
            auto start = line.find(":") + 1;

            std::string name = line.substr(start);

            name.erase(std::remove(name.begin(), name.end(), '"'), name.end());
            name.erase(std::remove(name.begin(), name.end(), ','), name.end());

            return name;
        }
    }

    return "Unknown";
}

void createHeader(std::string robotName)
{
    std::ofstream out("../include/generatedRobot.hpp");

    out << "#pragma once\n\n";
    out << "constexpr char ROBOT_NAME[] = \"";
    out << robotName;
    out << "\";\n";

    out.close();
}