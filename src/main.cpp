#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"
#include <cstdlib>

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
   // To check for the validity of the input
    //this code logic has been taken from https://stackoverflow.com/questions/4271315/c-how-to-check-an-input-float-variable-for-valid-input
    float start_x , start_y , end_x , end_y;
    int i= 0;
    std::string input1 , input2 , input3 , input4;
    while(i < 4)
    {
        if(i ==0)
        {
            std::cout << "Enter the start coordinate x " << std::endl;
            std::cin >> input1;
            if(input1.find_first_not_of("1234567890.-") != std::string::npos)
            {
                std::cout << " Please input valid float value for start coordinate x " << std::endl;
                continue;
            }
            else
            {
                start_x = std::stof(input1.c_str());

            }
        }
        if(i == 1)
        {
            std::cout << "Enter the start coordinate y " << std::endl;
            std::cin >> input2;
            if(input2.find_first_not_of("1234567890.-") != std::string::npos)
            {
                std::cout << " Please input valid float value for start coordinate y " << std::endl;
                continue;
            }
            else
            {
                 start_y = std::stof(input2.c_str());
            }
        }
        if(i ==2)
        {
            std::cout << "Enter the end coordinate x " << std::endl;
            std::cin >> input3;
            if(input3.find_first_not_of("1234567890.-") != std::string::npos)
            {
                std::cout << " Please input valid float value for end x coordinate " << std::endl;
                continue;
            }
            else
            {
                end_x = std::stof(input3.c_str());
            }
        }
        if(i ==3)
        {
            std::cout << "Enter the end coordinate y" << std::endl;
            std::cin >> input4;
            if(input4.find_first_not_of("1234567890.-") != std::string::npos)
            {
                std::cout << " Please input valid float value for end y coordinate " << std::endl;
                continue;
            }
            else
            {
                end_y = std::stof(input4.c_str());
            }
        }

        ++i;

    }
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
