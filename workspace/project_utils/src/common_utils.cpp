#include "project_utils/common_utils.hpp"
void load_map(std::string map_path, std::vector<float>& values_buf, MapArrayXfRow & mat_map_out){

    std::ifstream file(map_path); //map_path is  a string to the .csv file containing
    std::string line;
    //std::vector<float> values_vec;
    while(std::getline(file,line)){
        std::stringstream ss(line);
        std::string word; 
        while(std::getline(ss,word,',')){
            values_buf.push_back(std::stof(word)); // creating a row of 3x1
        }
    }
    int rows = values_buf.size()/NUMCOL;
    new (&mat_map_out) MapArrayXfRow(values_buf.data(), rows, NUMCOL);
    return;
}
