#include "project_utils/common_utils.hpp"
// typedef Eigen::Map<
//           Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
//         > MapArrayXfRow;
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
    // Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat_map(
    //     values_vec.data(),    // pointer to first element of v
    //     rows,        // number of rows
    //     NUMCOL            // number of columns
    // );
    // MatrixXfRow mat(rows, NUMCOL);
    // mat = mat_map;   // this does a copy
    new (&mat_map_out) MapArrayXfRow(values_buf.data(), rows, NUMCOL);
    return;
}
