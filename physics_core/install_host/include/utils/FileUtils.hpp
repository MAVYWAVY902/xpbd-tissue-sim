#ifndef __FILE_UTILS_HPP
#define __FILE_UTILS_HPP

#include "common/types.hpp"

#include <fstream>
#include <string>

struct FileUtils
{
    template <typename T>
    static std::vector<T> readVectorFromFile(const std::string& filename)
    {
        std::vector<T> vals;

        std::ifstream infile(filename);
        std::string line;

        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            T val;
            if (!(iss >> val)) { break; }

            // add vertex indices to set
            vals.push_back(val);
        }

        return vals;
    }
};

#endif