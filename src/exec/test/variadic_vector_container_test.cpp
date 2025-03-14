#include "common/VariadicVectorContainer.hpp"

#include <iostream>
#include <string>
#include <vector>

int main(int /* argc */, char** /* argv */)
{
    VariadicVectorContainer<double, int, std::string> c;
    c.push_back<std::string>("test1");
    c.push_back<std::string>("test2");
    c.push_back<int>(100);
    c.push_back<double>(0.1);

    const std::vector<int>& int_vec = c.get<int>();
    const std::vector<double>& double_vec = c.get<double>();
    std::vector<std::string>& string_vec = c.get<std::string>();
    string_vec.push_back("test3");

    std::cout << "int_vec:" << std::endl;
    for (const auto& elem : int_vec)
    {
        std::cout << elem << ", ";
    }

    std::cout << "\n\ndouble_vec:" << std::endl;
    for (const auto& elem : double_vec)
    {
        std::cout << elem << ", ";
    }

    std::cout << "\n\nstring_vec:" << std::endl;
    for (const auto& elem : string_vec)
    {
        std::cout << elem << ", ";
    }

    std::cout << "\n\nAll together:" << std::endl;

    c.for_each_element([](const auto& element)
    {
        std::cout << element << ", ";
    });

    std::cout << std::endl;
}