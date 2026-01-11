#include "utils/MTLParser.hpp"
#include "common/colors.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <filesystem>

namespace Utils
{

std::map<std::string, Material> MTLParser::parse(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << KRED << "ERROR: Could not open MTL file: " << filename << RST << std::endl;
        return {};
    }
    
    std::cout << KGRN << "Loading MTL file: " << filename << RST << std::endl;
    
    std::map<std::string, Material> materials;
    Material current_material;
    bool has_current = false;
    
    std::string line;
    int line_number = 0;
    
    while (std::getline(file, line)) {
        line_number++;
        line = _trim(line);
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream line_stream(line);
        std::string command;
        line_stream >> command;
        
        // New material definition
        if (command == "newmtl") {
            // Save previous material if exists
            if (has_current && !current_material.name.empty()) {
                materials[current_material.name] = current_material;
                std::cout << "  - Material '" << current_material.name << "': "
                          << "Kd(" << current_material.diffuse_color.transpose() << ") "
                          << "Ks(" << current_material.specular_color.transpose() << ") "
                          << "Ns=" << current_material.specular_exponent << std::endl;
            }
            
            // Start new material
            std::string mat_name;
            line_stream >> mat_name;
            current_material = Material(mat_name);
            has_current = true;
        }
        // Ambient color
        else if (command == "Ka") {
            current_material.ambient_color = _parseVec3(line_stream);
        }
        // Diffuse color (main color)
        else if (command == "Kd") {
            current_material.diffuse_color = _parseVec3(line_stream);
        }
        // Specular color
        else if (command == "Ks") {
            current_material.specular_color = _parseVec3(line_stream);
        }
        // Specular exponent (shininess)
        else if (command == "Ns") {
            line_stream >> current_material.specular_exponent;
        }
        // Opacity (dissolve)
        else if (command == "d") {
            line_stream >> current_material.opacity;
        }
        // Transparency (inverse of opacity)
        else if (command == "Tr") {
            Real transparency;
            line_stream >> transparency;
            current_material.opacity = 1.0 - transparency;
        }
        // Illumination model
        else if (command == "illum") {
            line_stream >> current_material.illumination_model;
        }
        // Texture maps (parse but don't use yet)
        else if (command == "map_Ka") {
            std::string texture_file;
            line_stream >> texture_file;
            current_material.ambient_texture = texture_file;
        }
        else if (command == "map_Kd") {
            std::string texture_file;
            line_stream >> texture_file;
            current_material.diffuse_texture = texture_file;
        }
        else if (command == "map_Ks") {
            std::string texture_file;
            line_stream >> texture_file;
            current_material.specular_texture = texture_file;
        }
        else if (command == "map_Bump" || command == "bump") {
            std::string texture_file;
            line_stream >> texture_file;
            current_material.bump_texture = texture_file;
        }
    }
    
    // Save the last material
    if (has_current && !current_material.name.empty()) {
        materials[current_material.name] = current_material;
        std::cout << "  - Material '" << current_material.name << "': "
                  << "Kd(" << current_material.diffuse_color.transpose() << ") "
                  << "Ks(" << current_material.specular_color.transpose() << ") "
                  << "Ns=" << current_material.specular_exponent << std::endl;
    }
    
    std::cout << KGRN << "Loaded " << materials.size() << " material(s) from MTL file" << RST << std::endl;
    
    return materials;
}

std::map<std::string, Material> MTLParser::parseFromString(const std::string& content)
{
    std::map<std::string, Material> materials;
    Material current_material;
    bool has_current = false;
    
    std::istringstream content_stream(content);
    std::string line;
    
    while (std::getline(content_stream, line)) {
        line = _trim(line);
        
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        _parseLine(line, current_material);
        
        // Check if we started a new material
        if (line.substr(0, 6) == "newmtl") {
            if (has_current && !current_material.name.empty()) {
                materials[current_material.name] = current_material;
            }
            has_current = true;
        }
    }
    
    // Save last material
    if (has_current && !current_material.name.empty()) {
        materials[current_material.name] = current_material;
    }
    
    return materials;
}

std::map<std::string, Material> MTLParser::parseFromOBJ(const std::string& obj_filename)
{
    std::ifstream obj_file(obj_filename);
    if (!obj_file.is_open()) {
        std::cerr << KYEL << "WARNING: Could not open OBJ file to search for MTL: " 
                  << obj_filename << RST << std::endl;
        return {};
    }
    
    // Search for mtllib directive in OBJ file
    std::string line;
    std::string mtl_filename;
    
    while (std::getline(obj_file, line)) {
        line = _trim(line);
        
        if (line.substr(0, 6) == "mtllib") {
            std::istringstream line_stream(line);
            std::string command;
            line_stream >> command >> mtl_filename;
            break;
        }
    }
    
    obj_file.close();
    
    if (mtl_filename.empty()) {
        // No mtllib directive found, try to find MTL file with same name
        std::filesystem::path obj_path(obj_filename);
        std::filesystem::path mtl_path = obj_path.parent_path() / (obj_path.stem().string() + ".mtl");
        
        if (std::filesystem::exists(mtl_path)) {
            std::cout << KCYN << "INFO: No mtllib directive in OBJ, but found MTL file: " 
                      << mtl_path.string() << RST << std::endl;
            return parse(mtl_path.string());
        }
        
        std::cout << KYEL << "INFO: No MTL file found for OBJ: " << obj_filename << RST << std::endl;
        return {};
    }
    
    // Make MTL path relative to OBJ file location
    std::filesystem::path obj_path(obj_filename);
    std::filesystem::path mtl_path = obj_path.parent_path() / mtl_filename;
    
    if (!std::filesystem::exists(mtl_path)) {
        std::cerr << KYEL << "WARNING: MTL file referenced in OBJ not found: " 
                  << mtl_path.string() << RST << std::endl;
        return {};
    }
    
    return parse(mtl_path.string());
}

void MTLParser::_parseLine(const std::string& line, Material& current_material)
{
    // This method is used by parseFromString
    // Implementation is similar to the main parse() method
    // but works on pre-split lines
}

Vec3r MTLParser::_parseVec3(std::istringstream& line_stream)
{
    Vec3r result;
    line_stream >> result[0] >> result[1] >> result[2];
    return result;
}

std::string MTLParser::_trim(const std::string& str)
{
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

} // namespace Utils
