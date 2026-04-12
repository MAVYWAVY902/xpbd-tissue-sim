#ifndef __MTL_PARSER_HPP
#define __MTL_PARSER_HPP

#include "common/types.hpp"
#include <string>
#include <map>
#include <optional>

namespace Utils
{

/** Represents material properties from an MTL file.
 * 
 * MTL (Material Template Library) format specification:
 * - Ka: Ambient color (r g b)
 * - Kd: Diffuse color (r g b) 
 * - Ks: Specular color (r g b)
 * - Ns: Specular exponent (shininess, 0-1000)
 * - d:  Opacity/dissolve (0=transparent, 1=opaque)
 * - Tr: Transparency (0=opaque, 1=transparent, inverse of d)
 * - illum: Illumination model (0-10)
 */
struct Material
{
    std::string name;
    
    Vec3r ambient_color = Vec3r(0.2, 0.2, 0.2);    // Ka - ambient reflection
    Vec3r diffuse_color = Vec3r(0.8, 0.8, 0.8);    // Kd - diffuse reflection
    Vec3r specular_color = Vec3r(1.0, 1.0, 1.0);   // Ks - specular reflection
    
    Real specular_exponent = 10.0;                  // Ns - specular shininess (0-1000)
    Real opacity = 1.0;                             // d or Tr - opacity (0-1)
    int illumination_model = 2;                     // illum - lighting model
    
    // Optional texture map filenames (not used yet, but parsed for future)
    std::optional<std::string> ambient_texture;     // map_Ka
    std::optional<std::string> diffuse_texture;     // map_Kd
    std::optional<std::string> specular_texture;    // map_Ks
    std::optional<std::string> bump_texture;        // map_Bump or bump
    
    Material() = default;
    Material(const std::string& mat_name) : name(mat_name) {}
    
    /** Check if this material has valid diffuse color (not default) */
    bool hasDiffuseColor() const {
        return diffuse_color != Vec3r(0.8, 0.8, 0.8);
    }
    
    /** Check if this material has specular highlights */
    bool hasSpecular() const {
        return specular_color.norm() > 0.01 && specular_exponent > 0;
    }
};

/** Parser for Wavefront MTL (Material Template Library) files.
 * 
 * Usage:
 *   MTLParser parser;
 *   auto materials = parser.parse("model.mtl");
 *   if (materials.count("material_name") > 0) {
 *       Material& mat = materials["material_name"];
 *       // Use mat.diffuse_color, mat.specular_color, etc.
 *   }
 */
class MTLParser
{
public:
    MTLParser() = default;
    ~MTLParser() = default;
    
    /** Parse an MTL file and return a map of material name -> Material.
     * @param filename Path to the MTL file
     * @return Map of material names to Material objects
     * @throws std::runtime_error if file cannot be opened or parsed
     */
    std::map<std::string, Material> parse(const std::string& filename);
    
    /** Parse MTL content from a string (useful for testing).
     * @param content MTL file content as string
     * @return Map of material names to Material objects
     */
    std::map<std::string, Material> parseFromString(const std::string& content);
    
    /** Find and parse the MTL file associated with an OBJ file.
     * Looks for 'mtllib' directive in the OBJ file.
     * @param obj_filename Path to the OBJ file
     * @return Map of material names to Material objects (empty if no MTL found)
     */
    std::map<std::string, Material> parseFromOBJ(const std::string& obj_filename);
    
private:
    /** Parse a single line of MTL content.
     * @param line The line to parse
     * @param current_material Current material being built
     */
    void _parseLine(const std::string& line, Material& current_material);
    
    /** Parse a Vec3r (RGB triplet) from a string stream.
     * @param line_stream String stream positioned after the command
     * @return Vec3r with the parsed values
     */
    Vec3r _parseVec3(std::istringstream& line_stream);
    
    /** Trim whitespace from both ends of a string.
     * @param str String to trim
     * @return Trimmed string
     */
    std::string _trim(const std::string& str);
};

} // namespace Utils

#endif // __MTL_PARSER_HPP
