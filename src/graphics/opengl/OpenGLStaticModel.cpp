#include "graphics/opengl/OpenGLStaticModel.hpp"
#include "graphics/opengl/stb_image.h"

#include <GL/glew.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <algorithm>
#include <iostream>
#include <queue>
#include <dirent.h>

namespace Graphics
{

OpenGLStaticModel::OpenGLStaticModel(const std::string& filepath, const Eigen::Matrix4f& model_transform)
    : _model_transform(model_transform)
{
    _loadFromFile(filepath);
}

OpenGLStaticModel::~OpenGLStaticModel()
{
    for (auto& sm : _submeshes)
    {
        if (sm.vao) glDeleteVertexArrays(1, &sm.vao);
        if (sm.vbo) glDeleteBuffers(1, &sm.vbo);
        if (sm.nbo) glDeleteBuffers(1, &sm.nbo);
        if (sm.tbo) glDeleteBuffers(1, &sm.tbo);
        if (sm.tanbo) glDeleteBuffers(1, &sm.tanbo);
        if (sm.ebo) glDeleteBuffers(1, &sm.ebo);
        if (sm.texture_id) glDeleteTextures(1, &sm.texture_id);
        if (sm.normalmap_id) glDeleteTextures(1, &sm.normalmap_id);
    }
}

// ==================== Loading ====================

void OpenGLStaticModel::_loadFromFile(const std::string& filepath)
{
    // Store the directory for loading external textures
    std::string model_dir = filepath.substr(0, filepath.find_last_of("/\\") + 1);

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filepath,
        aiProcess_Triangulate |
        aiProcess_GenNormals |
        aiProcess_CalcTangentSpace |
        aiProcess_FlipUVs);

    if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !scene->mRootNode)
    {
        std::cerr << "[OpenGLStaticModel] ERROR loading '" << filepath
                  << "': " << importer.GetErrorString() << std::endl;
        return;
    }

    std::cout << "[OpenGLStaticModel] Loading '" << filepath
              << "' (" << scene->mNumMeshes << " meshes)" << std::endl;

    // BFS traverse all nodes to process all meshes
    std::queue<aiNode*> node_queue;
    node_queue.push(scene->mRootNode);

    while (!node_queue.empty())
    {
        aiNode* node = node_queue.front();
        node_queue.pop();

        for (unsigned int i = 0; i < node->mNumMeshes; ++i)
        {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];

            SubMesh sm;

            // Extract vertex data
            sm.positions.reserve(mesh->mNumVertices * 3);
            sm.normals.reserve(mesh->mNumVertices * 3);
            sm.tangents.reserve(mesh->mNumVertices * 3);
            sm.texcoords.reserve(mesh->mNumVertices * 2);

            for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
            {
                sm.positions.push_back(mesh->mVertices[v].x);
                sm.positions.push_back(mesh->mVertices[v].y);
                sm.positions.push_back(mesh->mVertices[v].z);

                if (mesh->mNormals)
                {
                    sm.normals.push_back(mesh->mNormals[v].x);
                    sm.normals.push_back(mesh->mNormals[v].y);
                    sm.normals.push_back(mesh->mNormals[v].z);
                }
                else
                {
                    sm.normals.push_back(0.0f);
                    sm.normals.push_back(1.0f);
                    sm.normals.push_back(0.0f);
                }

                if (mesh->mTangents)
                {
                    sm.tangents.push_back(mesh->mTangents[v].x);
                    sm.tangents.push_back(mesh->mTangents[v].y);
                    sm.tangents.push_back(mesh->mTangents[v].z);
                }
                else
                {
                    sm.tangents.push_back(1.0f);
                    sm.tangents.push_back(0.0f);
                    sm.tangents.push_back(0.0f);
                }

                if (mesh->mTextureCoords[0])
                {
                    sm.texcoords.push_back(mesh->mTextureCoords[0][v].x);
                    sm.texcoords.push_back(mesh->mTextureCoords[0][v].y);
                }
                else
                {
                    sm.texcoords.push_back(0.0f);
                    sm.texcoords.push_back(0.0f);
                }
            }

            // Extract indices
            for (unsigned int f = 0; f < mesh->mNumFaces; ++f)
            {
                const aiFace& face = mesh->mFaces[f];
                for (unsigned int j = 0; j < face.mNumIndices; ++j)
                    sm.indices.push_back(face.mIndices[j]);
            }
            sm.num_indices = static_cast<int>(sm.indices.size());

            // Extract material properties
            if (mesh->mMaterialIndex < scene->mNumMaterials)
            {
                aiMaterial* mat = scene->mMaterials[mesh->mMaterialIndex];

                // Read material diffuse color
                aiColor4D diffuse_color;
                if (mat->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse_color) == AI_SUCCESS)
                {
                    sm.color_r = diffuse_color.r;
                    sm.color_g = diffuse_color.g;
                    sm.color_b = diffuse_color.b;
                    sm.color_a = diffuse_color.a;
                }

                // Also try base color factor (PBR/glTF)
                aiColor4D base_color;
                if (mat->Get(AI_MATKEY_BASE_COLOR, base_color) == AI_SUCCESS)
                {
                    sm.color_r = base_color.r;
                    sm.color_g = base_color.g;
                    sm.color_b = base_color.b;
                    sm.color_a = base_color.a;
                }

                // Try to extract diffuse/base_color texture
                aiString tex_path;
                bool found_texture = false;

                // Only try actual diffuse/base_color types (NOT UNKNOWN — that's often metallic/roughness)
                const aiTextureType tex_types[] = {
                    aiTextureType_DIFFUSE,
                    aiTextureType_BASE_COLOR,
                };

                for (auto tex_type : tex_types)
                {
                    if (found_texture) break;
                    if (mat->GetTexture(tex_type, 0, &tex_path) == AI_SUCCESS)
                    {
                        const aiTexture* embedded = scene->GetEmbeddedTexture(tex_path.C_Str());
                        if (embedded)
                        {
                            if (embedded->mHeight == 0)
                            {
                                int w, h, ch;
                                unsigned char* pixels = stbi_load_from_memory(
                                    reinterpret_cast<const unsigned char*>(embedded->pcData),
                                    embedded->mWidth, &w, &h, &ch, 0);
                                if (pixels)
                                {
                                    sm.tex_width = w;
                                    sm.tex_height = h;
                                    sm.tex_channels = ch;
                                    sm.texture_pixels.assign(pixels, pixels + w * h * ch);
                                    sm.has_texture = true;
                                    found_texture = true;
                                    stbi_image_free(pixels);
                                }
                            }
                            else
                            {
                                int w = embedded->mWidth;
                                int h = embedded->mHeight;
                                sm.tex_width = w;
                                sm.tex_height = h;
                                sm.tex_channels = 4;
                                sm.texture_pixels.resize(w * h * 4);
                                const unsigned char* src = reinterpret_cast<const unsigned char*>(embedded->pcData);
                                for (int p = 0; p < w * h; ++p)
                                {
                                    sm.texture_pixels[p * 4 + 0] = src[p * 4 + 1];
                                    sm.texture_pixels[p * 4 + 1] = src[p * 4 + 2];
                                    sm.texture_pixels[p * 4 + 2] = src[p * 4 + 3];
                                    sm.texture_pixels[p * 4 + 3] = src[p * 4 + 0];
                                }
                                sm.has_texture = true;
                                found_texture = true;
                            }
                        }
                        else
                        {
                            // External texture file — try multiple search strategies
                            std::string tex_str = tex_path.C_Str();

                            // Extract just the filename (handle both / and \ separators for Windows paths)
                            std::string tex_filename = tex_str;
                            auto last_sep = tex_str.find_last_of("/\\");
                            if (last_sep != std::string::npos)
                                tex_filename = tex_str.substr(last_sep + 1);

                            // Try paths in order: original, model_dir + filename, model_dir/textures/ + filename
                            std::vector<std::string> try_paths = {
                                model_dir + tex_str,
                                model_dir + tex_filename,
                                model_dir + "textures/" + tex_filename,
                            };

                            bool loaded = false;
                            for (const auto& try_path : try_paths)
                            {
                                if (loaded) break;
                                int w, h, ch;
                                unsigned char* pixels = stbi_load(try_path.c_str(), &w, &h, &ch, 0);
                                if (pixels)
                                {
                                    sm.tex_width = w;
                                    sm.tex_height = h;
                                    sm.tex_channels = ch;
                                    sm.texture_pixels.assign(pixels, pixels + w * h * ch);
                                    sm.has_texture = true;
                                    found_texture = true;
                                    loaded = true;
                                    stbi_image_free(pixels);
                                    std::cout << "[OpenGLStaticModel]     Loaded external texture: " << try_path << std::endl;
                                }
                            }
                            if (!loaded)
                            {
                                std::cerr << "[OpenGLStaticModel]     Failed to load external texture: " << tex_str
                                          << " (tried " << try_paths.size() << " paths)" << std::endl;
                            }
                        }
                    }
                }

                // If no texture found via Assimp, scan model directory and textures/ subdirectory
                // for a file matching the material name + "base" keyword in its filename
                if (!found_texture)
                {
                    aiString mat_name_str;
                    mat->Get(AI_MATKEY_NAME, mat_name_str);
                    std::string mat_name_s = mat_name_str.C_Str();

                    // Directories to search: model_dir itself, and model_dir/textures/
                    std::vector<std::string> search_dirs = { model_dir, model_dir + "textures/" };

                    for (const auto& search_dir : search_dirs)
                    {
                        if (found_texture) break;
                        DIR* dir = opendir(search_dir.c_str());
                        if (!dir) continue;

                        struct dirent* entry;
                        while ((entry = readdir(dir)) != nullptr)
                        {
                            if (found_texture) break;
                            std::string fname = entry->d_name;
                            // Match files containing the material name AND "_base" (covers "base_color", "_base.", etc.)
                            // Also skip normal/roughness/emissive maps
                            if (fname.find(mat_name_s) != std::string::npos &&
                                fname.find("_base") != std::string::npos &&
                                fname.find("Normal") == std::string::npos &&
                                fname.find("Roughness") == std::string::npos &&
                                fname.find("emmisive") == std::string::npos &&
                                fname.find("emissive") == std::string::npos)
                            {
                                std::string full_path = search_dir + fname;
                                int w, h, ch;
                                unsigned char* pixels = stbi_load(full_path.c_str(), &w, &h, &ch, 0);
                                if (pixels)
                                {
                                    sm.tex_width = w;
                                    sm.tex_height = h;
                                    sm.tex_channels = ch;
                                    sm.texture_pixels.assign(pixels, pixels + w * h * ch);
                                    sm.has_texture = true;
                                    found_texture = true;
                                    stbi_image_free(pixels);
                                    std::cout << "[OpenGLStaticModel]     Auto-matched texture: " << full_path << std::endl;
                                }
                            }
                        }
                        closedir(dir);
                    }

                    // If still no texture, try matching just "_base" without material name
                    // (some models use generic texture names like "t_floor_tiles_base.jpg")
                    if (!found_texture)
                    {
                        for (const auto& search_dir : search_dirs)
                        {
                            if (found_texture) break;
                            DIR* dir = opendir(search_dir.c_str());
                            if (!dir) continue;

                            struct dirent* entry;
                            while ((entry = readdir(dir)) != nullptr)
                            {
                                if (found_texture) break;
                                std::string fname = entry->d_name;
                                // Try a looser match: just look for material name anywhere in the filename
                                // Convert material name to lowercase for case-insensitive matching
                                std::string fname_lower = fname;
                                std::string mat_lower = mat_name_s;
                                std::transform(fname_lower.begin(), fname_lower.end(), fname_lower.begin(), ::tolower);
                                std::transform(mat_lower.begin(), mat_lower.end(), mat_lower.begin(), ::tolower);

                                if (mat_lower.length() > 1 &&
                                    fname_lower.find(mat_lower) != std::string::npos &&
                                    fname_lower.find("normal") == std::string::npos &&
                                    fname_lower.find("roughness") == std::string::npos &&
                                    fname_lower.find("emissive") == std::string::npos &&
                                    fname_lower.find("emmisive") == std::string::npos &&
                                    fname_lower.find("metallic") == std::string::npos &&
                                    fname_lower.find("ambient_occlusion") == std::string::npos &&
                                    fname_lower.find("rma") == std::string::npos &&
                                    (fname_lower.find("base_color") != std::string::npos ||
                                     fname_lower.find("_base") != std::string::npos ||
                                     fname_lower.find("diffuse") != std::string::npos ||
                                     fname_lower.find("albedo") != std::string::npos))
                                {
                                    std::string full_path = search_dir + fname;
                                    int w, h, ch;
                                    unsigned char* pixels = stbi_load(full_path.c_str(), &w, &h, &ch, 0);
                                    if (pixels)
                                    {
                                        sm.tex_width = w;
                                        sm.tex_height = h;
                                        sm.tex_channels = ch;
                                        sm.texture_pixels.assign(pixels, pixels + w * h * ch);
                                        sm.has_texture = true;
                                        found_texture = true;
                                        stbi_image_free(pixels);
                                        std::cout << "[OpenGLStaticModel]     Auto-matched texture (loose): " << full_path << std::endl;
                                    }
                                }
                            }
                            closedir(dir);
                        }
                    }
                }

                // Normal map loading disabled — user prefers smoother look

                // Read roughness value from material
                float roughness_val = 0.5f;
                if (mat->Get(AI_MATKEY_ROUGHNESS_FACTOR, roughness_val) == AI_SUCCESS)
                    sm.roughness = roughness_val;

                // Debug: print material info
                aiString mat_name;
                mat->Get(AI_MATKEY_NAME, mat_name);
                std::cout << "[OpenGLStaticModel]   Mesh " << _submeshes.size()
                          << ": \"" << mat_name.C_Str() << "\""
                          << " verts=" << mesh->mNumVertices
                          << " color=(" << sm.color_r << "," << sm.color_g << "," << sm.color_b << ")"
                          << " texture=" << (sm.has_texture ? "YES" : "NO")
                          << " normalmap=" << (sm.has_normalmap ? "YES" : "NO")
                          << " roughness=" << sm.roughness
                          << std::endl;

                // Debug: enumerate ALL texture types for this material
                const struct { aiTextureType type; const char* name; } all_tex_types[] = {
                    {aiTextureType_DIFFUSE, "DIFFUSE"},
                    {aiTextureType_SPECULAR, "SPECULAR"},
                    {aiTextureType_AMBIENT, "AMBIENT"},
                    {aiTextureType_EMISSIVE, "EMISSIVE"},
                    {aiTextureType_NORMALS, "NORMALS"},
                    {aiTextureType_BASE_COLOR, "BASE_COLOR"},
                    {aiTextureType_METALNESS, "METALNESS"},
                    {aiTextureType_DIFFUSE_ROUGHNESS, "ROUGHNESS"},
                    {aiTextureType_AMBIENT_OCCLUSION, "AO"},
                    {aiTextureType_UNKNOWN, "UNKNOWN"},
                };
                for (const auto& tt : all_tex_types)
                {
                    unsigned int count = mat->GetTextureCount(tt.type);
                    if (count > 0)
                    {
                        aiString p;
                        mat->GetTexture(tt.type, 0, &p);
                        std::cout << "[OpenGLStaticModel]     -> " << tt.name
                                  << " x" << count << " path=\"" << p.C_Str() << "\""
                                  << std::endl;
                    }
                }
            }

            _submeshes.push_back(std::move(sm));
        }

        for (unsigned int c = 0; c < node->mNumChildren; ++c)
            node_queue.push(node->mChildren[c]);
    }

    std::cout << "[OpenGLStaticModel] Loaded " << _submeshes.size() << " sub-meshes." << std::endl;
}

// ==================== GL Init (lazy) ====================

void OpenGLStaticModel::_ensureGLInitialized() const
{
    if (_gl_initialized) return;
    _gl_initialized = true;

    for (auto& sm : _submeshes)
    {
        glGenVertexArrays(1, &sm.vao);
        glBindVertexArray(sm.vao);

        // Positions (location 0)
        glGenBuffers(1, &sm.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, sm.vbo);
        glBufferData(GL_ARRAY_BUFFER, sm.positions.size() * sizeof(float), sm.positions.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        // Normals (location 1)
        glGenBuffers(1, &sm.nbo);
        glBindBuffer(GL_ARRAY_BUFFER, sm.nbo);
        glBufferData(GL_ARRAY_BUFFER, sm.normals.size() * sizeof(float), sm.normals.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        // Tex coords (location 2)
        if (!sm.texcoords.empty())
        {
            glGenBuffers(1, &sm.tbo);
            glBindBuffer(GL_ARRAY_BUFFER, sm.tbo);
            glBufferData(GL_ARRAY_BUFFER, sm.texcoords.size() * sizeof(float), sm.texcoords.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        }

        // Tangents (location 3) — for normal mapping
        if (!sm.tangents.empty())
        {
            glGenBuffers(1, &sm.tanbo);
            glBindBuffer(GL_ARRAY_BUFFER, sm.tanbo);
            glBufferData(GL_ARRAY_BUFFER, sm.tangents.size() * sizeof(float), sm.tangents.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(3);
            glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        }

        // Indices
        glGenBuffers(1, &sm.ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sm.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sm.indices.size() * sizeof(unsigned int), sm.indices.data(), GL_STATIC_DRAW);

        glBindVertexArray(0);

        // Diffuse texture
        if (sm.has_texture)
        {
            GLenum fmt = (sm.tex_channels == 4) ? GL_RGBA : GL_RGB;
            glGenTextures(1, &sm.texture_id);
            glBindTexture(GL_TEXTURE_2D, sm.texture_id);
            glTexImage2D(GL_TEXTURE_2D, 0, fmt, sm.tex_width, sm.tex_height, 0, fmt, GL_UNSIGNED_BYTE, sm.texture_pixels.data());
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);

            std::cout << "[OpenGLStaticModel] Diffuse texture uploaded: "
                      << sm.tex_width << "x" << sm.tex_height << " (" << sm.tex_channels << "ch)" << std::endl;
        }

        // Normal map texture
        if (sm.has_normalmap)
        {
            GLenum fmt = (sm.nmap_channels == 4) ? GL_RGBA : GL_RGB;
            glGenTextures(1, &sm.normalmap_id);
            glBindTexture(GL_TEXTURE_2D, sm.normalmap_id);
            glTexImage2D(GL_TEXTURE_2D, 0, fmt, sm.nmap_width, sm.nmap_height, 0, fmt, GL_UNSIGNED_BYTE, sm.normalmap_pixels.data());
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);

            std::cout << "[OpenGLStaticModel] Normal map uploaded: "
                      << sm.nmap_width << "x" << sm.nmap_height << " (" << sm.nmap_channels << "ch)" << std::endl;
        }
    }
}

// ==================== Draw ====================

void OpenGLStaticModel::draw(unsigned int shader_program) const
{
    _ensureGLInitialized();

    // Set model matrix for this static object
    glUniformMatrix4fv(glGetUniformLocation(shader_program, "uModel"), 1, GL_FALSE, _model_transform.data());

    Eigen::Matrix3f normalMat = _model_transform.block<3,3>(0,0).inverse().transpose();
    glUniformMatrix3fv(glGetUniformLocation(shader_program, "uNormalMatrix"), 1, GL_FALSE, normalMat.data());

    // Enable lighting
    glUniform1i(glGetUniformLocation(shader_program, "uUseLighting"), 1);

    for (const auto& sm : _submeshes)
    {
        if (sm.num_indices == 0) continue;

        // Set roughness
        glUniform1f(glGetUniformLocation(shader_program, "uRoughness"), sm.roughness);

        // Bind diffuse texture or set color
        if (sm.has_texture && sm.texture_id)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, sm.texture_id);
            glUniform1i(glGetUniformLocation(shader_program, "uTexture"), 0);
            glUniform1i(glGetUniformLocation(shader_program, "uUseTexture"), 1);
        }
        else
        {
            glUniform1i(glGetUniformLocation(shader_program, "uUseTexture"), 0);
            glUniform4f(glGetUniformLocation(shader_program, "uColor"), sm.color_r, sm.color_g, sm.color_b, sm.color_a);
        }

        // Bind normal map
        if (sm.has_normalmap && sm.normalmap_id)
        {
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, sm.normalmap_id);
            glUniform1i(glGetUniformLocation(shader_program, "uNormalMap"), 1);
            glUniform1i(glGetUniformLocation(shader_program, "uUseNormalMap"), 1);
        }
        else
        {
            glUniform1i(glGetUniformLocation(shader_program, "uUseNormalMap"), 0);
        }

        glBindVertexArray(sm.vao);
        glDrawElements(GL_TRIANGLES, sm.num_indices, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);

        // Unbind textures
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    // Restore identity model matrix for other objects
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    glUniformMatrix4fv(glGetUniformLocation(shader_program, "uModel"), 1, GL_FALSE, identity.data());
    Eigen::Matrix3f identityNorm = Eigen::Matrix3f::Identity();
    glUniformMatrix3fv(glGetUniformLocation(shader_program, "uNormalMatrix"), 1, GL_FALSE, identityNorm.data());
}

} // namespace Graphics
