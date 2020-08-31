/*
Model class - v2
- OBJ models loading using Assimp library
- Convert data from Assimp data structure to a OpenGL-compatible data structure (Mesh class in mesh_v1.h)

N.B. 1) in this version of the class, eventual textures defined in the model (exported by modeling SWs) are loaded and applied

N.B. 2) adaptation of https://github.com/JoeyDeVries/LearnOpenGL/blob/master/includes/learnopengl/model.h

author: Davide Gadia

Real-Time Graphics Programming - a.a. 2018/2019
Master degree in Computer Science
Universita' degli Studi di Milano
*/



#pragma once
using namespace std;

// Std. Includes
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

// GL Includes
#include <glad/glad.h> // Contains all the necessery OpenGL includes
// we use GLM data structures to convert data in the Assimp data structures in a data structures suited for VBO, VAO and EBO buffers
#include <glm/glm.hpp>

// Assimp includes
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// we include the library for image loading
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image/stb_image.h>

// we include the Mesh class (v2), which manages the "OpenGL side" (= creation and allocation of VBO, VAO, EBO buffers) of the loading of models
#include <utils/Mesh.hpp>

// function used to load image data
GLint TextureFromFile(const char* path, string directory);


/////////////////// MODEL class ///////////////////////
class Model
{
public:
    // a vector with the loaded textures
    vector<Texture> textures_loaded;
    // at the end of loading, we will have a vector of Mesh class instances
    vector<Mesh> meshes;
    // the folder on disk of the model (needed for the loading of textures, if model is provided of textures)
    string directory;

    //////////////////////////////////////////

    // constructor
    Model(const string& path)
    {
        this->loadModel(path);
    }

    //////////////////////////////////////////

    // model rendering: calls rendering methods of each instance of Mesh class in the vector.
    // In this case, we pass also the Shader class instance, because it will be used for the textures
    void Draw(Shader shader)
    {
        for(GLuint i = 0; i < this->meshes.size(); i++)
            this->meshes[i].Draw(shader);
    }

    //////////////////////////////////////////

    // destructor. when application closes, we deallocate memory allocated by the instances of Mesh class
    virtual ~Model()
    {
        for(GLuint i = 0; i < this->meshes.size(); i++)
            this->meshes[i].Delete();
    }


private:

    //////////////////////////////////////////
    // loading of the model using Assimp library. Nodes are processed to build a vector of Mesh class instances
    void loadModel(string path)
    {
        // loading using Assimp
        // N.B.: it is possible to set, if needed, some operations to be performed by Assimp after the loading.
        // Details on the different flags to use are available at: http://assimp.sourceforge.net/lib_html/postprocess_8h.html#a64795260b95f5a4b3f3dc1be4f52e410
        // VERY IMPORTANT: calculation of Tangents and Bitangents is possible only if the model has Texture Coordinates
        // If they are not present, the calculation is skipped (but no error is provided in the foillowing checks!)
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_FlipUVs | aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace);

        // check for errors (see comment above)
        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
        {
            cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
            return;
        }

        // we get the folder on disk of the model
        this->directory = path.substr(0, path.find_last_of('/'));

        // we start the recursive processing of nodes in the Assimp data structure
        this->processNode(scene->mRootNode, scene);
    }

    //////////////////////////////////////////

    // Recursive processing of nodes of Assimp data structure
    void processNode(aiNode* node, const aiScene* scene)
    {
        // we process each mesh inside the current node
        for(GLuint i = 0; i < node->mNumMeshes; i++)
        {
            // the "node" object contains only the indices to objects in the scene
            // "Scene" contains all the data. Class node is used only to point to one or more mesh inside the scene and to maintain informations on relations between nodes
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            // we start processing of the Assimp mesh using processMesh method.
            // the result (an istance of the Mesh class) is added to the vector
            this->meshes.push_back(this->processMesh(mesh, scene));
        }
        // we then recursively process each of the children nodes
        for(GLuint i = 0; i < node->mNumChildren; i++)
        {
            this->processNode(node->mChildren[i], scene);
        }

    }

    //////////////////////////////////////////

    // Processing of the Assimp mesh in order to obtain an "OpenGL mesh"
    // = we create and allocate the buffers used to send mesh data to the GPU
    // In this case, we pass also aiScene instance, because we need to set the materials once loaded the textures
    Mesh processMesh(aiMesh* mesh, const aiScene* scene)
    {
      // data structures for vertices and indices of vertices (for faces)
        vector<Vertex> vertices;
        vector<GLuint> indices;
        // vector with all the model textures
        vector<Texture> textures;

        for(GLuint i = 0; i < mesh->mNumVertices; i++)
        {
            Vertex vertex;
            // the vector data type used by Assimp is different than the GLM vector needed to allocate the OpenGL buffers
            // I need to convert the data structures (from Assimp to GLM, which are fully compatible to the OpenGL)
            glm::vec3 vector;
            // vertices coordinates
            vector.x = mesh->mVertices[i].x;
            vector.y = mesh->mVertices[i].y;
            vector.z = mesh->mVertices[i].z;
            vertex.Position = vector;
            // Normals
            vector.x = mesh->mNormals[i].x;
            vector.y = mesh->mNormals[i].y;
            vector.z = mesh->mNormals[i].z;
            vertex.Normal = vector;
            // Texture Coordinates
            // if the model has texture coordinates, than we assign them to a GLM data structure, otherwise we set them at 0
            // if texture coordinates are present, than Assimp can calculate tangents and bitangents, otherwise we set them at 0 too
            if(mesh->mTextureCoords[0])
            {
                glm::vec2 vec;
                // in this example we assume the model has only one set of texture coordinates. Actually, a vertex can have ip to 8 different texture coordinates. For other models and formats, this code needs to be adapted and modified.
                vec.x = mesh->mTextureCoords[0][i].x;
                vec.y = mesh->mTextureCoords[0][i].y;
                vertex.TexCoords = vec;

                // Tangents
                vector.x = mesh->mTangents[i].x;
                vector.y = mesh->mTangents[i].y;
                vector.z = mesh->mTangents[i].z;
                vertex.Tangent = vector;
                // Bitangents
                vector.x = mesh->mBitangents[i].x;
                vector.y = mesh->mBitangents[i].y;
                vector.z = mesh->mBitangents[i].z;
                vertex.Bitangent = vector;
            }
            else{
                vertex.TexCoords = glm::vec2(0.0f, 0.0f);
                cout << "WARNING::ASSIMP:: MODEL WITHOUT UV COORDINATES -> TANGENT AND BITANGENT ARE = 0" << endl;
            }
            // we add the vertex to the list
            vertices.push_back(vertex);
        }

        // for each face of the mesh, we retrieve the indices of its vertices , and we store them in a vector data structure
        for(GLuint i = 0; i < mesh->mNumFaces; i++)
        {
            aiFace face = mesh->mFaces[i];
            for(GLuint j = 0; j < face.mNumIndices; j++)
                indices.push_back(face.mIndices[j]);
        }

        // we process the materials defined in the model file
        if(mesh->mMaterialIndex >= 0)
        {
            aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
            // We assume a convention for sampler names in the shaders. Each diffuse texture should be named
            // as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER.
            // Same applies to other texture as the following list summarizes:
            // Diffuse: texture_diffuseN
            // Specular: texture_specularN
            // Normal: texture_normalN

            // 1. Diffuse maps
            vector<Texture> diffuseMaps = this->loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
            textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
            // 2. Specular maps
            vector<Texture> specularMaps = this->loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
            textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
            // 3. Normal maps
            std::vector<Texture> normalMaps = this->loadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal");
            textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
            // 4. Height maps
            std::vector<Texture> heightMaps = this->loadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height");
            textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
        }

        // we return an instance of the Mesh class created using the vertices and faces data structures we have created above.
        return Mesh(vertices, indices, textures);
    }

    // Load (if not yet loaded) the textures defined in the model materials (if defined)
    vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, string typeName)
    {
        vector<Texture> textures;
        for(GLuint i = 0; i < mat->GetTextureCount(type); i++)
        {
            aiString str;
            mat->GetTexture(type, i, &str);
            // if texture has been already loaded, jump to the next one
            GLboolean skip = false;
            for(GLuint j = 0; j < textures_loaded.size(); j++)
            {
                if(textures_loaded[j].path == str)
                {
                    textures.push_back(textures_loaded[j]);
                    skip = true; // A texture with the same filepath has already been loaded, continue to next one. (optimization)
                    break;
                }
            }
            if(!skip)
            {   // If texture hasn't been loaded already, load it
                Texture texture;
                texture.id = TextureFromFile(str.C_Str(), this->directory);
                texture.type = typeName;
                texture.path = str;
                textures.push_back(texture);
                this->textures_loaded.push_back(texture);  // Store it as texture loaded for entire model, to ensure we won't unnecessary load duplicate textures.
            }
        }
        return textures;
    }
};

// we load texture from disk, and we create OpenGL Texture Unit
GLint TextureFromFile(const char* path, string directory)
{
     //Generate texture ID and load texture data
    string filename = string(path);
    filename = directory + '/' + filename;
    GLuint textureID;
    glGenTextures(1, &textureID);
    int width,height,channels;
    unsigned char* image = stbi_load(filename.c_str(), &width, &height, &channels, 0);

    // Assign texture to ID
    glBindTexture(GL_TEXTURE_2D, textureID);
    // 3 channels = RGB ; 4 channel = RGBA
    if (channels==3)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    else if (channels==4)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
    glGenerateMipmap(GL_TEXTURE_2D);

    // we set how to consider UVs outside [0,1] range
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    // we set the filtering for minification and magnification
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
    // we free the memory once we have created an OpenGL texture
    stbi_image_free(image);
    return textureID;
}
