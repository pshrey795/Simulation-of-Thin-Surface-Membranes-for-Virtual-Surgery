#ifndef MODEL_HPP
#define MODEL_HPP

#include "mesh.hpp"
using namespace std;
using namespace Eigen;

#include<assimp/Importer.hpp>
#include<assimp/scene.h>
#include<assimp/postprocess.h>

class Model {

    public:
        Model();
        Model(string file_path);
        void update(float dt);
        void processInput(Window &window);
        void renderModel();

    private:
        vector<Mesh*> meshes;
        void loadModel(string file_path);
        void processNode(aiNode* node, const aiScene* scene);
        Mesh* processMesh(aiMesh* mesh, const aiScene* scene);

};

#endif