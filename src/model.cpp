#include "../include/model.hpp"

Model::Model(){
    meshes.push_back(new Mesh());
}

Model::Model(string file_path){
    loadModel(file_path);
}

void Model::update(float dt){
    for(int i = 0;i < meshes.size();i++){
        meshes[i]->update(dt);
    }
}

void Model::renderModel(){
    for(int i = 0;i < meshes.size();i++){
        meshes[i]->renderMesh();
    }
}

void Model::processInput(Window &window){
    for(int i = 0;i < meshes.size();i++){
        meshes[i]->processInput(window);
    }
}

void Model::loadModel(string file_path){
    Assimp::Importer importer;
    //Require explicit triangulation since the object file may contain polygonal faces in general 
    const aiScene* scene = importer.ReadFile(file_path, aiProcess_Triangulate | aiProcess_FlipUVs);
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode){
        cout << "ERROR::ASSIMP::" << importer.GetErrorString() << endl;
        return;
    }
    this->processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode* node, const aiScene* scene){
    for(unsigned int i = 0; i < node->mNumMeshes; i++){
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes.push_back(processMesh(mesh, scene));
    }
    for(unsigned int i = 0; i < node->mNumChildren; i++){
        processNode(node->mChildren[i], scene);
    }
}

Mesh* Model::processMesh(aiMesh* mesh, const aiScene* scene){
    vector<vec3> vertices; 
    vector<unsigned int> indices;
    int N = mesh->mNumVertices;

    //Process Vertices
    for(unsigned int i=0;i<mesh->mNumVertices;i++){
        vec3 pos(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        // vec3d normal(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
        //normals read by assimp are not necessarily normalized
        vertices.push_back(pos);
    }

    //Process Faces
    for(unsigned int i=0;i<mesh->mNumFaces;i++){
        aiFace face = mesh->mFaces[i];
        for(unsigned int j=0;j<face.mNumIndices;j++){
            indices.push_back(face.mIndices[j]);
        }
    }

    Mesh* new_mesh = new Mesh(vertices, indices);

    return new_mesh;
}