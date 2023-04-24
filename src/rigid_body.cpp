#include "../include/rigid_body.hpp"

RigidBody::RigidBody(){

}

RigidBody::RigidBody(string filePath){
    Assimp::Importer importer;
    //Require explicit triangulation since the object file may contain polygonal faces in general 
    const aiScene* scene = importer.ReadFile(filePath, aiProcess_Triangulate | aiProcess_FlipUVs);
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode){
        cout << "ERROR::ASSIMP::" << importer.GetErrorString() << endl;
        return;
    }
    
    //Process the mesh at the root node
    //We assume that the object file contains only one mesh
    aiMesh* mesh = scene->mMeshes[0];
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
        vector<unsigned int> indices;
        for(unsigned int j=0;j<face.mNumIndices;j++){
            indices.push_back(face.mIndices[j]);
        }
        faces.push_back(Primitive(indices[0], indices[1], indices[2]));
    }

    //Process Material
    if(mesh->mMaterialIndex >= 0){
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
        aiColor3D color(0.0f, 0.0f, 0.0f);
        material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
        mat = Material(vec3(color.r, color.g, color.b));
    }

    //Default Parameters
    responseMode = 1;
    collisionMode = 1;
    drawRBD = true;
    velocity = vec3(0,0,0);
    if(collisionMode){
        //Approximating sphere
        vec3 centre = vec3(0,0,4.0f);
        double radius = 1.5f;
        sphere = Sphere(centre, radius);
    }else{
        //Bounding Volume Hierarchy
        //Number of nodes = 2 * number of primitives (-1 actually) 
        //Provided every leaf node has only one primitive
    }
}

void RigidBody::update(float dt){
    if(activatePhysics){
        for(int i = 0;i < vertices.size();i++){
            vertices[i] += velocity * dt;
        }
        if(collisionMode){
            sphere.centre += velocity * dt;
        }
    }
}

void RigidBody::processInput(Window &window){
    GLFWwindow* win = window.window;
    if(!window.debug){
        if(glfwGetKey(win, GLFW_KEY_O) == GLFW_PRESS){
            if(!activatePhysics){
                activatePhysics = true;
            }
        }else if(glfwGetKey(win, GLFW_KEY_I) == GLFW_PRESS){
            if(activatePhysics){
                activatePhysics = false;
            }
        }else if(glfwGetKey(win, GLFW_KEY_2) == GLFW_PRESS){
            drawRBD = false;
        }else if(glfwGetKey(win, GLFW_KEY_1) == GLFW_PRESS){
            drawRBD = true;
        }else if(glfwGetKey(win, GLFW_KEY_UP) == GLFW_PRESS){
            velocity = vec3(0.0f, 3.0f, 0.0f);
        }else if(glfwGetKey(win, GLFW_KEY_DOWN) == GLFW_PRESS){
            velocity = vec3(0.0f, -3.0f, 0.0f);
        }else if(glfwGetKey(win, GLFW_KEY_LEFT) == GLFW_PRESS){
            velocity = vec3(-3.0f, 0.0f, 0.0f);
        }else if(glfwGetKey(win, GLFW_KEY_RIGHT) == GLFW_PRESS){
            velocity = vec3(3.0f, 0.0f, 0.0f);
        }else if(glfwGetKey(win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS){
            velocity = vec3(0.0f, 0.0f, 3.0f);
        }else if(glfwGetKey(win, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS){
            velocity = vec3(0.0f, 0.0f, -3.0f);
        }
    }
}

void RigidBody::renderMesh(){
    int m = faces.size();

    if(drawRBD){
        //Drawing faces
        for(int i = 0; i < m; i++){
            glEnable(GL_BLEND);
            glDepthMask(GL_FALSE);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            vec3 v1 = vertices[faces[i].v1];
            vec3 v2 = vertices[faces[i].v2];
            vec3 v3 = vertices[faces[i].v3];
            setColor(vec4(mat.albedo[0], mat.albedo[1], mat.albedo[2], 0.5f));
            drawTri(v1, v2, v3);

            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);    
        }
    }
}

void RigidBody::printMeshInfo(){

}

bool RigidBody::checkSanity(){
    return true;
}