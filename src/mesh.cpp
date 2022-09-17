#include "../include/mesh.hpp"

//Mesh constructor
Mesh::Mesh(){                       
    upVec = vec3(0,0,1);

    vector<vec3> vertices;
    vector<unsigned int> indices;
    vector<bool> clamp;
 
    //Sample Mesh 1: M * N rectangular grid 
    for(int i = 0;i < 5;i++){
        for(int j = 0;j < 5;j++){
            vertices.push_back(vec3(2 * i - 4,2 * j,0));
            if((i==0 && j==0) || (i==4 && j==0)){
                clamp.push_back(true);
            }else{
                clamp.push_back(false);
            }
        }
    }

    for(int i = 0;i < 4;i++){
        for(int j = 0;j < 4;j++){
            indices.push_back(i*5+j);
            indices.push_back((i+1)*5 + j);
            indices.push_back((i+1)*5 + j+1);
            indices.push_back(i*5+j);
            indices.push_back((i+1)*5 + j+1);
            indices.push_back(i*5 + j + 1);
        }
    }

    // Sample Mesh 2: A single grid cell
    // vertices.push_back(vec3(2,2,0));
    // vertices.push_back(vec3(6,2,0));
    // vertices.push_back(vec3(2,6,0));
    // vertices.push_back(vec3(6,6,0));

    // indices.push_back(0);
    // indices.push_back(1);
    // indices.push_back(2);
    // indices.push_back(1);
    // indices.push_back(3);
    // indices.push_back(2);

    // clamp.push_back(true);
    // clamp.push_back(true);
    // clamp.push_back(false);
    // clamp.push_back(false);

    this->mesh = new HalfEdge(vertices, indices, clamp);

    //Extra springs for mesh 1
    for(int i = 0; i < 4; i++){
        for(int j = 1; j < 5; j++){
            double length = (mesh->particle_list[i*5+j]->position - mesh->particle_list[(i+1)*5+j-1]->position).norm();
            mesh->springs.push_back(Spring(i*5+j,(i+1)*5+j-1,length));
        }
    }

    //Extra springs for mesh 2
    // double length = (mesh->particle_list[0]->position - mesh->particle_list[3]->position).norm();
    // mesh->springs.push_back(Spring(0,3,length));
}

Mesh::Mesh(vector<vec3> vertices, vector<unsigned int> indices){
    //A mesh specified using Assimp 
    
    this->mesh = new HalfEdge(vertices, indices);
}

//Mesh Update
void Mesh::update(float dt){
    //Update the mesh
    count = (count + 1) % 2;
    if(isPlaying && count == 0){
        if(currIntersectIdx < intersectPts.size()){
            if(currIntersectIdx == 0){
                this->mesh->reMesh(intersectPts[currIntersectIdx], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), intersectPts[currIntersectIdx+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
            }else if(currIntersectIdx == intersectPts.size()-1){
                this->mesh->reMesh(intersectPts[currIntersectIdx], intersectPts[currIntersectIdx-1], make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1), vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
            }else{
                this->mesh->reMesh(intersectPts[currIntersectIdx], intersectPts[currIntersectIdx-1], intersectPts[currIntersectIdx+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
            }
            currIntersectIdx++;
        }else{
            isPlaying = false;
        }
    }

    this->mesh->updateMesh(dt);
}

//Setting up the path for cut/tear
void Mesh::setupPath(){
    //Custom path
    currentPath = new Path();
    vector<vec3> inputPts;
    vector<vec3> inputPts2;
    inputPts.push_back(vec3(3.234, 13.725, 0));
    inputPts.push_back(vec3(7.345, 21.345, 0));
    inputPts.push_back(vec3(10.237, 21.123, 0));
    inputPts.push_back(vec3(14.334, 13.721, 0));
    currentPath->addCurve(inputPts);
    inputPts2.push_back(vec3(14.334, 13.721, 0));
    inputPts2.push_back(vec3(18.234, 6.43, 0));
    inputPts2.push_back(vec3(20.34, 5.77, 0));
    inputPts2.push_back(vec3(23.237, 13.823, 0));
    currentPath->addCurve(inputPts2);

    //Setting up the intersection points
    vec3 startPoint = this->currentPath->lastPoint;
    vec3 startTangent = this->currentPath->lastTangent;
    bool isFirst = true;
    while(!this->currentPath->isOver){
        this->currentPath->updatePath();
        vec3 endPoint = this->currentPath->lastPoint;
        vec3 endTangent = this->currentPath->lastTangent;
        vec3 dir = (endPoint - startPoint).normalized();
        vec3 normal = upVec.cross(dir);
        Plane plane(startPoint, normal);
        vector<tuple<vec3, int, int>> intersections = this->mesh->Intersect(plane);
        auto filteredIntersections = filterAndSort(intersections, startPoint, endPoint, isFirst);
        for(auto intPt : filteredIntersections){
            this->intersectPts.push_back(intPt);
            this->normals.push_back(normal);
        }
        startPoint = endPoint;
        startTangent = endTangent;
        isFirst = false;
    }
    removeDuplicates(this->intersectPts);
}

//Mesh Process Input
void Mesh::processInput(Window &window){
    GLFWwindow *win = window.window;
    if(glfwGetKey(win, GLFW_KEY_P) == GLFW_PRESS){
        if(!isPlaying){
            isPlaying = true;
            setupPath();
        }
    }
}

void Mesh::removeDuplicates(vector<tuple<vec3,int,int>> &vertices){
    int i = 0;
    while(i < vertices.size()-1){
        if(get<0>(vertices[i]) == get<0>(vertices[i+1])){
            if(get<1>(vertices[i]) == 2){
                vertices.erase(vertices.begin()+i);
            }else{
                vertices.erase(vertices.begin()+i+1);
            }
        }else{
            i++;
        }
    }
}

//Filter intersection points
vector<tuple<vec3,int,int>> Mesh::filterAndSort(vector<tuple<vec3,int,int>> intersections, vec3 startPoint, vec3 endPoint,bool first){
    vector<tuple<vec3,int,int>> filteredIntersections;
    vec3 direction = endPoint - startPoint;
    double lowerBound = direction.dot(startPoint - startPoint);
    double upperBound = direction.dot(endPoint - startPoint);
    int currentSize = this->mesh->particle_list.size();
    if(first){
        this->mesh->particle_list.push_back(new Particle(startPoint));
        filteredIntersections.push_back(make_tuple(startPoint,2,currentSize++));
    }
    for(auto x : intersections){
        vec3 point = get<0>(x);
        double dist = direction.dot(point - startPoint);
        if(dist > lowerBound && dist < upperBound){
            filteredIntersections.push_back(x);
        }
    }
    auto directionSort = [startPoint, direction] (tuple<vec3, int, int> a, tuple<vec3, int, int> b) -> bool
    {
        double x = (get<0>(a) - startPoint).dot(direction);
        double y = (get<0>(b) - startPoint).dot(direction);
        return x <= y;  
    };
    sort(filteredIntersections.begin(),filteredIntersections.end(),directionSort);
    this->mesh->particle_list.push_back(new Particle(endPoint));
    filteredIntersections.push_back(make_tuple(endPoint,2,currentSize++));
    return filteredIntersections;
}

//Rendering the mesh
void Mesh::renderMesh(){
    int n = mesh->face_list.size();
    for(int i = 0; i < n; i++){
        Face* f = mesh->face_list[i];
        vec3 v1 = mesh->particle_list[f->indices[0]]->position;
        vec3 v2 = mesh->particle_list[f->indices[1]]->position;
        vec3 v3 = mesh->particle_list[f->indices[2]]->position;
        setColor(vec3(1.0f, 0.0f, 0.0f));
        drawTri(v1,v2,v3);
        setColor(vec3(0,0,0));
        setLineWidth(1);
        drawLine(v1,v2);
        drawLine(v2,v3);
        drawLine(v3,v1);
    }
}