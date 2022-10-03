#include "../include/mesh.hpp"

//Mesh constructor
Mesh::Mesh(){                       
    //Using a sample mesh
    upVec = vec3(0,0,1);

    vector<vec3> vertices;
    vector<unsigned int> indices;
    vector<bool> clamp; 

    // // Sample mesh 1
    // for(int i = 0;i < 5;i++){
    //     for(int j = 0;j < 5;j++){
    //         vertices.push_back(vec3(i*2 - 4.0f,j*2 - 4.0f,0));
    //         if((i==0 && j==0) || (i==4 && j==0) || (i==0 && j==4) || (i==4 && j==4)){
    //             clamp.push_back(true);
    //         }else{
    //             clamp.push_back(false);
    //         }
    //     }
    // }

    // for(int i = 0;i < 4;i++){
    //     for(int j = 0;j < 4;j++){
    //         indices.push_back(i*5+j);
    //         indices.push_back((i+1)*5 + j);
    //         indices.push_back((i+1)*5 + j+1);
    //         indices.push_back(i*5+j);
    //         indices.push_back((i+1)*5 + j+1);
    //         indices.push_back(i*5 + j + 1);
    //     }
    // }

    //Sample mesh 2
    vertices.push_back(vec3(-4,0,0));
    vertices.push_back(vec3(0,4,0));
    vertices.push_back(vec3(4,0,0));
    vertices.push_back(vec3(0,-4,0));
    vertices.push_back(vec3(0,0,0));

    indices.push_back(0);
    indices.push_back(4);
    indices.push_back(1);
    indices.push_back(4);
    indices.push_back(2);
    indices.push_back(1);
    indices.push_back(0);
    indices.push_back(3);
    indices.push_back(4);
    indices.push_back(3);
    indices.push_back(2);
    indices.push_back(4);
    

    this->mesh = new HalfEdge(vertices, indices);  

    checkSanity();
    cout << "\n\n\n";
}

Mesh::Mesh(vector<vec3> vertices, vector<unsigned int> indices){
    //A mesh specified using Assimp 
    this->mesh = new HalfEdge(vertices, indices);
    checkSanity();
    cout << "\n\n\n";
}

//Mesh Update
void Mesh::update(float dt){
    //Update the mesh
    count = (count + 1) % 2;
    if(isPlaying){
        if(count == 0){
            if(currIntersectIdx < intersectPts.size()){
                if(currIntersectIdx == 0){
                    auto newIntPt = make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1);
                    // this->mesh->reMesh(intersectPts[currIntersectIdx], newIntPt, intersectPts[currIntersectIdx+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
                    this->mesh->reMesh2(newIntPt, intersectPts[currIntersectIdx], intersectPts[currIntersectIdx+1], crossEdgeLeft, crossEdgeRight, normals[currIntersectIdx]);
                }else if(currIntersectIdx == intersectPts.size()-1){
                    auto newIntPt = make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1);
                    // this->mesh->reMesh(intersectPts[currIntersectIdx], intersectPts[currIntersectIdx-1], newIntPt, vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
                    this->mesh->reMesh2(intersectPts[currIntersectIdx-1], intersectPts[currIntersectIdx], newIntPt, crossEdgeLeft, crossEdgeRight, normals[currIntersectIdx]);
                }else{
                    // this->mesh->reMesh(intersectPts[currIntersectIdx], intersectPts[currIntersectIdx-1], intersectPts[currIntersectIdx+1], vertexLast, crossEdgeLeft, crossEdgeRight, sideEdgeLeft, sideEdgeRight, normals[currIntersectIdx]);
                    this->mesh->reMesh2(intersectPts[currIntersectIdx-1], intersectPts[currIntersectIdx], intersectPts[currIntersectIdx+1], crossEdgeLeft, crossEdgeRight, normals[currIntersectIdx]);
                }
                checkSanity();
                cout << "\n\n\n";
                currIntersectIdx++;
            }
        }
    }
    if(activatePhysics && !isPlaying){
        this->mesh->updateMesh(dt);
    }
}

//Setting up the path for cut/tear
void Mesh::setupPath(){
    //Custom path
    currentPath = new Path();
    vector<vec3> inputPts;
    // //Straight Cut
    inputPts.push_back(vec3(-3.5, -3, 0));
    inputPts.push_back(vec3(-1.5, -1, 0));
    inputPts.push_back(vec3(1.5, 1, 0));
    inputPts.push_back(vec3(2.5, 3, 0));
    //Curved Cut
    // inputPts.push_back(vec3(-3.226, -1.226, 0));
    // inputPts.push_back(vec3(-1.47, 3.435, 0));
    // inputPts.push_back(vec3(1.47, 3.435, 0));
    // inputPts.push_back(vec3(3.226, -1.226, 0));
    currentPath->addCurve(inputPts);

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
    removeFacePts(this->intersectPts);
    currIntersectIdx = 0;
}

void Mesh::setupCut(){
    //Define a sample plane
    Plane p(vec3(-5.0f, 5.0f, 0.0f), vec3(1.0f, 1.0f, 0.0f));

    //Get the intersection points
    this->intersectPts = dirSort(this->mesh->Intersect(p), p);
    removeDuplicates(this->intersectPts);

    for(int i=0;i<intersectPts.size();i++){
        auto intPt = get<0>(intersectPts[i]);
        // cout << intPt[0] << " " << intPt[1] << " " << intPt[2] << "\n";
        normals.push_back(p.normal);
    }

    currIntersectIdx = 0;
}

//Mesh Process Input
void Mesh::processInput(Window &window){
    GLFWwindow *win = window.window;
    if(glfwGetKey(win, GLFW_KEY_P) == GLFW_PRESS){
        if(!isPlaying){
            isPlaying = true;
            // setupPath();
            setupCut();
        }
    }else if(glfwGetKey(win, GLFW_KEY_O) == GLFW_PRESS){
        if(!activatePhysics){
            activatePhysics = true;
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

void Mesh::removeFacePts(vector<tuple<vec3, int, int>> &vertices){
    int i = 1;
    while(i < vertices.size() - 1){
        if(get<1>(vertices[i]) == 2){
            vertices.erase(vertices.begin()+i);
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
        int faceIndex;
        for(int i=0;i<mesh->face_list.size();i++){
            if(mesh->isInside(mesh->face_list[i],startPoint)){
                faceIndex = i;
                break;
            }
        }
        vec3 newStartPoint = mesh->getPosAtFace(mesh->face_list[faceIndex],startPoint);
        Particle* newParticle = new Particle(newStartPoint, startPoint); 
        this->mesh->particle_list.push_back(newParticle);
        filteredIntersections.push_back(make_tuple(newStartPoint,2,currentSize++));
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
    int faceIndex;
        for(int i=0;i<mesh->face_list.size();i++){
        if(mesh->isInside(mesh->face_list[i],endPoint)){
            faceIndex = i;
            break;
        }
    }
    vec3 newEndPoint = mesh->getPosAtFace(mesh->face_list[faceIndex],endPoint);
    filteredIntersections.push_back(make_tuple(newEndPoint,2,currentSize++));
    // filteredIntersections = mesh->updateIntersectionPts(filteredIntersections);
    return filteredIntersections;
}

vector<tuple<vec3, int, int>> Mesh::dirSort(vector<tuple<vec3, int, int>> intersections, Plane p){
    vec3 direction = get<0>(intersections[0]) - p.origin;
    auto directionSort = [direction, p] (tuple<vec3, int, int> a, tuple<vec3, int, int> b) -> bool
    {
        double x = (get<0>(a) - p.origin).dot(direction);
        double y = (get<0>(b) - p.origin).dot(direction);
        return x <= y;  
    };
    sort(intersections.begin(), intersections.end(), directionSort);
    return intersections;
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

//Checking sanity of the half-edge data structure after every update
bool Mesh::checkSanity(){
    cout << "Checking sanity...\n";
    auto particles = this->mesh->particle_list;
    auto faces = this->mesh->face_list;
    auto edges = this->mesh->edge_list;
    
    //Edge-Edge checks
    for(int i=0;i<edges.size();i++){
        Edge* edge = edges[i];
        Edge* twinEdge = edge->twin;
        if(twinEdge->twin != edge){
            cout << "Twin edge not set properly" << endl;
            return false;
        }
        Edge* nextEdge = edge->next;
        if(nextEdge != NULL){
            if(nextEdge->prev != edge){
                cout << "Next edge not set properly" << endl;
                return false;
            }
        }
        Edge* prevEdge = edge->prev;
        if(prevEdge != NULL){
            if(prevEdge->next != edge){
                cout << "Prev edge not set properly" << endl;
                return false;
            }
        }
    }

    cout << "Edge-Edge checks passed\n";

    //Edge-Face checks
    for(int i=0;i<faces.size();i++){
        Face* face = faces[i];
        Edge* edge = face->edge;
        Edge* nextEdge = edge->next;
        Edge* prevEdge = edge->prev;
        if(nextEdge->face != face || prevEdge->face != face || edge->face != face){
            cout << "Face edge not set properly" << endl;
            return false;
        }
    }

    cout << "Edge-Face checks passed\n";

    // //Vertex-Edge checks
    // for(int i=0;i<edges.size();i++){
    //     Edge* edge = edges[i];
    //     bool found = false;
    //     Particle* particle = edge->startParticle;
    //     Edge* currentEdge = particle->edge;
    //     auto edgeList = particle->getEdges();
    //     for(int i=0;i<edgeList.size();i++){
    //         if(edgeList[i] == edge){
    //             found = true;
    //             break;
    //         }
    //     }
    //     if(!found){
    //         cout << "Vertex edge not set properly: " << particle->position[0] << " " << particle->position[1] << endl;
    //         return false;
    //     }
    // }

    // cout << "Vertex-Edge checks passed\n";

    //Edge-Vertex checks
    for(int i=0;i<particles.size();i++){
        Particle* particle = particles[i];
        auto edgeList = particle->getEdges();
        for(int j=0;j<edgeList.size();j++){
            if(edgeList[j]->startParticle != particle){
                cout << "Edge Vertex not set properly: " << endl;
                cout << particle->position[0] << " " << particle->position[1] << endl;
                cout << edgeList[j]->startParticle->position[0] << " " << edgeList[j]->startParticle->position[1] << endl;
                return false;
            }
        }
    }

    cout << "Edge-Vertex checks passed\n";

    //Vertex-Face checks
    for(int i=0;i<faces.size();i++){
        Particle* p1 = particles[faces[i]->indices[0]];
        Particle* p2 = particles[faces[i]->indices[1]];
        Particle* p3 = particles[faces[i]->indices[2]];
        Edge* edge = faces[i]->edge;
        if(edge->startParticle != p1 && edge->next->startParticle != p2 && edge->prev->startParticle != p3){
            cout << "Face edge not set properly" << endl;
            return false;
        }

    }

    cout << "Vertex-Face checks passed\n";

    cout << "The mesh is correct!" << endl; 
    return true;
}