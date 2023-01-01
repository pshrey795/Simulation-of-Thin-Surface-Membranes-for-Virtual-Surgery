#include "../include/deformable_body.hpp"

//Mesh constructor
DeformableBody::DeformableBody(){                       
    //Using a sample mesh
    upVec = vec3(0,0,1);
    drawRefMesh = false;
    debug = false;

    vector<vec3> vertices;
    vector<unsigned int> indices;
    vector<bool> clamp; 
    unordered_map<int, vec3> constraints;

    int n = 20;
    double f = 8.0 / (double)(n-1);

    for(int i = 0;i < n;i++){
        for(int j = 0;j < n;j++){
            vertices.push_back(vec3(i*f - 4.0,j*f - 4.0,0));
        }
    }

    constraints[0] = vec3(0.0f, 0.0f, 0.0f);
    constraints[n-1] = vec3(0.0f, 0.0f, 0.0f);
    constraints[n*n-n] = vec3(0.0f, 0.0f, 0.0f);
    constraints[n*n-1] = vec3(0.0f, 0.0f, 0.0f);


    for(int i = 0;i < n-1;i++){
        for(int j = 0;j < n-1;j++){
            indices.push_back(i*n+j);
            indices.push_back((i+1)*n + j);
            indices.push_back((i+1)*n + j+1);
            indices.push_back(i*n+j);
            indices.push_back((i+1)*n + j+1);
            indices.push_back(i*n + j + 1);
        }
    }

    this->mesh = new HalfEdge(vertices, indices, constraints); 
    // this->mesh = new HalfEdge(vertices, indices);

    checkSanity();
}

//Mesh Update
void DeformableBody::update(float dt){
    count = (count + 1) % 2;
    if(isPlaying){
        if(count == 0){
            if(currIntersectIdx < intersectPts.size()){
                if(currIntersectIdx == 0){
                    auto newIntPt = make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1);
                    this->mesh->reMesh(newIntPt, intersectPts[currIntersectIdx], intersectPts[currIntersectIdx+1], crossEdgeLeft, crossEdgeRight, normals[currIntersectIdx], splitMode);
                }else if(currIntersectIdx == intersectPts.size()-1){
                    auto newIntPt = make_tuple(vec3(0.0f, 0.0f, 0.0f), -1, -1);
                    this->mesh->reMesh(intersectPts[currIntersectIdx-1], intersectPts[currIntersectIdx], newIntPt, crossEdgeLeft, crossEdgeRight, normals[currIntersectIdx], splitMode);
                }else{
                    this->mesh->reMesh(intersectPts[currIntersectIdx-1], intersectPts[currIntersectIdx], intersectPts[currIntersectIdx+1], crossEdgeLeft, crossEdgeRight, normals[currIntersectIdx], splitMode);
                }
                checkSanity();
                currIntersectIdx++;
            }else{
                // isPlaying = false;
            }
        }
    }
    if(activatePhysics && !isPlaying){
        this->mesh->updateMesh(dt, timeIntegrationType);
    }
}

//Setting up the path for cut/tear
void DeformableBody::setupPath(){
    //Custom path
    currentPath = new Path();
    vector<vec3> inputPts;
    if(drawMode == 1){
        //Straight Cut
        inputPts.push_back(vec3(-3.5, -3, 0));
        inputPts.push_back(vec3(-1.5, -1, 0));
        inputPts.push_back(vec3(1.5, 1, 0));
        inputPts.push_back(vec3(2.5, 3, 0));
    }else if(drawMode == 2){
        //Curved Cut
        inputPts.push_back(vec3(-3.226, -1.226, 0));
        inputPts.push_back(vec3(-1.47, 2.435, 0));
        inputPts.push_back(vec3(1.47, 2.435, 0));
        inputPts.push_back(vec3(3.226, -1.226, 0));
    }
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

void DeformableBody::setupCut(){
    //Define a sample plane
    Plane p(vec3(-4.0f, 6.0f, 0.0f), vec3(1.0f, 1.0f, 0.0f));

    //Get the intersection points
    this->intersectPts = dirSort(this->mesh->Intersect(p), p);
    removeDuplicates(this->intersectPts);

    for(int i=0;i<intersectPts.size();i++){
        normals.push_back(p.normal);
    }

    // vec3 newVertex(3.5f, -1.0f, 0.0f);
    // intersectPts.pop_back();
    // intersectPts.push_back(make_tuple(newVertex, 2, 0));

    // vec3 newVertex1(-3.5f, -1.0f, 0.0f);
    // intersectPts.erase(intersectPts.begin());
    // intersectPts.insert(intersectPts.begin(), make_tuple(newVertex1, 2, 0));

    currIntersectIdx = 0;
}

//Mesh Process Input
void DeformableBody::processInput(Window &window){
    GLFWwindow *win = window.window;
    if(glfwGetKey(win, GLFW_KEY_P) == GLFW_PRESS){
        if(!isPlaying){
            isPlaying = true;
            if(drawMode == 1 || drawMode == 2){
                setupPath();
            }else if(drawMode == 0){
                setupCut();
            }
        }
    }else if(glfwGetKey(win, GLFW_KEY_O) == GLFW_PRESS){
        if(!activatePhysics){
            activatePhysics = true;
        }
    }else if(glfwGetKey(win, GLFW_KEY_L) == GLFW_PRESS){
        if(isPlaying){
            isPlaying = false;
        }
    }
}

void DeformableBody::removeDuplicates(vector<tuple<vec3,int,int>> &vertices){
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

void DeformableBody::removeFacePts(vector<tuple<vec3, int, int>> &vertices){
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
vector<tuple<vec3,int,int>> DeformableBody::filterAndSort(vector<tuple<vec3,int,int>> intersections, vec3 startPoint, vec3 endPoint,bool first){
    vector<tuple<vec3,int,int>> filteredIntersections;
    vec3 direction = endPoint - startPoint;
    double lowerBound = direction.dot(startPoint - startPoint);
    double upperBound = direction.dot(endPoint - startPoint);
    int currentSize = this->mesh->particle_list.size();
    if(first){
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
    filteredIntersections.push_back(make_tuple(endPoint,2,currentSize++));
    // filteredIntersections = mesh->updateIntersectionPts(filteredIntersections);
    return filteredIntersections;
}

vector<tuple<vec3, int, int>> DeformableBody::dirSort(vector<tuple<vec3, int, int>> intersections, Plane p){
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
void DeformableBody::renderMesh(){
    int n = mesh->face_list.size();
    int m = mesh->edge_list.size();

    //Drawing Faces
    for(int i = 0; i < n; i++){
        Face* f = mesh->face_list[i];

        //Drawing the actual mesh
        vec3 v1 = mesh->particle_list[f->indices[0]]->position;
        vec3 v2 = mesh->particle_list[f->indices[1]]->position;
        vec3 v3 = mesh->particle_list[f->indices[2]]->position;
        setColor(vec3(1.0f, 0.0f, 0.0f));
        drawTri(v1,v2,v3);

        if(drawRefMesh){
            //Drawing the reference mesh
            glEnable(GL_BLEND);
            glDepthMask(GL_FALSE);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            vec3 iv1 = mesh->particle_list[f->indices[0]]->initPos;
            vec3 iv2 = mesh->particle_list[f->indices[1]]->initPos;
            vec3 iv3 = mesh->particle_list[f->indices[2]]->initPos;
            setColor(vec4(1.0f, 0.0f, 0.0f, 0.5f));
            drawTri(iv1,iv2,iv3);

            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }
    }

    //Drawing Edges
    for(int i = 0; i < m; i++){
        Edge* e = mesh->edge_list[i];
        if(true){
            setLineWidth(3.0f);
            setColor(vec3(0.0f, 0.0f, 0.0f));

            //Drawing the actual mesh
            vec3 v1 = e->startParticle->position;
            vec3 v2 = e->twin->startParticle->position;
            drawLine(v1,v2);

            if(drawRefMesh){
                //Drawing the reference mesh
                vec3 iv1 = e->startParticle->initPos;
                vec3 iv2 = e->twin->startParticle->initPos;
                drawLine(iv1,iv2);
            }
        }
    }
}

//Checking sanity of the half-edge data structure after every update
bool DeformableBody::checkSanity(){
    if(debug){
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
            if(particle->edge != NULL){
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

        cout << "\n\n\n";
    }
    return true;
}

void DeformableBody::printMeshInfo(){
    for(int i=0;i<this->mesh->particle_list.size();i++){
        Particle* p = this->mesh->particle_list[i];
        cout << p->particleID << endl;
    }
}