#include "../include/half_edge.hpp"

//Constructor of Half Edge data structure from a list of vertices and faces
HalfEdge::HalfEdge(vector<vec3> Vertices, vector<unsigned int> Indices){
    int n = Vertices.size();

    for(unsigned int i=0;i<n;i++){
        vector<int> adj;
        for(unsigned int j=0;j<n;j++){
            adj.push_back(-1);
        }
        this->adjList.push_back(adj); 
    }

    //Adding vertices
    for(unsigned int i=0;i<Vertices.size();i++){
        auto newParticle = new Particle(Vertices[i]);
        newParticle->listIdx = i;
        this->particle_list.push_back(newParticle);
    }

    int i=0;
    while(i < Indices.size()){
        int a,b,c;

        //Particle Indices
        a = Indices[i];
        b = Indices[i+1];
        c = Indices[i+2];

        //Create Face and Half Edges
        struct Face* f = new Face(a,b,c);
        struct Edge* e1 = new Edge(particle_list[a],f);
        struct Edge* e2 = new Edge(particle_list[b],f);
        struct Edge* e3 = new Edge(particle_list[c],f);

        //Linking edges to vertices if not done already 
        if(particle_list[a]->edge == NULL){
            particle_list[a]->edge = e1;
        }
        if(particle_list[b]->edge == NULL){
            particle_list[b]->edge = e2;
        }
        if(particle_list[c]->edge == NULL){
            particle_list[c]->edge = e3;
        }

        //Log twin edge values for later linking
        adjList[a][b] = i;
        adjList[b][c] = i+1;
        adjList[c][a] = i+2;

        //VERY IMPORTANT
        //Need to make sure that the indices are given in anti clockwise order
        //This condition is required for rendering as well as remeshing algorithms

        //Linking next and prev entries
        e1->next = e2;
        e2->next = e3;
        e3->next = e1;
        e1->prev = e3;
        e2->prev = e1;
        e3->prev = e2;

        //Linking face to one of the half edges
        f->edge = e1;
        f->listIdx = this->face_list.size();
        this->face_list.push_back(f);

        //Adding half edges
        e1->listIdx = this->edge_list.size();
        this->edge_list.push_back(e1);
        e2->listIdx = this->edge_list.size();
        this->edge_list.push_back(e2);
        e3->listIdx = this->edge_list.size();
        this->edge_list.push_back(e3);

        //Next iteration
        i+=3;
    }
    int currentSize = this->edge_list.size();

    //Linking twin edges
    for(unsigned int a=0;a<n;a++){
        for(unsigned int b=0;b<n;b++){
            if(adjList[a][b] == -1){
                if(adjList[b][a] != -1){
                    auto v = this->edge_list[adjList[b][a]]->next->startParticle;
                    struct Edge* newEdge = new Edge(v,NULL);
                    newEdge->twin = this->edge_list[adjList[b][a]];
                    this->edge_list[adjList[b][a]]->twin = newEdge;
                    newEdge->listIdx = this->edge_list.size();
                    this->edge_list.push_back(newEdge);
                    adjList[a][b] = currentSize++;
                }
            }else{
                if(adjList[b][a] != -1){
                    edge_list[adjList[a][b]]->twin = edge_list[adjList[b][a]];
                }
            }
        }
    }

    assignInitialState();
    updateGhostSprings();
    redistributeMass();
}

HalfEdge::HalfEdge(vector<vec3> Vertices, vector<unsigned int> Indices, unordered_map<int, vec3> Constraints){
    int n = Vertices.size();

    for(unsigned int i=0;i<n;i++){
        vector<int> adj;
        for(unsigned int j=0;j<n;j++){
            adj.push_back(-1);
        }
        this->adjList.push_back(adj); 
    }

    //Adding vertices
    for(unsigned int i=0;i<Vertices.size();i++){
        auto newParticle = new Particle(Vertices[i]);
        newParticle->listIdx = i;
        this->particle_list.push_back(newParticle);
    }

    //Adding constraints
    for(unsigned int i = 0; i < Vertices.size(); i++){
        if(Constraints.find(i) != Constraints.end()){
            auto c = Constraints.find(i);
            int id = c->first;
            vec3 vel = c->second;
            particle_list[id]->velocity = vel;
            particle_list[id]->constraint = Constraint(vel);
            constraints.insert(id);
        }else{
            particle_list[i]->constraint = Constraint();
        }
    }

    int i=0;
    while(i < Indices.size()){
        int a,b,c;

        //Particle Indices
        a = Indices[i];
        b = Indices[i+1];
        c = Indices[i+2];

        //Create Face and Half Edges
        struct Face* f = new Face(a,b,c);
        struct Edge* e1 = new Edge(particle_list[a],f);
        struct Edge* e2 = new Edge(particle_list[b],f);
        struct Edge* e3 = new Edge(particle_list[c],f);

        //Linking edges to vertices if not done already 
        if(particle_list[a]->edge == NULL){
            particle_list[a]->edge = e1;
        }
        if(particle_list[b]->edge == NULL){
            particle_list[b]->edge = e2;
        }
        if(particle_list[c]->edge == NULL){
            particle_list[c]->edge = e3;
        }

        //Log twin edge values for later linking
        adjList[a][b] = i;
        adjList[b][c] = i+1;
        adjList[c][a] = i+2;

        //VERY IMPORTANT
        //Need to make sure that the indices are given in anti clockwise order
        //This condition is required for rendering as well as remeshing algorithms

        //Linking next and prev entries
        e1->next = e2;
        e2->next = e3;
        e3->next = e1;
        e1->prev = e3;
        e2->prev = e1;
        e3->prev = e2;

        //Linking face to one of the half edges
        f->edge = e1;
        f->listIdx = this->face_list.size();
        this->face_list.push_back(f);

        //Adding half edges
        e1->listIdx = this->edge_list.size();
        this->edge_list.push_back(e1);
        e2->listIdx = this->edge_list.size();
        this->edge_list.push_back(e2);
        e3->listIdx = this->edge_list.size();
        this->edge_list.push_back(e3);

        //Next iteration
        i+=3;
    }
    int currentSize = this->edge_list.size();

    //Linking twin edges
    for(unsigned int a=0;a<n;a++){
        for(unsigned int b=0;b<n;b++){
            if(adjList[a][b] == -1){
                if(adjList[b][a] != -1){
                    auto v = this->edge_list[adjList[b][a]]->next->startParticle;
                    struct Edge* newEdge = new Edge(v,NULL);
                    newEdge->twin = this->edge_list[adjList[b][a]];
                    this->edge_list[adjList[b][a]]->twin = newEdge;
                    newEdge->listIdx = this->edge_list.size();
                    this->edge_list.push_back(newEdge);
                    adjList[a][b] = currentSize++;
                }
            }else{
                if(adjList[b][a] != -1){
                    edge_list[adjList[a][b]]->twin = edge_list[adjList[b][a]];
                }
            }
        }
    }

    assignInitialState();
    updateGhostSprings();
    redistributeMass();
}

void HalfEdge::assignInitialState(){
    // this->particle_list[0]->velocity = vec3(-100.0f,-100.0f,0.0f);
    // this->particle_list[4]->velocity = vec3(-100.0f,100.0f,0.0f);
    // this->particle_list[20]->velocity = vec3(100.0f,-100.0f,0.0f);
    // this->particle_list[24]->velocity = vec3(100.0f,100.0f,0.0f);
}

//Intersection with a plane
vector<tuple<vec3,int,int>> HalfEdge::Intersect(Plane plane){
    ParticleOffsets(plane);
    auto intersectingEdges = IntersectingEdges();
    return IntersectingVertices(intersectingEdges);
}
vector<tuple<vec3, int, int>> HalfEdge::updateIntersectionPts(vector<tuple<vec3, int, int>> intersectionPts){
    vector<tuple<vec3, int, int>> newIntersectionPts;
    for(auto i : intersectionPts){
        int iCode = get<1>(i);
        if(iCode == 0){
            vec3 newPt = getPosAtPoint(particle_list[get<2>(i)]);
            newIntersectionPts.push_back(make_tuple(newPt, iCode, get<2>(i)));
        }else if(iCode == 1){
            vec3 newPt = getPosAtEdge(edge_list[get<2>(i)], get<0>(i));
            newIntersectionPts.push_back(make_tuple(newPt, iCode, get<2>(i)));
        }else{
            newIntersectionPts.push_back(i);
        }
    }
    return newIntersectionPts;
}
void HalfEdge::ParticleOffsets(Plane plane){
    for(unsigned int i=0;i<particle_list.size();i++){
        double newOffset = plane.normal.dot(particle_list[i]->initPos - plane.origin);
        particle_list[i]->offset = newOffset;
    }
}
vector<int> HalfEdge::IntersectingEdges(){
    vector<int> intersectingEdges;
    vector<int> isEdgeIntersecting;
    for(unsigned int i=0;i<this->edge_list.size();i++){
        isEdgeIntersecting.push_back(0);
    }
    for(unsigned int i=0;i<edge_list.size();i++){
        if(isEdgeIntersecting[i] == 0){
            auto v1 = edge_list[i]->startParticle;
            auto v2 = edge_list[i]->twin->startParticle;
            if((v1->offset > 0 && v2->offset < 0) || (v1->offset < 0 && v2->offset > 0)){
                if(abs(v1->offset) > DELTA && abs(v2->offset) > DELTA){
                    intersectingEdges.push_back(i);
                }
                isEdgeIntersecting[i] = 1;
                int j = find(edge_list.begin(),edge_list.end(),edge_list[i]->twin) - edge_list.begin();
                isEdgeIntersecting[j] = 1;
            }
        }
    }
    return intersectingEdges;
}
vector<tuple<vec3,int,int>> HalfEdge::IntersectingVertices(vector<int> edges){
    vector<tuple<vec3,int,int>> intersectingVertices;
    for(unsigned int i=0;i<particle_list.size();i++){
        if(abs(particle_list[i]->offset) < DELTA){
            intersectingVertices.push_back(make_tuple(particle_list[i]->initPos,0,i));
        }
    }
    for(auto idx : edges){
        double offset1 = edge_list[idx]->startParticle->offset;
        double offset2 = edge_list[idx]->twin->startParticle->offset;
        double factor = offset1 / (offset1 - offset2);
        vec3 edgeStart = edge_list[idx]->startParticle->initPos;
        vec3 edgeEnd = edge_list[idx]->twin->startParticle->initPos;
        vec3 newPoint = edgeStart + factor * (edgeEnd - edgeStart);
        intersectingVertices.push_back(make_tuple(newPoint,1,idx));
    }
    return intersectingVertices;
}

bool HalfEdge::isInside(Face* face, vec3 point){
    vec3 a = this->particle_list[face->indices[0]]->initPos;
    vec3 b = this->particle_list[face->indices[1]]->initPos;
    vec3 c = this->particle_list[face->indices[2]]->initPos;
    double A = triArea(a,b,c);
    double A1 = triArea(point,b,c);
    double A2 = triArea(point,c,a);
    double A3 = triArea(point,a,b);
    return (abs((A1 + A2 + A3) - A) < MIN_DIFF);
}

bool HalfEdge::isInsidePos(Face* face, vec3 point){
    vec3 a = this->particle_list[face->indices[0]]->position;
    vec3 b = this->particle_list[face->indices[1]]->position;
    vec3 c = this->particle_list[face->indices[2]]->position;
    double A = triArea(a,b,c);
    double A1 = triArea(point,b,c);
    double A2 = triArea(point,c,a);
    double A3 = triArea(point,a,b);
    return (abs((A1 + A2 + A3) - A) < MIN_DIFF);
}

void HalfEdge::reMesh(tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> intPt, tuple<vec3, int, int> &nextIntPt, Edge* &leftCrossEdge, Edge* &rightCrossEdge, vec3 normal){
    //Auxiliary variables
    int currentType = get<1>(intPt);
    int lastType = get<1>(lastIntPt);
    int nextType = get<1>(nextIntPt);
    bool first = (lastType == -1);
    bool last = (nextType == -1);
    int n = this->particle_list.size();

    //New mesh entities
    Particle* newParticleLeft;
    Particle* newParticleRight;
    int newIndexLeft;
    int newIndexRight;

    //Creating new vertices and edges or using an existing Particle depending on the type of intersection point 
    // vec3 displacement = (1?(splitMode == 0):0) * EPSILON_CUSTOM * normal;
    vec3 displacement = vec3(0.0f, 0.0f, 0.0f);
    if(first){
        if(currentType == 0){
            newParticleLeft = this->particle_list[get<2>(intPt)];
            newIndexLeft = get<2>(intPt);
            vec3 oldPos = newParticleLeft->position;
            vec3 newInitPos = getInitPosAtPoint(newParticleLeft);
            vec3 newVelocity = getVelAtPos(newParticleLeft);
            newParticleLeft->position = oldPos - displacement;
            newParticleRight = new Particle(oldPos + displacement, newInitPos, newVelocity);
            newIndexRight = n;
            newParticleRight->listIdx = newIndexRight;
            this->particle_list.push_back(newParticleRight);
        }else if(currentType == 1){
            vec3 newInitPos = get<0>(intPt);
            vec3 oldPos = getPosAtEdge(this->edge_list[get<2>(intPt)], newInitPos);
            vec3 newVelocity = getVelAtEdge(this->edge_list[get<2>(intPt)], oldPos);
            newParticleLeft = new Particle(oldPos - displacement, newInitPos, newVelocity);
            newParticleRight = new Particle(oldPos + displacement, newInitPos, newVelocity);
            newIndexLeft = n;
            newIndexRight = n+1;
            newParticleLeft->listIdx = newIndexLeft;
            newParticleRight->listIdx = newIndexRight;
            this->particle_list.push_back(newParticleLeft);
            this->particle_list.push_back(newParticleRight);
        }else{
            vec3 newInitPos = get<0>(intPt);
            vec3 oldPos;
            vec3 newVelocity;
            for(unsigned int i=0;i<face_list.size();i++){
                if(isInside(face_list[i], newInitPos)){
                    oldPos = getPosAtFace(face_list[i], newInitPos);
                    newVelocity = getVelAtFace(face_list[i], oldPos);
                    break;
                }
            }
            Particle* newParticle = new Particle(oldPos, newInitPos, newVelocity);
            newParticleLeft = newParticleRight = newParticle;
            newIndexLeft = newIndexRight = n;
            newParticle->listIdx = newIndexLeft;
            this->particle_list.push_back(newParticle);
        }
    }else if(last){
        if(currentType == 0){
            newParticleLeft = this->particle_list[get<2>(intPt)];
            newIndexLeft = get<2>(intPt);
            vec3 oldPos = newParticleLeft->position;
            vec3 newInitPos = getInitPosAtPoint(newParticleLeft);
            vec3 newVelocity = getVelAtPos(newParticleLeft);
            newParticleLeft->position = oldPos - displacement;
            newParticleRight = new Particle(oldPos + displacement, newInitPos, newVelocity);
            newIndexRight = n;
            newParticleRight->listIdx = newIndexRight;
            this->particle_list.push_back(newParticleRight);
        }
    }else{  
        newParticleLeft = this->particle_list[get<2>(intPt)];
        newIndexLeft = get<2>(intPt);
        vec3 oldPos = newParticleLeft->position;
        vec3 newInitPos = getInitPosAtPoint(newParticleLeft);
        vec3 newVelocity = getVelAtPos(newParticleLeft);
        newParticleLeft->position = oldPos - displacement;
        newParticleRight = new Particle(oldPos + displacement, newInitPos, newVelocity);
        newIndexRight = n;
        newParticleRight->listIdx = newIndexRight;
        this->particle_list.push_back(newParticleRight);
    }

    //Complete restructuring of data structure 
    if(first){
        if(currentType == 0){
            if(nextType == 0){
                //Current: Particle, Next: Particle
                Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                int nextIndex = get<2>(nextIntPt);
                auto edgeList = newParticleLeft->getEdges();
                Edge* currentEdge;
                for(unsigned int i=0;i<edgeList.size();i++){
                    if(edgeList[i]->twin->startParticle == nextParticle){
                        currentEdge = edgeList[i];
                        break;
                    }
                }
                Edge* currentTwinEdge = currentEdge->twin;

                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();

                //Edge to Edge relations 
                newEdge1->twin = currentTwinEdge;
                newEdge2->twin = currentEdge;
                currentEdge->twin = newEdge2;
                currentTwinEdge->twin = newEdge1;

                //Edge to Particle relations
                newEdge1->startParticle = newParticleLeft;
                newEdge2->startParticle = nextParticle;
                
                //Face to particle relations 
                Edge* rightEdge = currentEdge;
                Face* rightFace = rightEdge->face;
                while(rightFace != NULL){
                    rightEdge->startParticle = newParticleRight;
                    for(unsigned int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    rightEdge = rightEdge->prev->twin;
                    rightFace = rightEdge->face;
                }
                rightEdge->startParticle = newParticleRight;

                //Particle to Edge relations 
                newParticleLeft->edge = newEdge1;
                newParticleRight->edge = currentEdge;
                nextParticle->edge = currentTwinEdge;

                leftCrossEdge = currentTwinEdge;
                rightCrossEdge = newEdge2;
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
            }else if(nextType == 1){
                //Current: Particle, Next: Edge
                vec3 newInitPos = get<0>(nextIntPt);
                Edge* nextEdge = this->edge_list[get<2>(nextIntPt)];
                vec3 newPos = getPosAtEdge(nextEdge, newInitPos);
                vec3 newVelocity = getVelAtEdge(nextEdge, newPos);
                Particle* nextParticle = new Particle(newPos, newInitPos, newVelocity);
                int nextIndex = this->particle_list.size();
                nextParticle->listIdx = nextIndex;
                this->particle_list.push_back(nextParticle);
                nextIntPt = make_tuple(newPos, 0, nextIndex);

                Edge* currentEdge = nextEdge;
                if(currentEdge->prev == NULL){
                    currentEdge = currentEdge->twin->prev;
                }else{
                    if(currentEdge->prev->startParticle == newParticleLeft){
                        currentEdge = currentEdge->prev;
                    }else{
                        currentEdge = currentEdge->twin->prev;
                    }
                }
                bool hasOppFace = (currentEdge->next->twin->face != NULL);
                Edge* currentPrevEdge = currentEdge->prev;
                Edge* currentNextEdge = currentEdge->next;
                Edge* currentOppEdge;
                Edge* currentOppPrevEdge;
                Edge* currentOppNextEdge;
                if(hasOppFace){
                    currentOppEdge = currentNextEdge->twin->next;
                    currentOppNextEdge = currentOppEdge->next;
                    currentOppPrevEdge = currentOppEdge->prev;
                }

                //Edge to Edge relations 
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();
                Edge* newEdge7;
                Edge* newEdge8;
                if(hasOppFace){
                    newEdge7 = new Edge();
                    newEdge8 = new Edge();
                }

                //Twin Edges
                newEdge1->twin = newEdge2;
                newEdge2->twin = newEdge1;
                newEdge3->twin = newEdge4;
                newEdge4->twin = newEdge3;
                newEdge5->twin = newEdge6;
                newEdge6->twin = newEdge5;
                if(hasOppFace){
                    newEdge7->twin = newEdge8;
                    newEdge8->twin = newEdge7;
                }

                //Next/Prev Edges
                newEdge1->next = currentEdge;
                newEdge1->prev = currentNextEdge;
                currentEdge->prev = newEdge1;
                currentNextEdge->next = newEdge1;
                newEdge4->next = newEdge5;
                newEdge4->prev = currentPrevEdge;
                currentPrevEdge->next = newEdge4;
                currentPrevEdge->prev = newEdge5;
                newEdge5->next = currentPrevEdge;
                newEdge5->prev = newEdge4;
                if(hasOppFace){
                    newEdge7->next = currentOppNextEdge;
                    newEdge7->prev = newEdge6;
                    newEdge6->next = newEdge7;
                    newEdge6->prev = currentOppNextEdge;
                    currentOppNextEdge->next = newEdge6;
                    currentOppNextEdge->prev = newEdge7;
                    newEdge8->next = currentOppPrevEdge;
                    newEdge8->prev = currentOppEdge;
                    currentOppPrevEdge->prev = newEdge8;
                    currentOppEdge->next = newEdge8;
                }

                //Edge to Particle relations 
                newEdge1->startParticle = nextParticle;
                newEdge2->startParticle = newParticleLeft;
                newEdge3->startParticle = nextParticle;
                newEdge5->startParticle = nextParticle;
                newEdge6->startParticle = currentPrevEdge->startParticle;
                currentNextEdge->twin->startParticle = nextParticle;
                Edge* rightEdge = newEdge4;
                while(rightEdge->prev != NULL){
                    rightEdge->startParticle = newParticleRight;
                    rightEdge = rightEdge->prev->twin;
                }
                rightEdge->startParticle = newParticleRight;
                if(hasOppFace){
                    newEdge7->startParticle = nextParticle;
                    newEdge8->startParticle = currentOppNextEdge->startParticle;
                }

                //Particle to Edge relations 
                nextParticle->edge = newEdge1;
                newParticleLeft->edge = newEdge2;
                newParticleRight->edge = newEdge4;

                //Obtaining old particle indices
                int oldIndexLeft, oldIndexRight;
                Face* oldFace = currentEdge->face;
                if(oldFace->edge == currentEdge){
                    oldIndexLeft = oldFace->indices[1];
                    oldIndexRight = oldFace->indices[2];
                }else if(oldFace->edge == currentEdge->next){
                    oldIndexLeft = oldFace->indices[0];
                    oldIndexRight = oldFace->indices[1];
                }else{
                    oldIndexRight = oldFace->indices[0];
                    oldIndexLeft = oldFace->indices[2];
                }
                this->particle_list[oldIndexRight]->edge = currentPrevEdge;

                //Face to particle/edge relations 
                oldFace->setFace(newIndexLeft, oldIndexLeft, nextIndex);
                oldFace->edge = currentEdge;
                Face* newFace = new Face(newIndexRight, nextIndex, oldIndexRight);
                newFace->edge = newEdge4;

                rightEdge = currentPrevEdge->twin;
                Face* rightFace = rightEdge->face;
                while(rightFace != NULL){
                    for(unsigned int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    rightEdge = rightEdge->prev->twin;
                    rightFace = rightEdge->face;
                }

                //Edge to Face relations 
                newEdge1->face = oldFace;
                newEdge4->face = newFace;
                newEdge5->face = newFace;
                currentPrevEdge->face = newFace;

                //Adding new edges/faces
                leftCrossEdge = newEdge1;
                rightCrossEdge = newEdge3;
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newEdge5);
                this->edge_list.push_back(newEdge6);
                this->face_list.push_back(newFace);

                if(hasOppFace){
                    int oldOppIndexLeft, oldOppIndexRight, centreOppIndex;
                    Face* oldOppFace = currentOppEdge->face;
                    if(oldOppFace->edge == currentOppEdge){
                        oldOppIndexLeft = oldOppFace->indices[0];
                        centreOppIndex = oldOppFace->indices[1];
                        oldOppIndexRight = oldOppFace->indices[2];
                    }else if(oldOppFace->edge == currentOppNextEdge){
                        oldOppIndexLeft = oldOppFace->indices[2];
                        centreOppIndex = oldOppFace->indices[0];
                        oldOppIndexRight = oldOppFace->indices[1];
                    }else{
                        oldOppIndexLeft = oldOppFace->indices[1];
                        centreOppIndex = oldOppFace->indices[2];
                        oldOppIndexRight = oldOppFace->indices[0];
                    }

                    oldOppFace->setFace(oldOppIndexLeft, centreOppIndex, nextIndex);
                    oldOppFace->edge = currentOppEdge;
                    Face* newOppFace = new Face(nextIndex, centreOppIndex, oldOppIndexRight);
                    newOppFace->edge = newEdge7;

                    newEdge7->face = newOppFace;
                    newEdge8->face = oldOppFace;
                    newEdge6->face = newOppFace;
                    currentOppNextEdge->face = newOppFace;

                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newEdge8);
                    this->face_list.push_back(newOppFace);
                }
            }else{
                //Current: Particle, Next: Face
            }
        }else if(currentType == 1){
            Edge* intersectingEdge = this->edge_list[get<2>(intPt)];
            if(intersectingEdge->face == NULL){
                intersectingEdge = intersectingEdge->twin;
            }
            Face* intersectingFace = intersectingEdge->face;

            int oldIndexLeft, oldIndexRight, oppIndex;
            Edge* currentEdge = intersectingFace->edge;
            int i = 0;
            while(currentEdge != intersectingEdge){
                i++;
                currentEdge = currentEdge->next;
            }
            oldIndexRight = intersectingFace->indices[i%3];
            oldIndexLeft = intersectingFace->indices[(i+1)%3];
            oppIndex = intersectingFace->indices[(i+2)%3];

            if(nextType == 0){
                //Current: Edge, Next: Particle
                Edge* currentPrevEdge = currentEdge->prev;
                Edge* currentNextEdge = currentEdge->next;

                //Edge to Edge relations
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();

                //Twin edges
                newEdge1->twin = newEdge2;
                newEdge2->twin = newEdge1;
                newEdge3->twin = newEdge4;
                newEdge4->twin = newEdge3;
                newEdge5->twin = newEdge6;
                newEdge6->twin = newEdge5;

                //Next/Prev Edges
                newEdge1->next = currentEdge;
                newEdge1->prev = currentNextEdge;
                currentEdge->prev = newEdge1;
                currentNextEdge->next = newEdge1;
                newEdge4->next = currentPrevEdge;
                newEdge4->prev = newEdge5;
                currentPrevEdge->prev = newEdge4;
                currentPrevEdge->next = newEdge5;
                newEdge5->prev = currentPrevEdge;
                newEdge5->next = newEdge4;

                //Edge to particle relations
                newEdge1->startParticle = this->particle_list[oppIndex];
                newEdge2->startParticle = newParticleLeft;
                newEdge3->startParticle = this->particle_list[oppIndex];
                newEdge4->startParticle = newParticleRight;
                newEdge5->startParticle = this->particle_list[oldIndexRight];
                newEdge6->startParticle = newParticleRight;
                currentEdge->startParticle = newParticleLeft;

                //Particle to edge relations 
                newParticleLeft->edge = newEdge2;
                newParticleRight->edge = newEdge4;
                this->particle_list[oppIndex]->edge = newEdge1;
                this->particle_list[oldIndexRight]->edge = newEdge5;

                //Face to particle/edge relations
                intersectingFace->setFace(newIndexLeft, oldIndexLeft, oppIndex);
                intersectingFace->edge = currentEdge;
                Face* newFace = new Face(newIndexRight, oppIndex, oldIndexRight);
                newFace->edge = newEdge4;

                //Edge to Face relations
                newEdge1->face = intersectingFace;
                newEdge4->face = newFace;
                newEdge5->face = newFace;
                currentPrevEdge->face = newFace;

                //Adding new edges/faces
                leftCrossEdge = newEdge1;
                rightCrossEdge = newEdge3;
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newEdge5);
                this->edge_list.push_back(newEdge6);
                this->face_list.push_back(newFace);
            }else if(nextType == 1){
                //Current: Edge, Next: Edge
                Edge* nextIntersectingEdge = this->edge_list[get<2>(nextIntPt)];
                if(nextIntersectingEdge->face != intersectingFace){
                    nextIntersectingEdge = nextIntersectingEdge->twin;
                }

                vec3 newInitPos = get<0>(nextIntPt);
                Edge* nextEdge = this->edge_list[get<2>(nextIntPt)];
                vec3 newPos = getPosAtEdge(nextEdge, newInitPos);
                vec3 newVelocity = getVelAtEdge(nextEdge, newPos);
                Particle* nextParticle = new Particle(newPos, newInitPos, newVelocity);
                int nextIndex = this->particle_list.size();
                nextParticle->listIdx = nextIndex;
                this->particle_list.push_back(nextParticle);
                nextIntPt = make_tuple(newPos, 0, nextIndex);

                //Edge to Edge relations
                Edge* currentPrevEdge = currentEdge->prev;
                Edge* currentNextEdge = currentEdge->next;
                Edge* currentOppEdge;
                Edge* currentOppNextEdge;
                Edge* currentOppPrevEdge;

                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();
                Edge* newEdge7 = new Edge();
                Edge* newEdge8 = new Edge();
                Edge* newEdge9 = new Edge();
                Edge* newEdge10 = new Edge();
                Edge* newEdge11;
                Edge* newEdge12;

                //Two cases, in which the split is along the left or the right side of the triangle
                bool left = (nextIntersectingEdge == currentNextEdge);

                if(left){
                    bool hasOppFace = (currentNextEdge->twin->face != NULL);

                    if(hasOppFace){
                        newEdge11 = new Edge();
                        newEdge12 = new Edge();
                    }

                    if(hasOppFace){
                        currentOppEdge = currentNextEdge->twin->next;
                        currentOppNextEdge = currentOppEdge->next;
                        currentOppPrevEdge = currentOppEdge->prev;
                    }

                    //Twin edges
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newEdge4;
                    newEdge4->twin = newEdge3;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;
                    newEdge7->twin = newEdge8;
                    newEdge8->twin = newEdge7;
                    newEdge9->twin = newEdge10;
                    newEdge10->twin = newEdge9;
                    if(hasOppFace){
                        newEdge11->twin = newEdge12;
                        newEdge12->twin = newEdge11;
                    }

                    //Next/Prev Edges
                    newEdge1->next = currentEdge;
                    newEdge1->prev = currentNextEdge;
                    currentEdge->prev = newEdge1;
                    currentNextEdge->next = newEdge1;
                    newEdge4->next = newEdge7;
                    newEdge4->prev = newEdge5;
                    newEdge7->next = newEdge5;
                    newEdge7->prev = newEdge4;
                    newEdge5->prev = newEdge7;
                    newEdge5->next = newEdge4;
                    newEdge8->next = newEdge9;
                    newEdge8->prev = currentPrevEdge;
                    newEdge9->next = currentPrevEdge;
                    newEdge9->prev = newEdge8;
                    currentPrevEdge->prev = newEdge9;
                    currentPrevEdge->next = newEdge8;
                    if(hasOppFace){
                        newEdge11->next = currentOppNextEdge;
                        newEdge11->prev = newEdge10;
                        newEdge10->next = newEdge11;
                        newEdge10->prev = currentOppNextEdge;
                        currentOppNextEdge->next = newEdge10;
                        currentOppNextEdge->prev = newEdge11;
                        newEdge12->next = currentOppPrevEdge;
                        newEdge12->prev = currentOppEdge;
                        currentOppPrevEdge->prev = newEdge12;
                        currentOppEdge->next = newEdge12;
                    }

                    //Edge to particle relations
                    currentEdge->startParticle = newParticleLeft;
                    currentNextEdge->twin->startParticle = nextParticle;
                    newEdge1->startParticle = nextParticle;
                    newEdge2->startParticle = newParticleLeft;
                    newEdge3->startParticle = nextParticle;
                    newEdge4->startParticle = newParticleRight;
                    newEdge5->startParticle = this->particle_list[oldIndexRight];
                    newEdge6->startParticle = newParticleRight;
                    newEdge7->startParticle = nextParticle;
                    newEdge8->startParticle = this->particle_list[oldIndexRight];
                    newEdge9->startParticle = nextParticle;
                    newEdge10->startParticle = this->particle_list[oppIndex];
                    if(hasOppFace){
                        newEdge11->startParticle = nextParticle;
                        newEdge12->startParticle = currentOppNextEdge->startParticle;
                    }

                    //Particle to edge relations
                    nextParticle->edge = newEdge1;
                    newParticleLeft->edge = newEdge2;
                    newParticleRight->edge = newEdge4;
                    this->particle_list[oppIndex]->edge = newEdge10;
                    this->particle_list[oldIndexRight]->edge = newEdge5;

                    //Face to particle/edge relations
                    intersectingFace->setFace(newIndexLeft, oldIndexLeft, nextIndex);
                    intersectingFace->edge = currentEdge;
                    Face* newFaceTop = new Face(newIndexRight, nextIndex, oldIndexRight);
                    newFaceTop->edge = newEdge4;
                    Face* newFaceBot = new Face(oldIndexRight, nextIndex, oppIndex);
                    newFaceBot->edge = newEdge8;

                    //Edge to Face relations
                    newEdge1->face = intersectingFace;
                    newEdge4->face = newFaceTop;
                    newEdge5->face = newFaceTop;
                    newEdge7->face = newFaceTop;
                    newEdge8->face = newFaceBot;
                    newEdge9->face = newFaceBot;
                    currentPrevEdge->face = newFaceBot;

                    //Adding the edges/faces
                    leftCrossEdge = newEdge1;
                    rightCrossEdge = newEdge3;
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newEdge8);
                    this->edge_list.push_back(newEdge9);
                    this->edge_list.push_back(newEdge10);
                    this->face_list.push_back(newFaceTop);
                    this->face_list.push_back(newFaceBot);

                    if(hasOppFace){
                        int oldOppIndexLeft, oldOppIndexRight, centreOppIndex;
                        Face* oldOppFace = currentOppEdge->face;
                        if(oldOppFace->edge == currentOppEdge){
                            oldOppIndexLeft = oldOppFace->indices[0];
                            centreOppIndex = oldOppFace->indices[1];
                            oldOppIndexRight = oldOppFace->indices[2];
                        }else if(oldOppFace->edge == currentOppNextEdge){
                            oldOppIndexLeft = oldOppFace->indices[2];
                            centreOppIndex = oldOppFace->indices[0];
                            oldOppIndexRight = oldOppFace->indices[1];
                        }else{
                            oldOppIndexLeft = oldOppFace->indices[1];
                            centreOppIndex = oldOppFace->indices[2];
                            oldOppIndexRight = oldOppFace->indices[0];
                        }

                        oldOppFace->setFace(oldOppIndexLeft, centreOppIndex, nextIndex);
                        oldOppFace->edge = currentOppEdge;
                        Face* newOppFace = new Face(nextIndex, centreOppIndex, oldOppIndexRight);
                        newOppFace->edge = newEdge11;

                        newEdge11->face = newOppFace;
                        newEdge12->face = oldOppFace;
                        newEdge10->face = newOppFace;
                        currentOppNextEdge->face = newOppFace;

                        this->edge_list.push_back(newEdge11);
                        this->edge_list.push_back(newEdge12);
                        this->face_list.push_back(newOppFace);
                    }
                }else{
                    bool hasOppFace = currentPrevEdge->twin->face != NULL;

                    if(hasOppFace){
                        newEdge11 = new Edge();
                        newEdge12 = new Edge();
                    }

                    if(hasOppFace){
                        currentOppEdge = currentPrevEdge->twin->prev;
                        currentOppNextEdge = currentOppEdge->next;
                        currentOppPrevEdge = currentOppEdge->prev;
                    }

                    //Twin edges
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newEdge4;
                    newEdge4->twin = newEdge3;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;
                    newEdge7->twin = newEdge8;
                    newEdge8->twin = newEdge7;
                    newEdge9->twin = newEdge10;
                    newEdge10->twin = newEdge9;
                    if(hasOppFace){
                        newEdge11->twin = newEdge12;
                        newEdge12->twin = newEdge11;
                    }

                    //Next/Prev Edges
                    newEdge1->next = currentPrevEdge;
                    newEdge1->prev = currentEdge;
                    currentEdge->next = newEdge1;
                    currentPrevEdge->prev = newEdge1;
                    newEdge4->next = newEdge5;
                    newEdge4->prev = newEdge7;
                    newEdge7->next = newEdge4;
                    newEdge7->prev = newEdge5;
                    newEdge5->prev = newEdge4;
                    newEdge5->next = newEdge7;
                    newEdge8->next = currentNextEdge;
                    newEdge8->prev = newEdge9;
                    newEdge9->next = newEdge8;
                    newEdge9->prev = currentNextEdge;
                    currentNextEdge->prev = newEdge8;
                    currentNextEdge->next = newEdge9;
                    if(hasOppFace){
                        newEdge11->next = newEdge10;
                        newEdge11->prev = currentOppPrevEdge;
                        newEdge10->next = currentOppPrevEdge;
                        newEdge10->prev = newEdge11;
                        currentOppPrevEdge->next = newEdge11;
                        currentOppPrevEdge->prev = newEdge10;
                        newEdge12->next = currentOppEdge;
                        newEdge12->prev = currentOppNextEdge;
                        currentOppNextEdge->next = newEdge12;
                        currentOppEdge->prev = newEdge12;
                    }

                    //Edge to particle relations
                    currentEdge->twin->startParticle = newParticleRight;
                    currentPrevEdge->startParticle = nextParticle;
                    newEdge1->startParticle = newParticleRight;
                    newEdge2->startParticle = nextParticle;
                    newEdge3->startParticle = newParticleLeft;
                    newEdge4->startParticle = nextParticle;
                    newEdge5->startParticle = newParticleLeft;
                    newEdge6->startParticle = this->particle_list[oldIndexLeft];
                    newEdge7->startParticle = this->particle_list[oldIndexLeft];
                    newEdge8->startParticle = nextParticle;
                    newEdge9->startParticle = this->particle_list[oppIndex];
                    newEdge10->startParticle = nextParticle;
                    if(hasOppFace){
                        newEdge11->startParticle = currentOppEdge->startParticle;
                        newEdge12->startParticle = nextParticle;
                    }

                    //Particle to edge relations
                    nextParticle->edge = newEdge4;
                    newParticleLeft->edge = newEdge3;
                    newParticleRight->edge = newEdge1;
                    this->particle_list[oppIndex]->edge = newEdge9;
                    this->particle_list[oldIndexLeft]->edge = newEdge6;

                    //Face to particle/edge relations
                    intersectingFace->setFace(oldIndexRight, newIndexRight, nextIndex);
                    intersectingFace->edge = currentEdge;
                    Face* newFaceTop = new Face(nextIndex, newIndexLeft, oldIndexLeft);
                    newFaceTop->edge = newEdge4;
                    Face* newFaceBot = new Face(nextIndex, oldIndexLeft, oppIndex);
                    newFaceBot->edge = newEdge8;

                    //Edge to Face relations
                    newEdge1->face = intersectingFace;
                    newEdge4->face = newFaceTop;
                    newEdge5->face = newFaceTop;
                    newEdge7->face = newFaceTop;
                    newEdge8->face = newFaceBot;
                    newEdge9->face = newFaceBot;
                    currentNextEdge->face = newFaceBot;

                    //Adding the edges/faces
                    leftCrossEdge = newEdge4;
                    rightCrossEdge = newEdge2;
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newEdge8);
                    this->edge_list.push_back(newEdge9);
                    this->edge_list.push_back(newEdge10);
                    this->face_list.push_back(newFaceTop);
                    this->face_list.push_back(newFaceBot);

                    if(hasOppFace){
                        int oldOppIndexLeft, oldOppIndexRight, centreOppIndex;
                        Face* oldOppFace = currentOppEdge->face;
                        if(oldOppFace->edge == currentOppEdge){
                            oldOppIndexLeft = oldOppFace->indices[2];
                            centreOppIndex = oldOppFace->indices[0];
                            oldOppIndexRight = oldOppFace->indices[1];
                        }else if(oldOppFace->edge == currentOppNextEdge){
                            oldOppIndexLeft = oldOppFace->indices[1];
                            centreOppIndex = oldOppFace->indices[2];
                            oldOppIndexRight = oldOppFace->indices[0];
                        }else{
                            oldOppIndexLeft = oldOppFace->indices[0];
                            centreOppIndex = oldOppFace->indices[1];
                            oldOppIndexRight = oldOppFace->indices[2];
                        }

                        oldOppFace->setFace(centreOppIndex, oldOppIndexRight, nextIndex);
                        oldOppFace->edge = currentOppEdge;
                        Face* newOppFace = new Face(centreOppIndex, nextIndex, oldOppIndexLeft);
                        newOppFace->edge = newEdge11;

                        newEdge11->face = newOppFace;
                        newEdge12->face = oldOppFace;
                        newEdge10->face = newOppFace;
                        currentOppPrevEdge->face = newOppFace;

                        this->edge_list.push_back(newEdge11);
                        this->edge_list.push_back(newEdge12);
                        this->face_list.push_back(newOppFace);
                    }
                }
            }else{
                //Current: Edge, Next: Face
            }
        }else{
            if(nextType == 0){
                //Current: Face, Next: Particle
                Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                int nextIndex = get<2>(nextIntPt);
                auto edgeList = nextParticle->getEdges();
                Edge* currentEdge;
                for(unsigned int i=0;i<edgeList.size();i++){
                    if(isInside(edgeList[i]->face, newParticleLeft->initPos)){
                        currentEdge = edgeList[i];
                        break;
                    }
                }

                //Obtaining old indices
                int oldIndexLeft, oldIndexRight;
                Face* oldFace = currentEdge->face;
                if(oldFace->edge == currentEdge){
                    oldIndexLeft = oldFace->indices[2];
                    oldIndexRight = oldFace->indices[1];
                }else if(oldFace->edge == currentEdge->next){
                    oldIndexLeft = oldFace->indices[1];
                    oldIndexRight = oldFace->indices[0];
                }else{
                    oldIndexLeft = oldFace->indices[0];
                    oldIndexRight = oldFace->indices[2];
                }

                //Edge to edge relations 
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();
                Edge* currentPrevEdge = currentEdge->prev;
                Edge* currentNextEdge = currentEdge->next;

                //Twin edges
                newEdge1->twin = newEdge2;
                newEdge2->twin = newEdge1;
                newEdge3->twin = newEdge4;
                newEdge4->twin = newEdge3;
                newEdge5->twin = newEdge6;
                newEdge6->twin = newEdge5;

                //Next/Prev edges
                newEdge1->next = newEdge6;
                newEdge1->prev = currentEdge;
                newEdge6->next = currentEdge;
                newEdge6->prev = newEdge1;
                currentEdge->next = newEdge1;
                currentEdge->prev = newEdge6;
                newEdge2->next = currentNextEdge;
                newEdge2->prev = newEdge3;
                currentNextEdge->next = newEdge3;
                currentNextEdge->prev = newEdge2;
                newEdge3->next = newEdge2;
                newEdge3->prev = currentNextEdge;
                newEdge4->next = currentPrevEdge;
                newEdge4->prev = newEdge5;
                currentPrevEdge->next = newEdge5;
                currentPrevEdge->prev = newEdge4;
                newEdge5->next = newEdge4;
                newEdge5->prev = currentPrevEdge;

                //Edge to particle relations
                newEdge1->startParticle = this->particle_list[oldIndexRight];
                newEdge2->startParticle = newParticleLeft;
                newEdge3->startParticle = this->particle_list[oldIndexLeft];
                newEdge4->startParticle = newParticleLeft;
                newEdge5->startParticle = nextParticle;
                newEdge6->startParticle = newParticleLeft;

                //Particle to edge relations 
                newParticleLeft->edge = newEdge4;

                //Face to particle/edge relations 
                oldFace->setFace(nextIndex, oldIndexRight, newIndexLeft);
                oldFace->edge = currentEdge;
                Face* newFaceTop = new Face(oldIndexRight, oldIndexLeft, newIndexLeft);
                newFaceTop->edge = currentNextEdge;
                Face* newFaceBot = new Face(oldIndexLeft, nextIndex, newIndexLeft);
                newFaceBot->edge = currentPrevEdge;

                //Edge to face relations
                newEdge1->face = oldFace;
                newEdge6->face = oldFace;
                newEdge2->face = newFaceTop;
                newEdge3->face = newFaceTop;
                currentNextEdge->face = newFaceTop;
                newEdge4->face = newFaceBot;
                newEdge5->face = newFaceBot;
                currentPrevEdge->face = newFaceBot;

                //Adding new edges and faces to lists
                leftCrossEdge = newEdge5; 
                rightCrossEdge = NULL;
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newEdge5);
                this->edge_list.push_back(newEdge6);
                this->face_list.push_back(newFaceBot);
                this->face_list.push_back(newFaceTop);
            }else if(nextType == 1){
                //Current: Face, Next: Edge
                vec3 newInitPos = get<0>(nextIntPt);
                Edge* nextEdge = this->edge_list[get<2>(nextIntPt)];
                vec3 newPos = getPosAtEdge(nextEdge, newInitPos);
                vec3 newVelocity = getVelAtEdge(nextEdge, newPos);
                Particle* nextParticle = new Particle(newPos, newInitPos, newVelocity);
                int nextIndex = this->particle_list.size();
                nextParticle->listIdx = nextIndex;
                this->particle_list.push_back(nextParticle);
                nextIntPt = make_tuple(newPos, 0, nextIndex);

                Edge* currentEdge = nextEdge;
                if(currentEdge->face == NULL){
                    currentEdge = currentEdge->twin->prev;
                }else{
                    if(isInside(currentEdge->face, newParticleLeft->initPos)){
                        currentEdge = currentEdge->prev;
                    }else{
                        currentEdge = currentEdge->twin->prev;
                    }
                }
                bool hasOppFace = (currentEdge->next->twin->face != NULL);
                Edge* currentPrevEdge = currentEdge->prev;
                Edge* currentNextEdge = currentEdge->next;
                Edge* currentOppEdge;
                Edge* currentOppPrevEdge;
                Edge* currentOppNextEdge;
                if(hasOppFace){
                    currentOppEdge = currentNextEdge->twin->next;
                    currentOppNextEdge = currentOppEdge->next;
                    currentOppPrevEdge = currentOppEdge->prev;
                }

                //Obtaining old indices
                int oldIndexRight, oldIndexLeft, oppIndex;
                Face* oldFace = currentEdge->face;
                if(oldFace->edge == currentEdge){
                    oppIndex = oldFace->indices[0];
                    oldIndexLeft = oldFace->indices[1];
                    oldIndexRight = oldFace->indices[2];
                }else if(oldFace->edge == currentEdge->next){
                    oppIndex = oldFace->indices[2];
                    oldIndexLeft = oldFace->indices[0];
                    oldIndexRight = oldFace->indices[1];
                }else{
                    oppIndex = oldFace->indices[1];
                    oldIndexLeft = oldFace->indices[2];
                    oldIndexRight = oldFace->indices[0];
                }

                //Edge to edge relations 
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();
                Edge* newEdge7 = new Edge();
                Edge* newEdge8 = new Edge();
                Edge* newEdge9 = new Edge();
                Edge* newEdge10 = new Edge();
                Edge* newEdge11;
                Edge* newEdge12;
                if(hasOppFace){
                    newEdge11 = new Edge();
                    newEdge12 = new Edge();
                }

                //Twin Edges
                newEdge1->twin = newEdge2;
                newEdge2->twin = newEdge1;
                newEdge3->twin = newEdge4;
                newEdge4->twin = newEdge3;
                newEdge5->twin = newEdge6;
                newEdge6->twin = newEdge5;
                newEdge7->twin = newEdge8;
                newEdge8->twin = newEdge7;
                newEdge9->twin = newEdge10;
                newEdge10->twin = newEdge9;
                if(hasOppFace){
                    newEdge11->twin = newEdge12;
                    newEdge12->twin = newEdge11;
                }

                //Next/Prev Edges
                newEdge1->next = currentNextEdge;
                newEdge1->prev = newEdge8;
                newEdge8->next = newEdge1;
                newEdge8->prev = currentNextEdge;
                currentNextEdge->next = newEdge8;
                currentNextEdge->prev = newEdge1;
                newEdge2->prev = currentEdge;
                newEdge2->next = newEdge3;
                newEdge3->prev = newEdge2;
                newEdge3->next = currentEdge;
                currentEdge->next = newEdge2;
                currentEdge->prev = newEdge3;
                newEdge4->prev = currentPrevEdge;
                newEdge4->next = newEdge5;
                newEdge5->prev = newEdge4;
                newEdge5->next = currentPrevEdge;
                currentPrevEdge->next = newEdge4;
                currentPrevEdge->prev = newEdge5;
                newEdge6->next = newEdge7;
                newEdge6->prev = newEdge9;
                newEdge7->next = newEdge9;
                newEdge7->prev = newEdge6;
                newEdge9->next = newEdge6;
                newEdge9->prev = newEdge7;
                if(hasOppFace){
                    newEdge11->next = currentOppNextEdge;
                    newEdge11->prev = newEdge10;
                    newEdge10->next = newEdge11;
                    newEdge10->prev = currentOppNextEdge;
                    currentOppNextEdge->next = newEdge10;
                    currentOppNextEdge->prev = newEdge11;
                    newEdge12->next = currentOppPrevEdge;
                    newEdge12->prev = currentOppEdge;
                    currentOppPrevEdge->prev = newEdge12;
                    currentOppEdge->next = newEdge12;
                }

                //Edge to particle relations 
                currentNextEdge->twin->startParticle = nextParticle;
                newEdge1->startParticle = newParticleLeft;
                newEdge2->startParticle = this->particle_list[oldIndexLeft];
                newEdge3->startParticle = newParticleLeft;
                newEdge4->startParticle = this->particle_list[oppIndex];
                newEdge5->startParticle = newParticleLeft;
                newEdge6->startParticle = this->particle_list[oldIndexRight];
                newEdge7->startParticle = newParticleLeft;
                newEdge8->startParticle = nextParticle;
                newEdge9->startParticle = nextParticle;
                newEdge10->startParticle = this->particle_list[oldIndexRight];
                if(hasOppFace){
                    newEdge11->startParticle = nextParticle;
                    newEdge12->startParticle = currentOppNextEdge->startParticle;
                }

                //Particle to edge relations 
                newParticleLeft->edge = newEdge3;
                nextParticle->edge = newEdge8;
                this->particle_list[oldIndexRight]->edge = newEdge6;

                //Face to particle/edge relations 
                oldFace->setFace(oppIndex, oldIndexLeft, newIndexLeft);
                oldFace->edge = currentEdge;
                Face* newFaceBotLeft = new Face(oldIndexLeft, nextIndex, newIndexLeft);
                newFaceBotLeft->edge = currentNextEdge;
                Face* newFaceBotRight = new Face(nextIndex, oldIndexRight, newIndexLeft);
                newFaceBotRight->edge = newEdge9;
                Face* newFaceTop = new Face(oldIndexRight, oppIndex, newIndexLeft);
                newFaceTop->edge = currentPrevEdge;

                //Edge to face relations 
                newEdge2->face = oldFace;
                newEdge3->face = oldFace;
                newEdge1->face = newFaceBotLeft;
                newEdge8->face = newFaceBotLeft;
                currentNextEdge->face = newFaceBotLeft;
                newEdge6->face = newFaceBotRight;
                newEdge7->face = newFaceBotRight;
                newEdge9->face = newFaceBotRight;
                newEdge4->face = newFaceTop;
                newEdge5->face = newFaceTop;
                currentPrevEdge->face = newFaceTop;

                //Adding the edges/faces
                leftCrossEdge = newEdge8;
                rightCrossEdge = NULL;
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newEdge5);
                this->edge_list.push_back(newEdge6);
                this->edge_list.push_back(newEdge7);
                this->edge_list.push_back(newEdge8);
                this->edge_list.push_back(newEdge9);
                this->edge_list.push_back(newEdge10);
                this->face_list.push_back(newFaceTop);
                this->face_list.push_back(newFaceBotLeft);
                this->face_list.push_back(newFaceBotRight);

                if(hasOppFace){
                    int oldOppIndexLeft, oldOppIndexRight, centreOppIndex;
                    Face* oldOppFace = currentOppEdge->face;
                    if(oldOppFace->edge == currentOppEdge){
                        oldOppIndexLeft = oldOppFace->indices[0];
                        centreOppIndex = oldOppFace->indices[1];
                        oldOppIndexRight = oldOppFace->indices[2];
                    }else if(oldOppFace->edge == currentOppNextEdge){
                        oldOppIndexLeft = oldOppFace->indices[2];
                        centreOppIndex = oldOppFace->indices[0];
                        oldOppIndexRight = oldOppFace->indices[1];
                    }else{
                        oldOppIndexLeft = oldOppFace->indices[1];
                        centreOppIndex = oldOppFace->indices[2];
                        oldOppIndexRight = oldOppFace->indices[0];
                    }

                    oldOppFace->setFace(oldOppIndexLeft, centreOppIndex, nextIndex);
                    oldOppFace->edge = currentOppEdge;
                    Face* newOppFace = new Face(nextIndex, centreOppIndex, oldOppIndexRight);
                    newOppFace->edge = newEdge11;

                    newEdge11->face = newOppFace;
                    newEdge12->face = oldOppFace;
                    newEdge10->face = newOppFace;
                    currentOppNextEdge->face = newOppFace;

                    this->edge_list.push_back(newEdge11);
                    this->edge_list.push_back(newEdge12);
                    this->face_list.push_back(newOppFace);
                }
            }else{
                //Current: Face, Next: Face
            }
        }
    }else if(last){
        if(currentType == 0){
            rightCrossEdge->startParticle = newParticleRight;
            Edge* currentEdge = rightCrossEdge->twin->next;
            while(currentEdge != NULL){
                currentEdge->startParticle = newParticleRight;
                Face* rightFace = currentEdge->face;
                for(unsigned int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                        break;
                    }
                }
                currentEdge = currentEdge->twin->next;
            }
            newParticleRight->edge = rightCrossEdge;
        }else{
            //Do nothing, everything has already been handled
        }
    }else{
        if(lastType == 2){
            Edge* lastTwin = leftCrossEdge->twin;
            Particle* lastParticle = lastTwin->startParticle;
            Edge* edge1 = new Edge();
            edge1->startParticle = lastParticle;
            edge1->twin = leftCrossEdge;
            leftCrossEdge->twin = edge1;

            rightCrossEdge = new Edge();
            rightCrossEdge->twin = lastTwin;
            lastTwin->twin = rightCrossEdge;
            rightCrossEdge->startParticle = newParticleRight;

            edge_list.push_back(edge1);
            edge_list.push_back(rightCrossEdge);
        }
        if(nextType == 0){
            //Current: Particle, Next: Particle
            Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
            int nextIndex = get<2>(nextIntPt);
            auto edgeList = newParticleLeft->getEdges();
            Edge* currentEdge;
            for(unsigned int i=0;i<edgeList.size();i++){
                if(edgeList[i]->twin->startParticle == nextParticle){
                    currentEdge = edgeList[i];
                    break;
                }
            }
            Edge* currentTwinEdge = currentEdge->twin;

            Edge* newEdge1 = new Edge();
            Edge* newEdge2 = new Edge();

            //Edge to Edge relations 
            newEdge1->twin = currentTwinEdge;
            newEdge2->twin = currentEdge;
            currentEdge->twin = newEdge2;
            currentTwinEdge->twin = newEdge1;

            //Edge to Particle relations
            newEdge1->startParticle = newParticleLeft;
            newEdge2->startParticle = nextParticle;
            
            //Face to particle relations 
            Edge* rightEdge = currentEdge;
            Face* rightFace = rightEdge->face;
            while(rightEdge != rightCrossEdge){
                rightEdge->startParticle = newParticleRight;
                for(unsigned int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                        break;
                    }
                }
                rightEdge = rightEdge->prev->twin;
                rightFace = rightEdge->face;
            }
            rightEdge->startParticle = newParticleRight;

            //Particle to Edge relations 
            newParticleLeft->edge = newEdge1;
            newParticleRight->edge = currentEdge;
            nextParticle->edge = currentTwinEdge;

            leftCrossEdge = currentTwinEdge;
            rightCrossEdge = newEdge2;
            this->edge_list.push_back(newEdge1);
            this->edge_list.push_back(newEdge2);
        }else if(nextType == 1){
            //Current: Particle, Next: Edge
            vec3 newInitPos = get<0>(nextIntPt);
            Edge* nextEdge = this->edge_list[get<2>(nextIntPt)];
            vec3 newPos = getPosAtEdge(nextEdge, newInitPos);
            vec3 newVelocity = getVelAtEdge(nextEdge, newPos);
            Particle* nextParticle = new Particle(newPos, newInitPos, newVelocity);
            int nextIndex = this->particle_list.size();
            nextParticle->listIdx = nextIndex;
            this->particle_list.push_back(nextParticle);
            nextIntPt = make_tuple(newPos, 0, nextIndex);

            Edge* currentEdge = nextEdge;
            if(currentEdge->prev == NULL){
                currentEdge = currentEdge->twin->prev;
            }else{
                if(currentEdge->prev->startParticle == newParticleLeft){
                    currentEdge = currentEdge->prev;
                }else{
                    currentEdge = currentEdge->twin->prev;
                }
            }
            bool hasOppFace = (currentEdge->next->twin->face != NULL);
            Edge* currentPrevEdge = currentEdge->prev;
            Edge* currentNextEdge = currentEdge->next;
            Edge* currentOppEdge;
            Edge* currentOppPrevEdge;
            Edge* currentOppNextEdge;
            if(hasOppFace){
                currentOppEdge = currentNextEdge->twin->next;
                currentOppPrevEdge = currentNextEdge->twin;
                currentOppNextEdge = currentOppEdge->next;
            }

            //Edge to Edge relations 
            Edge* newEdge1 = new Edge();
            Edge* newEdge2 = new Edge();
            Edge* newEdge3 = new Edge();
            Edge* newEdge4 = new Edge();
            Edge* newEdge5 = new Edge();
            Edge* newEdge6 = new Edge();
            Edge* newEdge7;
            Edge* newEdge8;
            if(hasOppFace){
                newEdge7 = new Edge();
                newEdge8 = new Edge();
            }

            //Twin Edges
            newEdge1->twin = newEdge2;
            newEdge2->twin = newEdge1;
            newEdge3->twin = newEdge4;
            newEdge4->twin = newEdge3;
            newEdge5->twin = newEdge6;
            newEdge6->twin = newEdge5;
            if(hasOppFace){
                newEdge7->twin = newEdge8;
                newEdge8->twin = newEdge7;
            }

            //Next/Prev Edges
            newEdge1->next = currentEdge;
            newEdge1->prev = currentNextEdge;
            currentEdge->prev = newEdge1;
            currentNextEdge->next = newEdge1;
            newEdge4->next = newEdge5;
            newEdge4->prev = currentPrevEdge;
            currentPrevEdge->next = newEdge4;
            currentPrevEdge->prev = newEdge5;
            newEdge5->next = currentPrevEdge;
            newEdge5->prev = newEdge4;
            if(hasOppFace){
                newEdge7->next = currentOppNextEdge;
                newEdge7->prev = newEdge6;
                newEdge6->next = newEdge7;
                newEdge6->prev = currentOppNextEdge;
                currentOppNextEdge->next = newEdge6;
                currentOppNextEdge->prev = newEdge7;
                newEdge8->next = currentOppPrevEdge;
                newEdge8->prev = currentOppEdge;
                currentOppPrevEdge->prev = newEdge8;
                currentOppEdge->next = newEdge8;
            }

            //Edge to Particle relations 
            newEdge1->startParticle = nextParticle;
            newEdge2->startParticle = newParticleLeft;
            newEdge3->startParticle = nextParticle;
            newEdge5->startParticle = nextParticle;
            newEdge6->startParticle = currentPrevEdge->startParticle;
            currentNextEdge->twin->startParticle = nextParticle;
            Edge* rightEdge = newEdge4;
            while(rightEdge != rightCrossEdge){
                rightEdge->startParticle = newParticleRight;
                rightEdge = rightEdge->prev->twin;
            }
            rightEdge->startParticle = newParticleRight;
            if(hasOppFace){
                newEdge7->startParticle = nextParticle;
                newEdge8->startParticle = currentOppNextEdge->startParticle;
            }

            //Particle to Edge relations 
            nextParticle->edge = newEdge1;
            newParticleLeft->edge = newEdge2;
            newParticleRight->edge = newEdge4;

            //Obtaining old particle indices
            int oldIndexLeft, oldIndexRight;
            Face* oldFace = currentEdge->face;
            if(oldFace->edge == currentEdge){
                oldIndexLeft = oldFace->indices[1];
                oldIndexRight = oldFace->indices[2];
            }else if(oldFace->edge == currentEdge->next){
                oldIndexLeft = oldFace->indices[0];
                oldIndexRight = oldFace->indices[1];
            }else{
                oldIndexRight = oldFace->indices[0];
                oldIndexLeft = oldFace->indices[2];
            }
            this->particle_list[oldIndexRight]->edge = currentPrevEdge;

            //Face to particle/edge relations 
            oldFace->setFace(newIndexLeft, oldIndexLeft, nextIndex);
            oldFace->edge = currentEdge;
            Face* newFace = new Face(newIndexRight, nextIndex, oldIndexRight);
            newFace->edge = newEdge4;

            rightEdge = currentPrevEdge->twin;
            Face* rightFace = rightEdge->face;
            while(rightEdge != rightCrossEdge){
                for(unsigned int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                        break;
                    }
                }
                rightEdge = rightEdge->prev->twin;
                rightFace = rightEdge->face;
            }

            //Edge to Face relations 
            newEdge1->face = oldFace;
            newEdge4->face = newFace;
            newEdge5->face = newFace;
            currentPrevEdge->face = newFace;

            //Adding new edges/faces
            leftCrossEdge = newEdge1;
            rightCrossEdge = newEdge3;
            this->edge_list.push_back(newEdge1);
            this->edge_list.push_back(newEdge2);
            this->edge_list.push_back(newEdge3);
            this->edge_list.push_back(newEdge4);
            this->edge_list.push_back(newEdge5);
            this->edge_list.push_back(newEdge6);
            this->face_list.push_back(newFace);

            if(hasOppFace){
                int oldOppIndexLeft, oldOppIndexRight, centreOppIndex;
                Face* oldOppFace = currentOppEdge->face;
                if(oldOppFace->edge == currentOppEdge){
                    oldOppIndexLeft = oldOppFace->indices[0];
                    centreOppIndex = oldOppFace->indices[1];
                    oldOppIndexRight = oldOppFace->indices[2];
                }else if(oldOppFace->edge == currentOppNextEdge){
                    oldOppIndexLeft = oldOppFace->indices[2];
                    centreOppIndex = oldOppFace->indices[0];
                    oldOppIndexRight = oldOppFace->indices[1];
                }else{
                    oldOppIndexLeft = oldOppFace->indices[1];
                    centreOppIndex = oldOppFace->indices[2];
                    oldOppIndexRight = oldOppFace->indices[0];
                }

                oldOppFace->setFace(oldOppIndexLeft, centreOppIndex, nextIndex);
                oldOppFace->edge = currentOppEdge;
                Face* newOppFace = new Face(nextIndex, centreOppIndex, oldOppIndexRight);
                newOppFace->edge = newEdge7;

                newEdge7->face = newOppFace;
                newEdge8->face = oldOppFace;
                newEdge6->face = newOppFace;
                currentOppNextEdge->face = newOppFace;

                this->edge_list.push_back(newEdge7);
                this->edge_list.push_back(newEdge8);
                this->face_list.push_back(newOppFace);
            }
        }else{
            //Current: Particle, Next: Face
            vec3 newInitPos = get<0>(nextIntPt);

            //Finding the intersecting face
            auto edgeList = newParticleLeft->getEdges();
            Edge* currentEdge;
            for(unsigned int i=0;i<edgeList.size();i++){
                if(edgeList[i]->face != NULL){
                    if(isInside(edgeList[i]->face, newInitPos)){
                        currentEdge = edgeList[i];
                        break;
                    }
                }
            }
            vec3 newPos = getPosAtFace(currentEdge->face, newInitPos);
            vec3 newVelocity = getVelAtFace(currentEdge->face, newPos);
            Particle* nextParticle = new Particle(newPos, newInitPos, newVelocity);
            int nextIndex = this->particle_list.size();
            nextParticle->listIdx = nextIndex;
            this->particle_list.push_back(nextParticle);

            //Obtaining old particle indices
            int oldIndexLeft, oldIndexRight;
            Face* oldFace = currentEdge->face;
            if(oldFace->edge == currentEdge){
                oldIndexLeft = oldFace->indices[1];
                oldIndexRight = oldFace->indices[2];
            }else if(oldFace->edge == currentEdge->prev){
                oldIndexLeft = oldFace->indices[2];
                oldIndexRight = oldFace->indices[0];
            }else{
                oldIndexLeft = oldFace->indices[0];
                oldIndexRight = oldFace->indices[1];
            }

            //Edge to edge relations
            Edge* newEdge1 = new Edge();
            Edge* newEdge2 = new Edge();
            Edge* newEdge3 = new Edge();
            Edge* newEdge4 = new Edge(); 
            Edge* newEdge5 = new Edge();
            Edge* newEdge6 = new Edge();
            Edge* newEdge7 = new Edge();
            Edge* newEdge8 = new Edge();
            Edge* currentPrevEdge = currentEdge->prev;
            Edge* currentNextEdge = currentEdge->next;

            //Twin Edges
            newEdge1->twin = newEdge2;
            newEdge2->twin = newEdge1;
            newEdge3->twin = newEdge4;
            newEdge4->twin = newEdge3;
            newEdge5->twin = newEdge6;
            newEdge6->twin = newEdge5;
            newEdge7->twin = newEdge8;
            newEdge8->twin = newEdge7;

            //Next/Prev edges
            newEdge2->next = currentEdge;
            newEdge2->prev = newEdge3;
            newEdge3->next = newEdge2;
            newEdge3->prev = currentEdge;
            currentEdge->next = newEdge3;
            currentEdge->prev = newEdge2;
            newEdge4->next = currentNextEdge;
            newEdge4->prev = newEdge5;
            newEdge5->next = newEdge4;
            newEdge5->prev = currentNextEdge;
            currentNextEdge->next = newEdge5;
            currentNextEdge->prev = newEdge4;
            newEdge6->next = currentPrevEdge;
            newEdge6->prev = newEdge7;
            newEdge7->next = newEdge6;
            newEdge7->prev = currentPrevEdge;
            currentPrevEdge->next = newEdge7;
            currentPrevEdge->prev = newEdge6;

            //Edge to particle relations 
            newEdge1->startParticle = newParticleLeft;
            newEdge2->startParticle = nextParticle;
            newEdge3->startParticle = this->particle_list[oldIndexLeft];
            newEdge4->startParticle = nextParticle;
            newEdge5->startParticle = this->particle_list[oldIndexRight];
            newEdge6->startParticle = nextParticle;
            newEdge8->startParticle = nextParticle;
            Edge* rightEdge = newEdge7;
            while(rightEdge != rightCrossEdge){
                rightEdge->startParticle = newParticleRight;
                rightEdge = rightEdge->prev->twin;
            }
            rightEdge->startParticle = newParticleRight;

            //Particle to edge relations 
            newParticleLeft->edge = newEdge1;
            nextParticle->edge = newEdge2;
            newParticleRight->edge = newEdge7;

            //Face to particle/edge relations 
            oldFace->setFace(newIndexLeft, oldIndexLeft, nextIndex);
            oldFace->edge = currentEdge;
            Face* newFaceBot = new Face(oldIndexLeft, oldIndexRight, nextIndex);
            newFaceBot->edge = currentNextEdge;
            Face* newFaceTop = new Face(nextIndex, oldIndexRight, newIndexRight);
            newFaceTop->edge = newEdge6;
            rightEdge = currentPrevEdge->twin;
            Face* rightFace = rightEdge->face;
            while(rightEdge != rightCrossEdge){
                for(unsigned int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                        break;
                    }
                }
                rightEdge = rightEdge->prev->twin;
                rightFace = rightEdge->face;
            }

            //Edge to face relations
            newEdge2->face = oldFace;
            newEdge3->face = oldFace;
            newEdge4->face = newFaceBot;
            newEdge5->face = newFaceBot;
            newEdge6->face = newFaceTop;
            newEdge7->face = newFaceTop;
            currentNextEdge->face = newFaceBot;
            currentPrevEdge->face = newFaceTop;

            //Adding new edges and faces to lists
            leftCrossEdge = newEdge2;
            rightCrossEdge = newEdge8;
            this->edge_list.push_back(newEdge1);
            this->edge_list.push_back(newEdge2);
            this->edge_list.push_back(newEdge3);
            this->edge_list.push_back(newEdge4);
            this->edge_list.push_back(newEdge5);
            this->edge_list.push_back(newEdge6);
            this->edge_list.push_back(newEdge7);
            this->edge_list.push_back(newEdge8);
            this->face_list.push_back(newFaceBot);
            this->face_list.push_back(newFaceTop);
        }
    }

    if(leftCrossEdge != NULL){
        leftCrossEdge->isBoundary = true;
    }
    if(rightCrossEdge != NULL){
        rightCrossEdge->isBoundary = true;
    }
}

void HalfEdge::reMeshEdge(int i){
    Edge* currentEdge = this->edge_list[i];
    int n = this->particle_list.size();
    int cutCase = 0;
    if(currentEdge->face == NULL){
        //The edge is shared by only one face, i.e. a boundary edge
        cutCase = 1;
        currentEdge = currentEdge->twin; 
    }else if(currentEdge->twin->face == NULL){
        cutCase = 1; 
    }

    //New mesh entities
    Particle* newParticleLeft;
    Particle* newParticleRight;
    int newIndexLeft;
    int newIndexRight;
    Particle* leftParticle = currentEdge->startParticle;
    Particle* rightParticle = currentEdge->twin->startParticle;
    vec3 newPos = (leftParticle->position + rightParticle->position) / 2.0f;
    vec3 normal = (leftParticle->position - rightParticle->position).normalized();
    vec3 newInitPos = (leftParticle->initPos + rightParticle->initPos) / 2.0f;
    vec3 newVelocity = (leftParticle->velocity + rightParticle->velocity) / 2.0f;
    newParticleLeft = new Particle(newPos + normal * 0.1f, newInitPos, newVelocity);
    newParticleRight = new Particle(newPos - normal * 0.1f, newInitPos, newVelocity);
    newIndexLeft = n;
    newIndexRight = n+1;
    newParticleLeft->listIdx = newIndexLeft;
    newParticleRight->listIdx = newIndexRight;
    this->particle_list.push_back(newParticleLeft);
    this->particle_list.push_back(newParticleRight);

    if(cutCase){
        //Obtaining old particle indices
        Particle* centreParticle = currentEdge->prev->startParticle;
        int oldIndexLeft = leftParticle->listIdx;
        int oldIndexRight = rightParticle->listIdx;
        int oldIndexCentre = centreParticle->listIdx;

        //Edge edge relations
        Edge* newEdge1 = new Edge();
        Edge* newEdge2 = new Edge();
        Edge* newEdge3 = new Edge();
        Edge* newEdge4 = new Edge();
        Edge* newEdge5 = new Edge();
        Edge* newEdge6 = new Edge();

        //Twin edges
        newEdge1->twin = newEdge2;
        newEdge2->twin = newEdge1;
        newEdge3->twin = newEdge4;
        newEdge4->twin = newEdge3;
        newEdge5->twin = newEdge6;
        newEdge6->twin = newEdge5;

        //Next/Prev edges
        newEdge2->next = currentEdge->next;
        newEdge2->prev = newEdge3;
        newEdge3->next = newEdge2;
        newEdge3->prev = currentEdge->next;
        currentEdge->next->next = newEdge3;
        currentEdge->next->prev = newEdge2;
        newEdge6->next = currentEdge->prev;
        newEdge6->prev = currentEdge;
        currentEdge->prev->prev = newEdge6;
        currentEdge->next = newEdge6;

        //Edge to particle relations
        newEdge1->startParticle = rightParticle;
        newEdge2->startParticle = newParticleRight;
        newEdge3->startParticle = centreParticle;
        newEdge4->startParticle = newParticleRight;
        newEdge5->startParticle = centreParticle;
        newEdge6->startParticle = newParticleLeft;
        currentEdge->twin->startParticle = newParticleLeft;

        //Particle to edge relations
        newParticleLeft->edge = newEdge6;
        newParticleRight->edge = newEdge2;
        rightParticle->edge = newEdge1;

        //Face to particle/edge relations
        Face* oldFace = currentEdge->face;
        oldFace->setFace(oldIndexLeft, newIndexLeft, oldIndexCentre);
        oldFace->edge = currentEdge;
        Face* newFace = new Face(newIndexRight, oldIndexRight, oldIndexCentre);
        newFace->edge = newEdge2;

        //Edge to face relations
        newEdge2->face = newFace; 
        newEdge2->next->face = newFace;
        newEdge3->face = newFace;
        newEdge6->face = oldFace;

        //Adding new edges/faces
        this->edge_list.push_back(newEdge1);
        this->edge_list.push_back(newEdge2);
        this->edge_list.push_back(newEdge3);
        this->edge_list.push_back(newEdge4);
        this->edge_list.push_back(newEdge5);
        this->edge_list.push_back(newEdge6);
        this->face_list.push_back(newFace);
    }else{
        //Obtaining old particle indices
        Particle* centreParticle1 = currentEdge->prev->startParticle;
        Particle* centreParticle2 = currentEdge->twin->prev->startParticle;
        int oldIndexLeft = leftParticle->listIdx;
        int oldIndexRight = rightParticle->listIdx;
        int oldIndexCentre1 = centreParticle1->listIdx;
        int oldIndexCentre2 = centreParticle2->listIdx;

        //Edge edge relations
        Edge* newEdge1 = new Edge();
        Edge* newEdge2 = new Edge();
        Edge* newEdge3 = new Edge();
        Edge* newEdge4 = new Edge();
        Edge* newEdge5 = new Edge();
        Edge* newEdge6 = new Edge();
        Edge* newEdge7 = new Edge();
        Edge* newEdge8 = new Edge();
        Edge* newEdge9 = new Edge();
        Edge* newEdge10 = new Edge();

        //Twin edges
        newEdge1->twin = newEdge2;
        newEdge2->twin = newEdge1;
        newEdge3->twin = newEdge4;
        newEdge4->twin = newEdge3;
        newEdge5->twin = newEdge6;
        newEdge6->twin = newEdge5;
        newEdge7->twin = newEdge8;
        newEdge8->twin = newEdge7;
        newEdge9->twin = newEdge10;
        newEdge10->twin = newEdge9;

        //Next/Prev edges
        Edge* currentPrevEdge2 = currentEdge->twin->prev;
        newEdge1->next = currentEdge->twin;
        newEdge1->prev = currentEdge->twin->next;
        currentEdge->twin->prev = newEdge1;
        currentEdge->twin->next->next = newEdge1;
        newEdge4->next = currentPrevEdge2;
        newEdge4->prev = newEdge5;
        currentPrevEdge2->next = newEdge5;
        currentPrevEdge2->prev = newEdge4;
        newEdge5->next = newEdge4;
        newEdge5->prev = currentPrevEdge2;
        newEdge6->next = currentEdge->next;
        newEdge6->prev = newEdge7;
        newEdge7->next = newEdge6;
        newEdge7->prev = currentEdge->next;
        currentEdge->next->next = newEdge7;
        currentEdge->next->prev = newEdge6;
        newEdge10->next = currentEdge->prev;
        newEdge10->prev = currentEdge;
        currentEdge->prev->prev = newEdge10;
        currentEdge->next = newEdge10;

        //Edge to particle relations
        newEdge1->startParticle = centreParticle2;
        newEdge2->startParticle = newParticleLeft;
        newEdge3->startParticle = centreParticle2;
        newEdge4->startParticle = newParticleRight;
        newEdge5->startParticle = rightParticle;
        newEdge6->startParticle = newParticleRight;
        newEdge7->startParticle = centreParticle1;
        newEdge8->startParticle = newParticleRight;
        newEdge9->startParticle = centreParticle1;
        newEdge10->startParticle = newParticleLeft;
        currentEdge->twin->startParticle = newParticleLeft;

        //Particle to edge relations
        newParticleLeft->edge = newEdge10;
        newParticleRight->edge = newEdge6;
        rightParticle->edge = newEdge5;

        //Face to particle/edge relations
        Face* oldFace1 = currentEdge->face;
        Face* oldFace2 = currentEdge->twin->face;
        oldFace1->setFace(oldIndexLeft, newIndexLeft, oldIndexCentre1);
        oldFace1->edge = currentEdge;
        oldFace2->setFace(newIndexLeft, oldIndexLeft, oldIndexCentre2);
        oldFace2->edge = currentEdge->twin;
        Face* newFace1 = new Face(newIndexRight, oldIndexRight, oldIndexCentre1);
        newFace1->edge = newEdge6;
        Face* newFace2 = new Face(oldIndexRight, newIndexRight, oldIndexCentre2);
        newFace2->edge = newEdge5;

        //Edge to face relations
        newEdge1->face = oldFace2;
        newEdge4->face = newFace2;
        newEdge5->face = newFace2;
        currentPrevEdge2->face = newFace2;
        newEdge6->face = newFace1; 
        newEdge6->next->face = newFace1;
        newEdge7->face = newFace1;
        newEdge10->face = oldFace1;

        //Adding new edges/faces
        this->edge_list.push_back(newEdge1);
        this->edge_list.push_back(newEdge2);
        this->edge_list.push_back(newEdge3);
        this->edge_list.push_back(newEdge4);
        this->edge_list.push_back(newEdge5);
        this->edge_list.push_back(newEdge6);
        this->edge_list.push_back(newEdge7);
        this->edge_list.push_back(newEdge8);
        this->edge_list.push_back(newEdge9);
        this->edge_list.push_back(newEdge10);
        this->face_list.push_back(newFace1);
        this->face_list.push_back(newFace2);
    }
}

void HalfEdge::splitVertex(int vertexIdx, vec3 normal){
    //Vertex Splitting Algorithm
    //Step 1: Choose the vertex to split using the given edge
    Particle* P = particle_list[vertexIdx];
    bool validP; 
    //Check validity of A
    int P_positive_count = 0;
    int P_negative_count = 0;
    vector<Face*> P_faces;
    //Anticlockwise traversal
    Edge* currentEdge = P->edge;
    int flag = 0;
    while(true){
        if(currentEdge == P->edge && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                if(currentEdge->twin->ghostTwinAcw == NULL){
                    break;
                }else if(currentEdge->twin->ghostTwinAcw->startParticle == currentEdge->startParticle){
                    currentEdge = currentEdge->twin->ghostTwinAcw;
                }else{
                    break;
                }
            }else{
                currentEdge = currentEdge->prev->twin;
            }
        }
        flag = 1;
    }
    //Clockwise traversal
    Edge* startEdge = currentEdge;
    flag = 0;
    while(true){
        if(currentEdge == startEdge && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                currentEdge = currentEdge->twin->next;
            }else{
                //Process current face
                P_faces.push_back(currentEdge->face);
                vec3 v1 = currentEdge->startParticle->position;
                vec3 v2 = currentEdge->next->startParticle->position;
                vec3 v3 = currentEdge->prev->startParticle->position;
                vec3 centroid = (v1 + v2 + v3) / 3.0f;
                float factor = (centroid - P->position).dot(normal); 
                if(factor >= 0.0f){
                    P_positive_count++;
                }else{
                    P_negative_count++;
                }
                if(currentEdge->twin->next == NULL){
                    if(currentEdge->ghostTwinCw == NULL){
                        break;
                    }else if(currentEdge->ghostTwinCw->twin->startParticle == currentEdge->startParticle){
                        currentEdge = currentEdge->ghostTwinCw->twin;
                    }else{
                        break;
                    }
                }else{
                    currentEdge = currentEdge->twin->next;
                }
            }
        }
        flag = 1;
    }
    validP = (P_positive_count != 0) && (P_negative_count != 0);
    if(!validP){
        return;
    }

    //Step 2: Split the chosen vertex
    Particle* newParticle = new Particle(P->position, P->initPos, P->velocity);
    newParticle->listIdx = this->particle_list.size();
    this->particle_list.push_back(newParticle);
    int oldIdx = P->listIdx;
    int newIdx = newParticle->listIdx;
    P->crackTip = false;
    
    //Step 3: Split the faces and edges between the old and the new vertex (i.e. remesh)
    //Step 4: Assign ghost twins to newly created edges
    int lastSign = 0; 
    Face* currFace;
    if(startEdge->face != NULL || startEdge->twin->ghostTwinAcw != NULL){
        currFace = P_faces[P_faces.size() - 1];
        vec3 v1 = particle_list[currFace->indices[0]]->position;
        vec3 v2 = particle_list[currFace->indices[1]]->position;
        vec3 v3 = particle_list[currFace->indices[2]]->position;
        vec3 centroid = (v1 + v2 + v3) / 3.0f;
        if((centroid - P->position).dot(normal) >= 0.0f){
            lastSign = 1;
        }else{
            lastSign = -1;
        }
    }
    vector<Edge*> ghostTwinEdges;
    vector<int> ghostVertices;
    ghostVertices.push_back(oldIdx);
    ghostVertices.push_back(newIdx);
    for(int i = 0; i < P_faces.size(); i++){
        currFace = P_faces[i];
        currFace->color = vec3(0.0f, 1.0f, 0.0f);
        vec3 v1 = particle_list[currFace->indices[0]]->position;
        vec3 v2 = particle_list[currFace->indices[1]]->position;
        vec3 v3 = particle_list[currFace->indices[2]]->position;
        vec3 centroid = (v1 + v2 + v3) / 3.0f;
        float factor = (centroid - P->position).dot(normal);
        if(abs(factor) < 0.0001f){
            factor = lastSign;
        }
        currentEdge = currFace->edge;
        while(currentEdge->startParticle != P && currentEdge->startParticle != newParticle){
            currentEdge = currentEdge->next;
        }
        if(lastSign * factor >= 0.0f){
            if(factor < 0.0f){
                int oldLeftIdx = currentEdge->prev->startParticle->listIdx;
                int oldRightIdx = currentEdge->next->startParticle->listIdx;
                currFace->setFace(newIdx, oldRightIdx, oldLeftIdx);
                currFace->edge = currentEdge;
                newParticle->edge = currentEdge;
                currentEdge->startParticle = newParticle;
                currentEdge->prev->twin->startParticle = newParticle;
                lastSign = -1;
            }else{
                currentEdge->startParticle = P;
                currentEdge->prev->twin->startParticle = P;
                P->edge = currentEdge;
                lastSign = 1;
            }
        }else{
            if(factor < 0.0f){
                currentEdge->startParticle = newParticle;
                newParticle->edge = currentEdge;
                int oldLeftIdx = currentEdge->prev->startParticle->listIdx;
                int oldRightIdx = currentEdge->next->startParticle->listIdx;
                currFace->setFace(newIdx, oldRightIdx, oldLeftIdx);
                currFace->edge = currentEdge;
                if(currentEdge->prev->twin->face == NULL){
                    //Joined by ghost twin edge
                    currentEdge->prev->twin->startParticle = newParticle;
                    int M = P_faces.size();
                    Face* prevFace = P_faces[(i+M-1)%M];
                    Edge* prevEdge = prevFace->edge;
                    while(prevEdge->startParticle != P && prevEdge->startParticle != newParticle){
                        prevEdge = prevEdge->next;
                    }
                    ghostTwinEdges.push_back(prevEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                }else{
                    //Joined by normal twin edge
                    Particle* rightParticle = currentEdge->prev->startParticle;
                    rightParticle->crackTip = true;
                    ghostVertices.push_back(rightParticle->listIdx);
                    Edge* upperEdge = new Edge();
                    Edge* lowerEdge = new Edge();
                    Edge* currentOppEdge = currentEdge->prev->twin;
                    upperEdge->startParticle = rightParticle;
                    lowerEdge->startParticle = newParticle;
                    currentEdge->prev->twin = lowerEdge;
                    lowerEdge->twin = currentEdge->prev;
                    upperEdge->twin = currentOppEdge;
                    currentOppEdge->twin = upperEdge;
                    //Assigning ghost twins
                    ghostTwinEdges.push_back(currentOppEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                    this->edge_list.push_back(upperEdge);
                    this->edge_list.push_back(lowerEdge);
                }
                lastSign = -1;
            }else{
                currentEdge->startParticle = P;
                P->edge = currentEdge;
                if(currentEdge->prev->twin->face == NULL){
                    //Joined by ghost twin edges
                    currentEdge->prev->twin->startParticle = P;
                    int M = P_faces.size();
                    Face* prevFace = P_faces[(i+M-1)%M];
                    Edge* prevEdge = prevFace->edge;
                    while(prevEdge->startParticle != P && prevEdge->startParticle != newParticle){
                        prevEdge = prevEdge->next;
                    }
                    ghostTwinEdges.push_back(prevEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                }else{
                    //Joined by normal twin edges
                    Particle* rightParticle = currentEdge->prev->startParticle;
                    rightParticle->crackTip = true;
                    ghostVertices.push_back(rightParticle->listIdx);
                    Edge* upperEdge = new Edge();
                    Edge* lowerEdge = new Edge();
                    Edge* currentOppEdge = currentEdge->prev->twin;
                    upperEdge->startParticle = rightParticle;
                    lowerEdge->startParticle = P;
                    currentEdge->prev->twin = lowerEdge;
                    lowerEdge->twin = currentEdge->prev;
                    upperEdge->twin = currentOppEdge;
                    currentOppEdge->twin = upperEdge;
                    //Assigning ghost twins
                    ghostTwinEdges.push_back(currentOppEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                    this->edge_list.push_back(upperEdge);
                    this->edge_list.push_back(lowerEdge);
                }
                lastSign = 1;
            }
        }
    }

    //Assigning stored ghost twin edges
    int N = ghostTwinEdges.size();
    if(N == 2){
        ghostTwinEdges[0]->ghostTwinAcw = ghostTwinEdges[1];
        ghostTwinEdges[1]->ghostTwinCw = ghostTwinEdges[0];
    }else if(N == 4){
        for(int i = 0; i < N; i++){
            if(!ghostTwinEdges[i]->ghostTwinCw || i%2==0){
                ghostTwinEdges[i]->ghostTwinCw = ghostTwinEdges[(i+N-1)%N];
            }
            if(!ghostTwinEdges[i]->ghostTwinAcw || i%2==1){
                ghostTwinEdges[i]->ghostTwinAcw = ghostTwinEdges[(i+1)%N]; 
            }
        }
    }

    //Handling ghost vertices
    if(debugMode){
        for(int i = 0; i < ghostVertices.size(); i++){
            splitGhostVertex(ghostVertices[i]);
        }
    }
}

void HalfEdge::splitGhostVertex(int vertexIdx){
    //Step 1: Check if there are two disconnected components of faces around the given vertex
    Particle* P = particle_list[vertexIdx];
    Edge* currentEdge = P->edge;
    //Anticlockwise traversal
    int flag = 0;
    while(true){
        if(currentEdge == P->edge && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                if(currentEdge->twin->ghostTwinAcw == NULL){
                    break;
                }else if(currentEdge->twin->ghostTwinAcw->startParticle == currentEdge->startParticle){
                    currentEdge = currentEdge->twin->ghostTwinAcw;
                }else{
                    break;
                }
            }else{
                currentEdge = currentEdge->prev->twin;
            }
        }
        flag = 1;
    }
    //Clockwise traversal
    Edge* startEdge = currentEdge;
    vector<Face*> P_faces;
    vector<int> label; 
    flag = 0;
    int count = 0;
    while(true){
        if(currentEdge == startEdge && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                currentEdge = currentEdge->twin->next;
            }else{
                P_faces.push_back(currentEdge->face);
                label.push_back(count % 2);
                if(currentEdge->twin->next == NULL){
                    count++;
                    if(currentEdge->ghostTwinCw == NULL){
                        break;
                    }else if(currentEdge->ghostTwinCw->twin->startParticle == currentEdge->startParticle){
                        currentEdge = currentEdge->ghostTwinCw->twin;
                    }else{
                        break;
                    }
                }else{
                    currentEdge = currentEdge->twin->next;
                }
            }
        }
        flag = 1;
    }
    if(count < 2){
        return;
    }

    //Step 2: Split the two components to different vertices
    Particle* newParticle = new Particle(P->position, P->initPos, P->velocity);
    newParticle->listIdx = this->particle_list.size();
    this->particle_list.push_back(newParticle);
    int oldIdx = P->listIdx;
    int newIdx = newParticle->listIdx;
    P->crackTip = false;

    int lastLabel = -1;
    vector<Edge*> ghostTwinEdges;
    if(startEdge->face != NULL || startEdge->twin->ghostTwinAcw != NULL){
        lastLabel = label[label.size() - 1];
    }
    for(int i = 0; i < P_faces.size(); i++){
        Face* currFace = P_faces[i];
        currentEdge = currFace->edge;
        while(currentEdge->startParticle != P && currentEdge->startParticle != newParticle){
            currentEdge = currentEdge->next;
        }
        if(label[i]){
            //Retains original vertex
            currentEdge->startParticle = P;
            currentEdge->prev->twin->startParticle = P;
            P->edge = currentEdge;
        }else{
            //Reassigned to new particle
            int oldLeftIdx = currentEdge->prev->startParticle->listIdx;
            int oldRightIdx = currentEdge->next->startParticle->listIdx;
            currFace->setFace(newIdx, oldRightIdx, oldLeftIdx);
            currFace->edge = currentEdge;
            currentEdge->startParticle = newParticle;
            currentEdge->prev->twin->startParticle = newParticle;
            newParticle->edge = currentEdge;
        }
        if(lastLabel != -1){
            if(lastLabel != label[i]){
                ghostTwinEdges.push_back(currentEdge->prev->ghostTwinAcw);
                ghostTwinEdges.push_back(currentEdge->prev);
            }
        }
        lastLabel = label[i];
    }

    //Assigning stored ghost twin edges
    int N = ghostTwinEdges.size();
    if(N == 2){
        ghostTwinEdges[0]->ghostTwinAcw = ghostTwinEdges[1];
        ghostTwinEdges[1]->ghostTwinCw = ghostTwinEdges[0];
    }else if(N == 4){
        for(int i = 0; i < N; i++){
            if(!ghostTwinEdges[i]->ghostTwinCw || i%2==0){
                ghostTwinEdges[i]->ghostTwinCw = ghostTwinEdges[(i+N-1)%N];
            }
            if(!ghostTwinEdges[i]->ghostTwinAcw || i%2==1){
                ghostTwinEdges[i]->ghostTwinAcw = ghostTwinEdges[(i+1)%N]; 
            }
        }
    }
}

//Vertex Splitting, adapted from Fast Simulation of Cloth Tearing 
void HalfEdge::reMeshEdge2(int vertexIdx){
    Edge* cutEdge = this->edge_list[vertexIdx];

    //Vertex Splitting Algorithm
    //Step 1: Choose the vertex to split using the given edge
    Particle* A = cutEdge->startParticle;
    Particle* B = cutEdge->twin->startParticle;
    bool validA, validB;
    vec3 normal = (B->position - A->position).normalized();
    //Check validity of A
    int A_positive_count = 0;
    int A_negative_count = 0;
    vector<Face*> A_faces;
    //Anticlockwise traversal
    Edge* currentEdge = cutEdge;
    int flag = 0;
    while(true){
        if(currentEdge == cutEdge && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                if(currentEdge->twin->ghostTwinAcw == NULL){
                    break;
                }else if(currentEdge->twin->ghostTwinAcw->startParticle == currentEdge->startParticle){
                    currentEdge = currentEdge->twin->ghostTwinAcw;
                }else{
                    break;
                }
            }else{
                currentEdge = currentEdge->prev->twin;
            }
        }
        flag = 1;
    }
    //Clockwise traversal
    Edge* startEdgeA = currentEdge;
    flag = 0;
    while(true){
        if(currentEdge == startEdgeA && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                currentEdge = currentEdge->twin->next;
            }else{
                //Process current face
                A_faces.push_back(currentEdge->face);
                vec3 v1 = currentEdge->startParticle->position;
                vec3 v2 = currentEdge->next->startParticle->position;
                vec3 v3 = currentEdge->prev->startParticle->position;
                vec3 centroid = (v1 + v2 + v3) / 3.0f;
                float factor = (centroid - A->position).dot(normal); 
                if(factor >= 0.0f){
                    A_positive_count++;
                }else{
                    A_negative_count++;
                }
                if(currentEdge->twin->next == NULL){
                    if(currentEdge->ghostTwinCw == NULL){
                        break;
                    }else if(currentEdge->ghostTwinCw->twin->startParticle == currentEdge->startParticle){
                        currentEdge = currentEdge->ghostTwinCw->twin;
                    }else{
                        break;
                    }
                }else{
                    currentEdge = currentEdge->twin->next;
                }
            }
        }
        flag = 1;
    }
    validA = (A_positive_count != 0) && (A_negative_count != 0);
    //Check validity of B
    int B_positive_count = 0;
    int B_negative_count = 0;
    vector<Face*> B_faces;
    //Anticlockwise traversal
    currentEdge = cutEdge->twin;
    flag = 0;
    while(true){
        if(currentEdge == cutEdge->twin && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                if(currentEdge->twin->ghostTwinAcw == NULL){
                    break;
                }else if(currentEdge->twin->ghostTwinAcw->startParticle == currentEdge->startParticle){
                    currentEdge = currentEdge->twin->ghostTwinAcw;
                }else{
                    break;
                }
            }else{
                currentEdge = currentEdge->prev->twin;
            }
        }
        flag = 1;
    }
    //Clockwise Traversal
    Edge* startEdgeB = currentEdge;
    flag = 0;
    while(true){
        if(currentEdge == startEdgeB && flag){
            break;
        }else{
            if(currentEdge->face == NULL){
                currentEdge = currentEdge->twin->next;
            }else{
                //Process current face
                B_faces.push_back(currentEdge->face);
                vec3 v1 = currentEdge->startParticle->position;
                vec3 v2 = currentEdge->next->startParticle->position;
                vec3 v3 = currentEdge->prev->startParticle->position;
                vec3 centroid = (v1 + v2 + v3) / 3.0f;
                float factor = (centroid - B->position).dot(normal); 
                if(factor >= 0.0f){
                    B_positive_count++;
                }else{
                    B_negative_count++;
                }
                if(currentEdge->twin->next == NULL){
                    if(currentEdge->ghostTwinCw == NULL){
                        break;
                    }else if(currentEdge->ghostTwinCw->twin->startParticle == currentEdge->startParticle){
                        currentEdge = currentEdge->ghostTwinCw->twin;
                    }else{
                        break;
                    }
                }else{
                    currentEdge = currentEdge->twin->next;
                }
            }
        }
        flag = 1;
    }
    validB = (B_positive_count != 0) && (B_negative_count != 0);
    //Choose vertex to split
    Particle* P;
    vector<Face*> P_faces;
    Edge* startEdge;
    if(validA || validB){
        if(validA && validB){
            if(A->crackTip){
                P = A;
                P_faces = A_faces;
                startEdge = startEdgeA;
            }else if(B->crackTip){
                P = B;
                P_faces = B_faces;
                startEdge = startEdgeB;
            }else{
                if(A->m >= B->m){
                    P = A;
                    P_faces = A_faces;
                    startEdge = startEdgeA;
                }else{
                    P = B;
                    P_faces = B_faces;
                    startEdge = startEdgeB;
                }
            }
        }else if(validA){
            P = A;
            P_faces = A_faces;
            startEdge = startEdgeA;
        }else{
            P = B;
            P_faces = B_faces;
            startEdge = startEdgeB;
        }
    }else{
        return;
    }

    //Step 2: Split the chosen vertex
    Particle* newParticle = new Particle(P->position, P->initPos, P->velocity);
    newParticle->listIdx = this->particle_list.size();
    this->particle_list.push_back(newParticle);
    int oldIdx = P->listIdx;
    int newIdx = newParticle->listIdx;
    P->crackTip = false;
    
    //Step 3: Split the faces and edges between the old and the new vertex (i.e. remesh)
    //Step 4: Assign ghost twins to newly created edges
    int lastSign = 0; 
    Face* currFace;
    if(startEdge->face != NULL || startEdge->twin->ghostTwinAcw != NULL){
        currFace = P_faces[P_faces.size() - 1];
        vec3 v1 = particle_list[currFace->indices[0]]->position;
        vec3 v2 = particle_list[currFace->indices[1]]->position;
        vec3 v3 = particle_list[currFace->indices[2]]->position;
        vec3 centroid = (v1 + v2 + v3) / 3.0f;
        if((centroid - P->position).dot(normal) >= 0.0f){
            lastSign = 1;
        }else{
            lastSign = -1;
        }
    }
    vector<Edge*> ghostTwinEdges;
    vector<int> ghostVertices;
    ghostVertices.push_back(oldIdx);
    ghostVertices.push_back(newIdx);
    for(int i = 0; i < P_faces.size(); i++){
        currFace = P_faces[i];
        currFace->color = vec3(0.0f, 1.0f, 0.0f);
        vec3 v1 = particle_list[currFace->indices[0]]->position;
        vec3 v2 = particle_list[currFace->indices[1]]->position;
        vec3 v3 = particle_list[currFace->indices[2]]->position;
        vec3 centroid = (v1 + v2 + v3) / 3.0f;
        float factor = (centroid - P->position).dot(normal);
        if(abs(factor) < 0.0001f){
            factor = lastSign;
        }
        currentEdge = currFace->edge;
        while(currentEdge->startParticle != P && currentEdge->startParticle != newParticle){
            currentEdge = currentEdge->next;
        }
        if(lastSign * factor >= 0.0f){
            if(factor < 0.0f){
                int oldLeftIdx = currentEdge->prev->startParticle->listIdx;
                int oldRightIdx = currentEdge->next->startParticle->listIdx;
                currFace->setFace(newIdx, oldRightIdx, oldLeftIdx);
                currFace->edge = currentEdge;
                newParticle->edge = currentEdge;
                currentEdge->startParticle = newParticle;
                currentEdge->prev->twin->startParticle = newParticle;
                lastSign = -1;
            }else{
                currentEdge->startParticle = P;
                currentEdge->prev->twin->startParticle = P;
                P->edge = currentEdge;
                lastSign = 1;
            }
        }else{
            if(factor < 0.0f){
                currentEdge->startParticle = newParticle;
                newParticle->edge = currentEdge;
                int oldLeftIdx = currentEdge->prev->startParticle->listIdx;
                int oldRightIdx = currentEdge->next->startParticle->listIdx;
                currFace->setFace(newIdx, oldRightIdx, oldLeftIdx);
                currFace->edge = currentEdge;
                if(currentEdge->prev->twin->face == NULL){
                    //Joined by ghost twin edge
                    currentEdge->prev->twin->startParticle = newParticle;
                    int M = P_faces.size();
                    Face* prevFace = P_faces[(i+M-1)%M];
                    Edge* prevEdge = prevFace->edge;
                    while(prevEdge->startParticle != P && prevEdge->startParticle != newParticle){
                        prevEdge = prevEdge->next;
                    }
                    ghostTwinEdges.push_back(prevEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                }else{
                    //Joined by normal twin edge
                    Particle* rightParticle = currentEdge->prev->startParticle;
                    rightParticle->crackTip = true;
                    ghostVertices.push_back(rightParticle->listIdx);
                    Edge* upperEdge = new Edge();
                    Edge* lowerEdge = new Edge();
                    Edge* currentOppEdge = currentEdge->prev->twin;
                    upperEdge->startParticle = rightParticle;
                    lowerEdge->startParticle = newParticle;
                    currentEdge->prev->twin = lowerEdge;
                    lowerEdge->twin = currentEdge->prev;
                    upperEdge->twin = currentOppEdge;
                    currentOppEdge->twin = upperEdge;
                    //Assigning ghost twins
                    ghostTwinEdges.push_back(currentOppEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                    this->edge_list.push_back(upperEdge);
                    this->edge_list.push_back(lowerEdge);
                }
                lastSign = -1;
            }else{
                currentEdge->startParticle = P;
                P->edge = currentEdge;
                if(currentEdge->prev->twin->face == NULL){
                    //Joined by ghost twin edges
                    currentEdge->prev->twin->startParticle = P;
                    int M = P_faces.size();
                    Face* prevFace = P_faces[(i+M-1)%M];
                    Edge* prevEdge = prevFace->edge;
                    while(prevEdge->startParticle != P && prevEdge->startParticle != newParticle){
                        prevEdge = prevEdge->next;
                    }
                    ghostTwinEdges.push_back(prevEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                }else{
                    //Joined by normal twin edges
                    Particle* rightParticle = currentEdge->prev->startParticle;
                    rightParticle->crackTip = true;
                    ghostVertices.push_back(rightParticle->listIdx);
                    Edge* upperEdge = new Edge();
                    Edge* lowerEdge = new Edge();
                    Edge* currentOppEdge = currentEdge->prev->twin;
                    upperEdge->startParticle = rightParticle;
                    lowerEdge->startParticle = P;
                    currentEdge->prev->twin = lowerEdge;
                    lowerEdge->twin = currentEdge->prev;
                    upperEdge->twin = currentOppEdge;
                    currentOppEdge->twin = upperEdge;
                    //Assigning ghost twins
                    ghostTwinEdges.push_back(currentOppEdge);
                    ghostTwinEdges.push_back(currentEdge->prev);
                    this->edge_list.push_back(upperEdge);
                    this->edge_list.push_back(lowerEdge);
                }
                lastSign = 1;
            }
        }
    }

    //Assigning stored ghost twin edges
    int N = ghostTwinEdges.size();
    if(N == 2){
        ghostTwinEdges[0]->ghostTwinAcw = ghostTwinEdges[1];
        ghostTwinEdges[1]->ghostTwinCw = ghostTwinEdges[0];
    }else if(N == 4){
        for(int i = 0; i < N; i++){
            if(!ghostTwinEdges[i]->ghostTwinCw || i%2==0){
                ghostTwinEdges[i]->ghostTwinCw = ghostTwinEdges[(i+N-1)%N];
            }
            if(!ghostTwinEdges[i]->ghostTwinAcw || i%2==1){
                ghostTwinEdges[i]->ghostTwinAcw = ghostTwinEdges[(i+1)%N]; 
            }
        }
    }

    //Handling ghost vertices
    if(debugMode){
        for(int i = 0; i < ghostVertices.size(); i++){
            splitGhostVertex(ghostVertices[i]);
        }
    }
}

void HalfEdge::resetForce(){
    for(unsigned int i=0;i<particle_list.size();i++){
        particle_list[i]->netForce = vec3(0,0,0);
    }
}

void HalfEdge::solveFwdEuler(float dt){
    //1) Force calculation
    this->resetForce();
    //Spring force from springs that dont align with the mesh
    for(unsigned int i=0;i<ghostSprings.size();i++){
        ghostSprings[i].addForce();
    }
    //Spring force from springs aligned with the mesh
    for(unsigned int i=0;i<edge_list.size();i++){
        edge_list[i]->addForce();
    }
    //External Forces
    for(unsigned int i = 0; i< particle_list.size(); i++){
        vec3 f_ext = particle_list[i]->calculateExternalForce();
        particle_list[i]->netForce += f_ext;
    }

    //2) Velocity update
    for(unsigned int i = 0; i < particle_list.size(); i++){
        if(!particle_list[i]->constraint.isActive){
            particle_list[i]->velocity += (particle_list[i]->netForce * dt * particle_list[i]->invM);
        }
    }
}

void HalfEdge::solveBwdEuler(float dt){
    //Requirements
    //1) Force of the current time step
    //2) Velocity of the current time step (v_n)
    //3) Jx = df / dx (Jacobian of the force w.r.t. position)
    //4) Jv = df / dv (Jacobian of the force w.r.t. velocity)
    //4) Mass Matrix (M)
    int systemSize = particle_list.size();
    vecX f_n(systemSize);
    vecX v_n(systemSize);
    matX Jx(systemSize, systemSize);
    matX Jv(systemSize, systemSize);
    matX M(systemSize, systemSize); 

    //Obtaining mass matrix
    //Debug: Correct
    for(int i = 0; i < systemSize; i++){
        for(int j = 0; j < systemSize; j++){
            if(i == j){
                //particle mass * identity matrix
                M.coeffRef(i,j) = particle_list[i]->m * mat3::Identity();
            }else{
                //Zero matrix of size 3 x 3
                M.coeffRef(i,j) = mat3::Zero();
            }
        }
    }

    //Obtaining velocity vector
    //Debug: Correct
    for(int i = 0; i < systemSize; i++){
        v_n(i) = particle_list[i]->velocity;
    }

    //Force calculation
    //Obtaining force vector 
    this->resetForce(); 
    for(int i = 0; i < systemSize; i++){
        f_n(i) = vec3(0,0,0);
        for(int j = 0; j < systemSize; j++){
            Jx(i,j) = mat3::Zero();
            Jv(i,j) = mat3::Zero();
        }
    }
    //Contributions from ghost springs
    for(unsigned int i=0;i<ghostSprings.size();i++){
        calculateForce(ghostSprings[i], f_n, Jx, Jv);
    }
    //Contributions from springs aligned with the mesh
    for(unsigned int i=0;i<edge_list.size();i++){
        //Assign particles to the spring 
        edge_list[i]->spring.p1 = edge_list[i]->startParticle;
        edge_list[i]->spring.p2 = edge_list[i]->twin->startParticle;
        calculateForce(edge_list[i]->spring, f_n, Jx, Jv);
    }
    //Contributions from external forces
    for(unsigned int i = 0; i< particle_list.size(); i++){
        vec3 f_ext = particle_list[i]->calculateExternalForce();
        f_n(i) += f_ext;
        particle_list[i]->netForce += f_ext;
    }

    //Setup of the linear system
    //System Matrix(Dense version)
    matX A = M - (dt * mat3::Identity()) * Jv - (dt * dt * mat3::Identity()) * Jx;
    //RHS of the equation
    vecX b = scalarMult(f_n + scalarMult(matVecMult(Jx, v_n),dt),dt);

    // //Check symmetry of A
    // for(int i = 0; i < systemSize; i++){
    //     for(int j = 0; j < systemSize; j++){
    //         if(A(i,j) != A(j,i).transpose()){
    //             debugStream << "A is not symmetric!" << endl;
    //             debugStream << firstVertexIdx << " " << i << " " << j << endl;
    //             debugStream << "\n\n\n";
    //         }
    //     }
    // }

    //Handling constraints
    int numConstraints = 0;
    for(int i = 0; i < systemSize; i++){
        if(particle_list[i]->constraint.isActive){
            int idx = i - numConstraints;
            // debugStream << "i: " << i << ", idx:" << idx << ", numConstraints: " << numConstraints << endl;
            //Delete the row and column in A and b, corresponding to the constraint
            removeColumn(A, idx);
            removeRow(A, idx);
            removeRow(b, idx);
            numConstraints++;
        }
    }
    // debugStream << "A: " << A.rows() << ", " << A.cols() << endl;
    // debugStream << "b: " << b.rows() << endl;
    // debugStream << "\n\n" << endl;

    //Currently, the entire system is in block form
    //Need to expand it in all dimensions and convert in into sparse form before passing it to the solver
    matXf A_exploded = explodeMatrix(A);
    vecXf b_exploded = explodeVector(b);

    //Converting the exploded matrix into sparse form
    SparseMatrix<float> A_sparse = A_exploded.sparseView();

    //Define the solver and the solution vector
    SimplicialLDLT<SparseMatrix<float>> solver;
    // SparseLU<SparseMatrix<float>> solver;
    // ConjugateGradient<SparseMatrix<float>> solver;
    solver.compute(A_sparse);
    if(solver.info() != Eigen::Success){
        cout << "Eigen factorization failed!" << endl; 
    }
    vecXf dv_exploded(3 * (systemSize - numConstraints));
    dv_exploded = solver.solve(b_exploded);
    if(solver.info() != Eigen::Success){
        cout << "Eigen solve failed!" << endl; 
    }
    vecX dv(systemSize - numConstraints);
    dv = compressVector(dv_exploded);   

    //Velocity update
    numConstraints = 0;
    for(int i = 0; i < particle_list.size(); i++){
        if(particle_list[i]->constraint.isActive){
            particle_list[i]->velocity = particle_list[i]->constraint.velocity;
            numConstraints++;
        }else{
            particle_list[i]->velocity += dv(i - numConstraints);
        }
    }
}

void HalfEdge::updateMesh(float dt, TimeIntegrationType integrationType){
    if(integrationType == FWD_EULER){
        this->solveFwdEuler(dt);
    }else if(integrationType == BWD_EULER){
        this->solveBwdEuler(dt);
    }

    //3) Update particle positions
    for(unsigned int i=0;i<particle_list.size();i++){
        particle_list[i]->updatePos(dt);
    }
}

void HalfEdge::updateGhostSprings(){
    ghostSprings.clear(); 
    for(unsigned int i=0;i<edge_list.size();i++){
        Edge* currentEdge = edge_list[i];
        if(currentEdge->face != NULL && currentEdge->twin->face != NULL){
            Particle* p1 = currentEdge->prev->startParticle;
            Particle* p2 = currentEdge->twin->prev->startParticle;
            ghostSprings.push_back(Spring(p1, p2));
        }
    }
}

void HalfEdge::redistributeMass(){
    for(unsigned int i=0;i<particle_list.size();i++){
        particle_list[i]->m = 0.0f;
    }
    for(unsigned int i=0;i<face_list.size();i++){
        Face* f = face_list[i];
        Particle* p1 = particle_list[f->indices[0]];
        Particle* p2 = particle_list[f->indices[1]];
        Particle* p3 = particle_list[f->indices[2]];
        double newMass = (DEFAULT_DENISTY * triArea(p1->initPos, p2->initPos, p3->initPos))/3.0f;
        p1->m += newMass;
        p2->m += newMass;
        p3->m += newMass;
    }
    for(unsigned int i=0;i<particle_list.size();i++){
        particle_list[i]->updateInvM();
    }
}

//Pos to InitPos
vec3 HalfEdge::getInitPosAtPoint(Particle* p){
    return p->initPos;
}

vec3 HalfEdge::getInitPosAtEdge(Edge* e, vec3 pos){
    vec3 ip1 = e->startParticle->initPos;
    vec3 ip2 = e->twin->startParticle->initPos;
    vec3 p1 = e->startParticle->position;
    vec3 p2 = e->twin->startParticle->position;

    //Linear interpolation in the deformed grid
    double t = (pos - p1).norm() / (p2 - p1).norm(); 
    return (1 - t) * ip1 + t * ip2;
}

vec3 HalfEdge::getInitPosAtFace(Face* f, vec3 pos){
    Particle* a = particle_list[f->indices[0]];
    Particle* b = particle_list[f->indices[1]];
    Particle* c = particle_list[f->indices[2]];
    vec3 ip1 = a->initPos;
    vec3 ip2 = b->initPos;
    vec3 ip3 = c->initPos;
    vec3 p1 = a->position;
    vec3 p2 = b->position;
    vec3 p3 = c->position;

    //Barycentric interpolation in the deformed grid
    double A = triArea(p1, p2, p3);
    double Au = triArea(pos, p2, p3);
    double Av = triArea(p1, pos, p3);
    double Aw = triArea(p1, p2, pos);
    double u = Au / A;
    double v = Av / A;
    double w = Aw / A;

    return u * ip1 + v * ip2 + w * ip3;
}

//InitPos to Pos
vec3 HalfEdge::getPosAtPoint(Particle* p){
    return p->position;
}

vec3 HalfEdge::getPosAtEdge(Edge* e, vec3 pos){
    vec3 ip1 = e->startParticle->initPos;
    vec3 ip2 = e->twin->startParticle->initPos;
    vec3 p1 = e->startParticle->position;
    vec3 p2 = e->twin->startParticle->position;

    //Linear interpolation in the rest grid
    double t = (pos - ip1).norm() / (ip2 - ip1).norm(); 
    return (1 - t) * p1 + t * p2;
}

vec3 HalfEdge::getPosAtFace(Face* f, vec3 pos){
    Particle* a = particle_list[f->indices[0]];
    Particle* b = particle_list[f->indices[1]];
    Particle* c = particle_list[f->indices[2]];
    vec3 ip1 = a->initPos;
    vec3 ip2 = b->initPos;
    vec3 ip3 = c->initPos;
    vec3 p1 = a->position;
    vec3 p2 = b->position;
    vec3 p3 = c->position;

    //Barycentric interpolation in the rest grid
    double A = triArea(ip1, ip2, ip3);
    double Au = triArea(pos, ip2, ip3);
    double Av = triArea(ip1, pos, ip3);
    double Aw = triArea(ip1, ip2, pos);
    double u = Au / A;
    double v = Av / A;
    double w = Aw / A;

    return u * p1 + v * p2 + w * p3;
}

//Pos to Vel
vec3 HalfEdge::getVelAtPos(Particle* p){
    return p->velocity;
}

vec3 HalfEdge::getVelAtEdge(Edge* e, vec3 pos){
    vec3 v1 = e->startParticle->velocity;
    vec3 v2 = e->twin->startParticle->velocity;
    vec3 p1 = e->startParticle->position;
    vec3 p2 = e->twin->startParticle->position;

    //Linear interpolation in the deformed grid
    double t = (pos - p1).norm() / (p2 - p1).norm();
    return (1 - t) * v1 + t * v2;
}

vec3 HalfEdge::getVelAtFace(Face* f, vec3 pos){
    Particle* a = particle_list[f->indices[0]];
    Particle* b = particle_list[f->indices[1]];
    Particle* c = particle_list[f->indices[2]];
    vec3 p1 = a->position;
    vec3 p2 = b->position;
    vec3 p3 = c->position;
    vec3 v1 = a->velocity;
    vec3 v2 = b->velocity;
    vec3 v3 = c->velocity;

    //Barycentric interpolation in the deformed grid
    double A = triArea(p1, p2, p3);
    double Au = triArea(pos, p2, p3);
    double Av = triArea(p1, pos, p3);
    double Aw = triArea(p1, p2, pos);
    double u = Au / A;
    double v = Av / A;
    double w = Aw / A;

    return u * v1 + v * v2 + w * v3;
}