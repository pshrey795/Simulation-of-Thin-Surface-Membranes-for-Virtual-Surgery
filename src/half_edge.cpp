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
        newParticle->particleID = i;
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
        struct Face* f = new Face(a,b,c,false);
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
        this->face_list.push_back(f);

        //Adding half edges
        this->edge_list.push_back(e1);
        this->edge_list.push_back(e2);
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

    updateGhostSprings();
    redistributeMass();
}

//Constructor of Half Edge data structure from a list of vertices and faces
HalfEdge::HalfEdge(vector<vec3> Vertices, vector<unsigned int> Indices, vector<bool> clamp){
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
        newParticle->isFixed = clamp[i];
        newParticle->particleID = i;
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
        struct Face* f = new Face(a,b,c,false);
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
        this->face_list.push_back(f);

        //Adding half edges
        this->edge_list.push_back(e1);
        this->edge_list.push_back(e2);
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
    updateGhostSprings();
    redistributeMass();
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

//Calculating offsets for each Particle from the plane
void HalfEdge::ParticleOffsets(Plane plane){
    for(unsigned int i=0;i<particle_list.size();i++){
        double newOffset = plane.normal.dot(particle_list[i]->initPos - plane.origin);
        particle_list[i]->offset = newOffset;
    }
}

//Obtaining the edges that intersect with the plane
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
double HalfEdge::triArea(vec3 a, vec3 b, vec3 c){
    vec3 x = b - a;
    vec3 y = c - a;
    return abs(0.5 * (x.cross(y)).norm());
}
bool HalfEdge::isInside(Face* face, vec3 point){
    vec3 a = this->particle_list[face->indices[0]]->initPos;
    vec3 b = this->particle_list[face->indices[1]]->initPos;
    vec3 c = this->particle_list[face->indices[2]]->initPos;
    double A = this->triArea(a,b,c);
    double A1 = this->triArea(point,b,c);
    double A2 = this->triArea(point,c,a);
    double A3 = this->triArea(point,a,b);
    return (abs((A1 + A2 + A3) - A) < MIN_DIFF);
}
bool HalfEdge::isInsidePos(Face* face, vec3 point){
    vec3 a = this->particle_list[face->indices[0]]->position;
    vec3 b = this->particle_list[face->indices[1]]->position;
    vec3 c = this->particle_list[face->indices[2]]->position;
    double A = this->triArea(a,b,c);
    double A1 = this->triArea(point,b,c);
    double A2 = this->triArea(point,c,a);
    double A3 = this->triArea(point,a,b);
    return (abs((A1 + A2 + A3) - A) < MIN_DIFF);
}

void HalfEdge::reMesh(tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> intPt, tuple<vec3, int, int> &nextIntPt, Edge* &leftCrossEdge, Edge* &rightCrossEdge, vec3 normal, int splitMode){
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
    vec3 displacement = (1?(splitMode == 0):0) * EPSILON * normal;
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
            newParticleRight->particleID = newIndexRight;
            this->particle_list.push_back(newParticleRight);
        }else if(currentType == 1){
            vec3 newInitPos = get<0>(intPt);
            vec3 oldPos = getPosAtEdge(this->edge_list[get<2>(intPt)], newInitPos);
            vec3 newVelocity = getVelAtEdge(this->edge_list[get<2>(intPt)], oldPos);
            newParticleLeft = new Particle(oldPos - displacement, newInitPos, newVelocity);
            newParticleRight = new Particle(oldPos + displacement, newInitPos, newVelocity);
            newIndexLeft = n;
            newIndexRight = n+1;
            newParticleLeft->particleID = newIndexLeft;
            newParticleRight->particleID = newIndexRight;
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
            newParticle->particleID = newIndexLeft;
            this->particle_list.push_back(newParticle);
        }
    }else if(last){
        if(currentType != 2){
            newParticleLeft = this->particle_list[get<2>(intPt)];
            newIndexLeft = get<2>(intPt);
            vec3 oldPos = newParticleLeft->position;
            vec3 newInitPos = getInitPosAtPoint(newParticleLeft);
            vec3 newVelocity = getVelAtPos(newParticleLeft);
            newParticleLeft->position = oldPos - displacement;
            newParticleRight = new Particle(oldPos + displacement, newInitPos, newVelocity);
            newIndexRight = n;
            newParticleRight->particleID = newIndexRight;
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
        newParticleRight->particleID = newIndexRight;
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
                nextParticle->particleID = nextIndex;
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
                nextParticle->particleID = nextIndex;
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
                nextParticle->particleID = nextIndex;
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
                newParticleLeft->edge = newEdge1;
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
        if(currentType != 2){
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
            nextParticle->particleID = nextIndex;
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
            nextParticle->particleID = nextIndex;
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

    updateGhostSprings();
    redistributeMass();
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
        if(!particle_list[i]->isFixed){
            particle_list[i]->velocity += particle_list[i]->netForce * dt * particle_list[i]->invM;
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
    //Debugging Info:
    //1) Force calculation is correct
    //2) Jx pending
    //3) Jv pending
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
        //Debugging
        // cout << (f_n(i)-particle_list[i]->netForce).norm() << endl;
    }

    //Setup of the linear system
    //System Matrix(Dense version)
    matX A = M - (dt * mat3::Identity()) * Jv - (dt * dt * mat3::Identity()) * Jx;
    //RHS of the equation
    vecX b = scalarMult(f_n + scalarMult(matVecMult(Jx, v_n),dt),dt);

    //Currently, the entire system is in block form
    //Need to expand it in all dimensions and convert in into sparse form before passing it to the solver
    matXf A_exploded = explodeMatrix(A);
    vecXf b_exploded = explodeVector(b);

    //Converting the exploded matrix into sparse form
    SparseMatrix<float> A_sparse = A_exploded.sparseView();

    //Define the solver and the solution vector
    SimplicialLDLT<SparseMatrix<float>> solver;
    solver.compute(A_sparse);
    if(solver.info() != Eigen::Success){
        cout << "Eigen factorization failed!" << endl; 
    }
    vecXf dv_exploded(3 * systemSize);
    dv_exploded = solver.solve(b_exploded);
    if(solver.info() != Eigen::Success){
        cout << "Eigen solve failed!" << endl; 
    }
    vecX dv(systemSize);
    dv = compressVector(dv_exploded);
    v_n += dv;

    //Velocity update
    for(unsigned int i = 0; i < systemSize; i++){
        if(!particle_list[i]->isFixed){
            particle_list[i]->velocity = v_n(i);
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


