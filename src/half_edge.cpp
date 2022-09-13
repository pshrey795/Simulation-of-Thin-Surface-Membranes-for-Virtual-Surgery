#include "../include/half_edge.hpp"

//Constructor of Half Edge data structure from a list of vertices and faces
HalfEdge::HalfEdge(vector<vec3> Vertices, vector<unsigned int> Indices){
    int n = Vertices.size();

    for(int i=0;i<n;i++){
        vector<int> adj;
        for(int j=0;j<n;j++){
            adj.push_back(-1);
        }
        this->adjList.push_back(adj); 
    }

    //Adding vertices
    for(int i=0;i<Vertices.size();i++){
        this->particle_list.push_back(new Particle(Vertices[i]));
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
    for(int a=0;a<n;a++){
        for(int b=0;b<n;b++){
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
}

//Constructor of Half Edge data structure from a list of vertices and faces
HalfEdge::HalfEdge(vector<vec3> Vertices, vector<unsigned int> Indices, vector<bool> clamp){
    int n = Vertices.size();

    for(int i=0;i<n;i++){
        vector<int> adj;
        for(int j=0;j<n;j++){
            adj.push_back(-1);
        }
        this->adjList.push_back(adj); 
    }

    //Adding vertices
    for(int i=0;i<Vertices.size();i++){
        this->particle_list.push_back(new Particle(Vertices[i]));
        this->particle_list[i]->isFixed = clamp[i];
    }

    int i=0;
    while(i < Indices.size()){
        int a,b,c;

        //Particle Indices
        a = Indices[i];
        b = Indices[i+1];
        c = Indices[i+2];
        double lenAB = (particle_list[a]->position - particle_list[b]->position).norm();
        double lenBC = (particle_list[b]->position - particle_list[c]->position).norm();
        double lenCA = (particle_list[c]->position - particle_list[a]->position).norm();

        //Create Face and Half Edges
        struct Face* f = new Face(a,b,c,false);
        struct Edge* e1 = new Edge(particle_list[a],f,lenAB);
        struct Edge* e2 = new Edge(particle_list[b],f,lenBC);
        struct Edge* e3 = new Edge(particle_list[c],f,lenCA);

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
    for(int a=0;a<n;a++){
        for(int b=0;b<n;b++){
            if(adjList[a][b] == -1){
                if(adjList[b][a] != -1){
                    auto v = this->edge_list[adjList[b][a]]->next->startParticle;
                    auto u = this->edge_list[adjList[b][a]]->startParticle;
                    double dist = (v->position - u->position).norm();
                    struct Edge* newEdge = new Edge(v,NULL,dist);
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
}

//Intersection with a plane
vector<tuple<vec3,int,int>> HalfEdge::Intersect(Plane plane){
    ParticleOffsets(plane);
    auto intersectingEdges = IntersectingEdges();
    return IntersectingVertices(intersectingEdges);
}

//Calculating offsets for each Particle from the plane
void HalfEdge::ParticleOffsets(Plane plane){
    for(int i=0;i<particle_list.size();i++){
        double newOffset = plane.normal.dot(particle_list[i]->position - plane.origin);
        particle_list[i]->offset = newOffset;
    }
}

//Obtaining the edges that intersect with the plane
vector<int> HalfEdge::IntersectingEdges(){
    vector<int> intersectingEdges;
    vector<int> isEdgeIntersecting;
    for(int i=0;i<this->edge_list.size();i++){
        isEdgeIntersecting.push_back(0);
    }
    for(int i=0;i<edge_list.size();i++){
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
    for(int i=0;i<particle_list.size();i++){
        if(abs(particle_list[i]->offset) < DELTA){
            intersectingVertices.push_back(make_tuple(particle_list[i]->position,0,i));
        }
    }
    for(auto idx : edges){
        double offset1 = edge_list[idx]->startParticle->offset;
        double offset2 = edge_list[idx]->twin->startParticle->offset;
        double factor = offset1 / (offset1 - offset2);
        vec3 edgeStart = edge_list[idx]->startParticle->position;
        vec3 edgeEnd = edge_list[idx]->twin->startParticle->position;
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
    vec3 a = this->particle_list[face->indices[0]]->position;
    vec3 b = this->particle_list[face->indices[1]]->position;
    vec3 c = this->particle_list[face->indices[2]]->position;
    double A = this->triArea(a,b,c);
    double A1 = this->triArea(point,b,c);
    double A2 = this->triArea(point,a,c);
    double A3 = this->triArea(point,a,b);
    return (abs((A1 + A2 + A3) - A) < DELTA);
}

//Remeshing 
void HalfEdge::reMesh(tuple<vec3, int, int> intPt, tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> nextIntPt, Particle* &lastParticle, Edge* &leftCrossEdge, Edge* &rightCrossEdge, Edge* &leftSideEdge, Edge* &rightSideEdge, vec3 normal){
    //Auxiliary variables
    int currentType = get<1>(intPt);
    int lastType = get<1>(lastIntPt);
    int nextType = get<1>(nextIntPt);
    bool first = (lastType == -1);
    bool last = (nextType == -1);
    int n = this->particle_list.size();

    //New mesh entities 
    Particle* newParticle;
    Particle* newParticleLeft;
    Particle* newParticleRight;
    Edge* newCrossEdgeLeft;
    Edge* newCrossEdgeRight;
    Edge* newSideEdgeLeft;
    Edge* newSideEdgeRight;
    
    //Creating new vertices and edges or using an existing Particle depending on the type of intersection point
    if(currentType == 0){
        //Intersection point is already a Particle of the mesh
        newParticleLeft = this->particle_list[get<2>(intPt)];
        vec3 oldPos = newParticleLeft->position;
        newParticleLeft->position = oldPos - normal * EPSILON;
        newParticleRight = new Particle(oldPos + normal * EPSILON);
        this->particle_list.push_back(newParticleRight);
    }else if(currentType == 1){
        //Intersection point is on an edge of the mesh
        if(first){
            vec3 oldPos = get<0>(intPt);
            newParticleLeft = new Particle(oldPos - normal * EPSILON);
            newParticleRight = new Particle(oldPos + normal * EPSILON);
            this->particle_list.push_back(newParticleLeft);
        }else{
            newParticleLeft = lastParticle;
            vec3 oldPos = lastParticle->position;
            newParticleLeft->position = oldPos - normal * EPSILON;
            newParticleRight = new Particle(oldPos + normal * EPSILON);
        }
        this->particle_list.push_back(newParticleRight);
    }else{
        //Intersection point is on a face of the mesh(in case of a tear)
        //Only case in which the Particle doesn't split
        //Such type of vertices are allotted only to the start and end points of a tear
        if(first || last){
            newParticleLeft  = newParticleRight = this->particle_list[get<2>(intPt)];
        }else{
            newParticleLeft = lastParticle;
            vec3 oldPos = lastParticle->position;
            newParticleLeft->position = oldPos - normal * EPSILON;
            newParticleRight = new Particle(oldPos + normal * EPSILON);
        }
        this->particle_list.push_back(newParticleRight);
    }

    //Adding the new vertices to the mesh
    int newIndexLeft;
    int newIndexRight; 
    if(currentType == 0){
        //Only one new Particle added
        newIndexLeft = get<2>(intPt);
        newIndexRight = n;
    }else if(currentType == 1){
        //Two new vertices added
        if(first){
            newIndexLeft = n;
            newIndexRight = n + 1;
        }else{
            newIndexLeft = find(this->particle_list.begin(),this->particle_list.end(),lastParticle) - this->particle_list.begin();
            newIndexRight = n;
        }
    }else{
        //Only one Particle
        if(first || last){
            newIndexLeft = newIndexRight = get<2>(intPt);
        }else{
            newIndexLeft = find(this->particle_list.begin(),this->particle_list.end(),lastParticle) - this->particle_list.begin();
            newIndexRight = n;
        }
    }

    //Complete Restructuring of data structure based on cases
    if(first){
        if(currentType == 0){
            
            //Depending on the type of the next Particle
            if(nextType == 0){
                //Current: Particle, Next: Particle
                Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                Edge* currentEdge = newParticleLeft->edge;

                //Finding the edge between the current and the next 
                //intersection point 
                while(true){
                    if(currentEdge->twin->startParticle == nextParticle){
                        break;
                    }else if(currentEdge->twin->next == NULL){
                        break;
                    }else{
                        currentEdge = currentEdge->twin->next;
                    }
                }

                while(currentEdge->twin->startParticle != nextParticle){
                    currentEdge = currentEdge->prev->twin;
                }

                //Edge to Edge relations
                Edge* newEdge = new Edge();
                newCrossEdgeLeft = currentEdge->twin;
                newCrossEdgeRight = new Edge();
                newCrossEdgeLeft->twin = newEdge;
                newEdge->twin = newCrossEdgeLeft;
                newCrossEdgeRight->twin = currentEdge;
                currentEdge->twin = newCrossEdgeRight;

                //Edge to Particle relations
                newEdge->startParticle = newParticleLeft;
                newCrossEdgeRight->startParticle = nextParticle;
                currentEdge->startParticle = newParticleRight;
                currentEdge->prev->twin->startParticle = newParticleRight;

                //Particle to Edge relations
                newParticleLeft->edge = newEdge;
                newParticleRight->edge = currentEdge;

                //Face to Particle relations
                Face* rightFace = currentEdge->face;
                for(int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                        break;
                    }
                }

                this->edge_list.push_back(newEdge);
                this->edge_list.push_back(newCrossEdgeRight);
            }else if(nextType == 1){
                //Current: Particle, Next: Edge
                newParticle = new Particle(get<0>(nextIntPt));
                this->particle_list.push_back(newParticle);
                int newIndex = this->particle_list.size() - 1;

                Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                if(currentEdge->prev == NULL){
                    currentEdge = currentEdge->twin->prev;
                }else{
                    if(currentEdge->prev->startParticle == newParticleLeft){
                        currentEdge = currentEdge->prev;
                    }else{
                        currentEdge = currentEdge->twin->prev;
                    }
                }
                //Now, currentEdge is the edge starting from the the current 
                //intersection point and adjacent to the cut

                //Edge to Edge relations
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();
                newSideEdgeLeft = currentEdge->next->twin;
                newSideEdgeRight = new Edge();

                //Twin Edges
                newCrossEdgeLeft->twin = newEdge1;
                newEdge1->twin = newCrossEdgeLeft;
                newCrossEdgeRight->twin = newEdge2;
                newEdge2->twin = newCrossEdgeRight;
                newSideEdgeLeft->twin = currentEdge->next;
                currentEdge->next->twin = newSideEdgeLeft;
                newSideEdgeRight->twin = newEdge3;
                newEdge3->twin = newSideEdgeRight;

                //Next/Prev Edges
                newCrossEdgeLeft->next = currentEdge;
                newCrossEdgeLeft->prev = currentEdge->next;
                newEdge2->next = newEdge3;
                newEdge2->prev = currentEdge->prev;
                newEdge3->next = currentEdge->prev;
                newEdge3->prev = newEdge2;
                currentEdge->prev->next = newEdge2;
                currentEdge->prev->prev = newEdge3;
                currentEdge->prev = newCrossEdgeLeft;
                currentEdge->next->next = newCrossEdgeLeft;

                //Edge to Particle relations
                newCrossEdgeLeft->startParticle = newParticle;
                newCrossEdgeRight->startParticle = newParticle;
                newEdge1->startParticle = newParticleLeft;
                newEdge2->startParticle = newParticleRight;
                newEdge3->startParticle = newParticle;
                newSideEdgeLeft->startParticle = newParticle;
                newSideEdgeRight->startParticle = newEdge3->next->startParticle;
                newEdge3->next->twin->startParticle = newParticleRight;

                //Particle to Edge relations
                newParticle->edge = newCrossEdgeLeft;
                newParticleLeft->edge = currentEdge;
                newParticleRight->edge = newEdge2;

                //Obtaining old Particle indices
                int oldIndexLeft, oldIndexRight, centreIndex;
                Face* oldFace = currentEdge->face;
                if(oldFace->edge == currentEdge){
                    centreIndex = oldFace->indices[0];
                    oldIndexLeft = oldFace->indices[1];
                    oldIndexRight = oldFace->indices[2];
                }else if(oldFace->edge->next == currentEdge){
                    centreIndex = oldFace->indices[1];
                    oldIndexLeft = oldFace->indices[2];
                    oldIndexRight = oldFace->indices[0];
                }else{
                    centreIndex = oldFace->indices[2];
                    oldIndexLeft = oldFace->indices[0];
                    oldIndexRight = oldFace->indices[1];
                }

                //Face to Particle/Edge relations
                oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                oldFace->edge = currentEdge;
                Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                newFace->edge = newEdge2;

                //Edge to Face relations
                newCrossEdgeLeft->face = oldFace;
                newEdge2->face = newFace;
                newEdge3->face = newFace;
                newEdge3->next->face = newFace;

                //Adding the edges/faces
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newCrossEdgeLeft);
                this->edge_list.push_back(newCrossEdgeRight);
                this->edge_list.push_back(newSideEdgeRight);
                this->face_list.push_back(newFace);
            }else{

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

            //Depending on the type of the next Particle
            if(nextType == 0){
                //Current: Edge, Next: Particle

                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();

                newCrossEdgeLeft->startParticle = this->particle_list[oppIndex];
                newCrossEdgeRight->startParticle = this->particle_list[oppIndex]; 

                //Edge to Edge relations
                //Two new edges on the intersecting edge
                //Four on the path from this intersection point to the next
                //Two of the second type are to be passed onto the next remeshing iteration
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();

                //For new edges
                newEdge1->twin = newEdge2;
                newEdge2->twin = newEdge1;
                newEdge3->twin = newCrossEdgeLeft;
                newCrossEdgeLeft->twin = newEdge3;
                newEdge4->twin = newCrossEdgeRight;
                newCrossEdgeRight->twin = newEdge4;

                newEdge1->prev = intersectingEdge->prev;
                newEdge1->next = newEdge4;
                newEdge4->next = intersectingEdge->prev;
                newEdge4->prev = newEdge1;
                newCrossEdgeLeft->next = intersectingEdge;
                newCrossEdgeLeft->prev = intersectingEdge->next;

                //For old edges
                intersectingEdge->next->next = newCrossEdgeLeft;
                intersectingEdge->prev->next = newEdge1;
                intersectingEdge->prev->prev = newEdge4;
                intersectingEdge->prev = newCrossEdgeLeft; 

                //Edge to Particle relations
                newEdge1->startParticle = this->particle_list[oldIndexRight];
                newEdge2->startParticle = newParticleRight;
                newEdge3->startParticle = newParticleLeft;
                newEdge4->startParticle = newParticleRight;

                //Particle to Edge relations
                newParticleLeft->edge = newEdge3;
                newParticleRight->edge = newEdge4;

                //Face to Particle/Edge relations
                Face* newFace = new Face(oppIndex,oldIndexRight,newIndexRight);
                newFace->edge = newEdge4->next;
                intersectingFace->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                intersectingFace->edge = newCrossEdgeLeft;

                //Edge to Face relations
                newEdge1->face = newFace;
                newEdge1->next->face = newFace;
                newEdge1->next->next->face = newFace;
                newCrossEdgeLeft->face = intersectingFace;

                //Adding the new edges and faces
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newCrossEdgeLeft);
                this->edge_list.push_back(newCrossEdgeRight);
                this->face_list.push_back(newFace);
            }else if(nextType == 1){
                //Current:Edge, Next:Edge
                newParticle = new Particle(get<0>(nextIntPt));
                this->particle_list.push_back(newParticle);
                int newIndex = this->particle_list.size()-1;

                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();
                Edge* newEdge7 = new Edge();
                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();
                newSideEdgeRight = new Edge();
                newSideEdgeLeft = this->edge_list[get<2>(nextIntPt)];
                if(newSideEdgeLeft->face == intersectingFace){
                    newSideEdgeLeft = newSideEdgeLeft->twin;
                }
                bool left = (newSideEdgeLeft->twin == intersectingEdge->next);

                if(left){
                    //Edge to edge relations
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge4;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;
                    newSideEdgeRight->twin = newEdge7;
                    newEdge7->twin = newSideEdgeRight;

                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = intersectingEdge->next;
                    newEdge2->next = newEdge5;
                    newEdge2->prev = intersectingEdge->prev;
                    newEdge3->next = newEdge7;
                    newEdge3->prev = newEdge6;
                    newEdge5->next = intersectingEdge->prev;
                    newEdge5->prev = newEdge2;
                    newEdge6->next = newEdge3;
                    newEdge6->prev = newEdge7;
                    newEdge7->next = newEdge6;
                    newEdge7->prev = newEdge3;
                    intersectingEdge->prev->next = newEdge2;
                    intersectingEdge->prev->prev = newEdge5;
                    intersectingEdge->prev = newCrossEdgeLeft;
                    intersectingEdge->next->next = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newEdge1->startParticle = newParticleRight;
                    newEdge2->startParticle = this->particle_list[oldIndexRight];
                    newEdge3->startParticle = newParticleRight;
                    newEdge4->startParticle = newParticleLeft;
                    newEdge5->startParticle = newParticleRight;
                    newEdge6->startParticle = this->particle_list[oppIndex];
                    newEdge7->startParticle = newParticle;
                    newSideEdgeRight->startParticle = this->particle_list[oppIndex];
                    newSideEdgeLeft->startParticle = newParticle;
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;

                    //Particle to edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleLeft->edge = intersectingEdge;
                    newParticleRight->edge = newEdge5;

                    //Face to Particle/Edge relations
                    Face* newFaceTop = new Face(newIndexRight,oppIndex,oldIndexRight);
                    newFaceTop->edge = newEdge5;
                    Face* newFaceBot = new Face(newIndexRight,newIndex,oppIndex);
                    newFaceBot->edge = newEdge3;
                    intersectingFace->setFace(oldIndexLeft, newIndex, newIndexLeft);
                    intersectingFace->edge = intersectingEdge->next;

                    //Edge to Face relations
                    newEdge2->face = newFaceTop;
                    newEdge3->face = newFaceBot;
                    newEdge5->face = newFaceTop;
                    newEdge6->face = newFaceBot;
                    newEdge7->face = newFaceBot;
                    newCrossEdgeLeft->face = intersectingFace;
                    newEdge5->next->face = newFaceTop;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);   
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFaceTop);
                    this->face_list.push_back(newFaceBot);
                }else{
                    //Edge to edge relations
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge4;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;
                    newSideEdgeRight->twin = newEdge7;
                    newEdge7->twin = newSideEdgeRight;

                    newCrossEdgeLeft->next = newEdge6;
                    newCrossEdgeLeft->prev = intersectingEdge->prev;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = newEdge7;
                    newEdge3->next = newEdge7;
                    newEdge3->prev = newEdge2;
                    newEdge5->next = intersectingEdge;
                    newEdge5->prev = intersectingEdge->next;
                    newEdge6->next = intersectingEdge->prev;
                    newEdge6->prev = newCrossEdgeLeft;
                    newEdge7->next = newEdge2;
                    newEdge7->prev = newEdge3;
                    intersectingEdge->prev->next = newCrossEdgeLeft;
                    intersectingEdge->prev->prev = newEdge6;
                    intersectingEdge->prev = newEdge5;
                    intersectingEdge->next->next = newEdge5;

                    //Edge to Particle relations
                    newEdge1->startParticle = newParticleRight;
                    newEdge2->startParticle = this->particle_list[oldIndexRight];
                    newEdge3->startParticle = newParticleRight;
                    newEdge4->startParticle = newParticleLeft;
                    newEdge5->startParticle = this->particle_list[oppIndex];
                    newEdge6->startParticle = newParticleLeft;
                    newEdge7->startParticle = newParticle;
                    newSideEdgeRight->startParticle = this->particle_list[oldIndexRight];
                    newSideEdgeLeft->startParticle = newParticle;
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;

                    //Particle to edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleLeft->edge = intersectingEdge;
                    newParticleRight->edge = newEdge3;

                    //Face to Particle/Edge relations
                    Face* newFaceLeft = new Face(newIndexLeft,oppIndex,newIndex);
                    newFaceLeft->edge = newEdge6;
                    Face* newFaceRight = new Face(newIndexRight,newIndex,oldIndexRight);
                    newFaceRight->edge = newEdge3;
                    intersectingFace->setFace(oldIndexLeft, oppIndex, newIndexLeft);
                    intersectingFace->edge = intersectingEdge->next;

                    //Edge to Face relations
                    newEdge2->face = newFaceRight;
                    newEdge3->face = newFaceRight;
                    newEdge5->face = intersectingFace;
                    newEdge6->face = newFaceLeft;
                    newEdge7->face = newFaceRight;
                    newCrossEdgeLeft->face = newFaceLeft;
                    newEdge6->next->face = newFaceLeft;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);   
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFaceLeft);
                    this->face_list.push_back(newFaceRight);
                }
            }else{

            }
        }else{
            //Determining the face containing the first intersection point
            if(nextType == 0){
                //Current:Face, Next: Particle
                Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                Edge* currentEdge = nextParticle->edge;
                while(currentEdge->twin->next != NULL){
                    currentEdge = currentEdge->twin->next;
                }

                //Finding the face which contains the current intersection point
                while(true){
                    Face* currentFace = currentEdge->face;
                    if(isInside(currentFace, get<0>(intPt))){
                        break;
                    }else{
                        currentEdge = currentEdge->prev->twin;
                    }
                }

                //Edge to edge relations
                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();

                //Twin Edges
                newEdge1->twin = newCrossEdgeRight;
                newCrossEdgeRight->twin = newEdge1;
                newEdge2->twin = newCrossEdgeLeft;
                newCrossEdgeLeft->twin = newEdge2;
                newEdge3->twin = newEdge4;
                newEdge4->twin = newEdge3;
                newEdge5->twin = newEdge6;
                newEdge6->twin = newEdge5;

                //Next/Prev Edges
                newCrossEdgeLeft->next = newEdge6;
                newCrossEdgeLeft->prev = currentEdge->prev;
                newEdge1->next = currentEdge;
                newEdge1->prev = newEdge3;
                newEdge3->next = newEdge1;
                newEdge3->prev = currentEdge;
                newEdge4->next = currentEdge->next;
                newEdge4->prev = newEdge5;
                newEdge5->next = newEdge4;
                newEdge5->prev = currentEdge->next;
                newEdge6->next = currentEdge->prev;
                newEdge6->prev = newCrossEdgeLeft;
                currentEdge->prev->next = newCrossEdgeLeft;
                currentEdge->prev->prev = newEdge6;
                currentEdge->next->next = newEdge5;
                currentEdge->next->prev = newEdge4;
                currentEdge->prev = newEdge1;
                currentEdge->next = newEdge3;

                //Obtaining old Particle indices
                int oldIndexLeft, oldIndexRight, centreIndex;
                Face* oldFace = currentEdge->face;
                if(oldFace->edge == currentEdge){
                    centreIndex = oldFace->indices[0];
                    oldIndexRight = oldFace->indices[1];
                    oldIndexLeft = oldFace->indices[2];
                }else if(oldFace->edge->next == currentEdge){
                    centreIndex = oldFace->indices[1];
                    oldIndexRight = oldFace->indices[2];
                    oldIndexLeft = oldFace->indices[0];
                }else{
                    centreIndex = oldFace->indices[2];
                    oldIndexRight = oldFace->indices[0];
                    oldIndexLeft = oldFace->indices[1];
                }

                //Edge to Particle relations
                newEdge1->startParticle = newParticleLeft;
                newEdge2->startParticle = newParticleLeft;
                newEdge3->startParticle = this->particle_list[oldIndexRight];
                newEdge4->startParticle = newParticleLeft;
                newEdge5->startParticle = this->particle_list[oldIndexLeft];
                newEdge6->startParticle = newParticleLeft;
                newCrossEdgeLeft->startParticle = nextParticle;
                newCrossEdgeRight->startParticle = nextParticle;

                //Particle to edge relations
                newParticleLeft->edge = newEdge4;

                //Face to Particle/Edge relations
                Face* newFaceLeft = new Face(newIndexLeft,oldIndexLeft,centreIndex);
                newFaceLeft->edge = newEdge6;
                Face* newFaceRight = new Face(newIndexRight,centreIndex,oldIndexRight);
                newFaceRight->edge = newEdge1;
                oldFace->setFace(newIndexLeft, oldIndexRight, oldIndexLeft);
                oldFace->edge = newEdge4;

                //Edge to Face relations
                newEdge1->face = newFaceRight;
                newEdge3->face = newFaceRight;
                currentEdge->face = newFaceRight;
                newEdge4->face = oldFace;
                newEdge5->face = oldFace;
                newEdge4->next->face = oldFace;
                newEdge6->face = newFaceLeft;
                newCrossEdgeLeft->face = newFaceLeft;
                newEdge6->next->face = newFaceLeft;

                //Adding the edges/faces
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newEdge5);
                this->edge_list.push_back(newEdge6);
                this->edge_list.push_back(newCrossEdgeLeft);
                this->edge_list.push_back(newCrossEdgeRight);
                this->face_list.push_back(newFaceLeft);
                this->face_list.push_back(newFaceRight);
            }else if(nextType == 1){
                //Current:Face, Next: Edge
                Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                if(!isInside(currentEdge->face, get<0>(intPt))){
                    currentEdge = currentEdge->twin;
                }
                //Adding a new Particle on the opposite edge
                newParticle = new Particle(get<0>(nextIntPt));
                int newIndex = this->particle_list.size();
                this->particle_list.push_back(newParticle);

                //Edge to edge relations 
                newCrossEdgeLeft = new Edge();
                newCrossEdgeRight = new Edge();
                newSideEdgeRight = new Edge();
                newSideEdgeLeft = currentEdge->twin;
                Edge* newEdge1 = new Edge();
                Edge* newEdge2 = new Edge();
                Edge* newEdge3 = new Edge();
                Edge* newEdge4 = new Edge();
                Edge* newEdge5 = new Edge();
                Edge* newEdge6 = new Edge();
                Edge* newEdge7 = new Edge();
                Edge* newEdge8 = new Edge();
                Edge* newEdge9 = new Edge();

                //Twin Edges
                newEdge1->twin = newCrossEdgeLeft;
                newCrossEdgeLeft->twin = newEdge1;
                newEdge2->twin = newCrossEdgeRight;
                newCrossEdgeRight->twin = newEdge2;
                newEdge3->twin = newSideEdgeRight;
                newSideEdgeRight->twin = newEdge3;
                newEdge4->twin = newEdge5;
                newEdge5->twin = newEdge4;
                newEdge6->twin = newEdge7;
                newEdge7->twin = newEdge6;
                newEdge8->twin = newEdge9;
                newEdge9->twin = newEdge8;
                

                //Next/Prev Edges
                newCrossEdgeLeft->next = newEdge9;
                newCrossEdgeLeft->prev = currentEdge;
                newEdge2->next = newEdge3;
                newEdge2->prev = newEdge4;
                newEdge3->next = newEdge4;
                newEdge3->prev = newEdge2;
                newEdge4->next = newEdge2;
                newEdge4->prev = newEdge3;
                newEdge5->next = currentEdge->next;
                newEdge5->prev = newEdge6;
                newEdge6->next = newEdge5;
                newEdge6->prev = currentEdge->next;
                newEdge7->next = currentEdge->prev;
                newEdge7->prev = newEdge8;
                newEdge8->next = newEdge7;
                newEdge8->prev = currentEdge->prev;
                newEdge9->next = currentEdge;
                newEdge9->prev = newCrossEdgeLeft;
                currentEdge->next = newCrossEdgeLeft;
                currentEdge->prev = newEdge9;

                //Obtaining indices of the old face
                int oldIndexLeft, oldIndexRight, oppIndex;
                Face* oldFace = currentEdge->face;
                if(oldFace->edge == currentEdge){
                    oldIndexLeft = oldFace->indices[0];
                    oldIndexRight = oldFace->indices[1];
                    oppIndex = oldFace->indices[2];
                }else if(oldFace->edge->next == currentEdge){
                    oldIndexLeft = oldFace->indices[1];
                    oldIndexRight = oldFace->indices[2];
                    oppIndex = oldFace->indices[0];
                }else{
                    oldIndexLeft = oldFace->indices[2];
                    oldIndexRight = oldFace->indices[0];
                    oppIndex = oldFace->indices[1];
                }

                //Edge to Particle relations 
                newCrossEdgeLeft->startParticle = newParticle;
                newCrossEdgeRight->startParticle = newParticle;
                newSideEdgeRight->startParticle = this->particle_list[oldIndexRight];
                newEdge1->startParticle = newParticleLeft;
                newEdge2->startParticle = newParticleRight;
                newEdge3->startParticle = newParticle;
                newEdge4->startParticle = this->particle_list[oldIndexRight];
                newEdge5->startParticle = newParticleRight;
                newEdge6->startParticle = this->particle_list[oppIndex];
                newEdge7->startParticle = newParticleLeft;
                newEdge8->startParticle = this->particle_list[oldIndexLeft];
                newEdge9->startParticle = newParticleLeft;

                //Particle to Edge relations
                newParticleLeft->edge = newEdge5;
                newParticle->edge = newEdge3;

                //Face to Particle/Edge relations
                Face* newFaceBotLeft = new Face(oldIndexLeft,newIndex,newIndexLeft);
                newFaceBotLeft->edge = currentEdge;
                Face* newFaceBotRight = new Face(newIndex,oldIndexRight,newIndexRight);
                newFaceBotRight->edge = newEdge3;
                Face* newFaceTopRight = new Face(newIndexRight,oldIndexRight,oppIndex);
                newFaceTopRight->edge = newEdge5;
                Face* newFaceTopLeft = currentEdge->face;
                newFaceTopLeft->setFace(newIndexLeft,oppIndex,oldIndexLeft);
                newFaceTopLeft->edge = newEdge7;

                //Edge to Face relations
                newCrossEdgeLeft->face = newFaceBotLeft;
                currentEdge->face = newFaceBotLeft;
                newEdge9->face = newFaceBotLeft;
                newEdge2->face = newFaceBotRight;
                newEdge3->face = newFaceBotRight;
                newEdge4->face = newFaceBotRight;
                newEdge5->face = newFaceTopRight;
                newEdge6->face = newFaceTopRight;
                newEdge5->next->face = newFaceTopRight;
                newEdge7->face = newFaceTopLeft;
                newEdge8->face = newFaceTopLeft;
                newEdge7->next->face = newFaceTopLeft;

                //Adding the edges and faces
                this->edge_list.push_back(newCrossEdgeLeft);
                this->edge_list.push_back(newCrossEdgeRight);
                this->edge_list.push_back(newSideEdgeRight);
                this->edge_list.push_back(newEdge1);
                this->edge_list.push_back(newEdge2);
                this->edge_list.push_back(newEdge3);
                this->edge_list.push_back(newEdge4);
                this->edge_list.push_back(newEdge5);
                this->edge_list.push_back(newEdge6);
                this->edge_list.push_back(newEdge7);
                this->edge_list.push_back(newEdge8);
                this->edge_list.push_back(newEdge9);
                this->face_list.push_back(newFaceBotLeft);
                this->face_list.push_back(newFaceBotRight);
                this->face_list.push_back(newFaceTopRight);
            }else{
                //When the cut never leaves the face(Special Case)
            }
        }
    }else if(last){
        //The last intersection point
        if(currentType == 2){
        }else{
            Edge* currentCrossEdge = rightCrossEdge;
            while(currentCrossEdge->twin->next != NULL){
                currentCrossEdge->startParticle = newParticleRight;
                Face* rightFace = currentCrossEdge->twin->face;
                for(int i=0;i<3;i++){
                    if(rightFace->indices[i] == newIndexLeft){
                        rightFace->indices[i] = newIndexRight;
                    }
                }
                currentCrossEdge = currentCrossEdge->twin->next;
            }
            currentCrossEdge->startParticle = newParticleRight;
            newParticleRight->edge = rightCrossEdge;
        }
    }else{
        //A middle intersection point
        if(lastType == 0){
            if(currentType == 0){
                if(nextType == 0){
                    //Last: Particle, Current: Particle, Next: Particle
                    Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                    Edge* currentEdge = newParticleLeft->edge;

                    //Finding the edge between the current and the next 
                    //intersection point 
                    while(true){
                        if(currentEdge->twin->startParticle == nextParticle){
                            break;
                        }else if(currentEdge->twin->next == NULL){
                            break;
                        }else{
                            currentEdge = currentEdge->twin->next;
                        }
                    }

                    while(currentEdge->twin->startParticle != nextParticle){
                        currentEdge = currentEdge->prev->twin;
                    }

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Edge to Edge relations
                    Edge* newEdge = new Edge();
                    newCrossEdgeLeft = currentEdge->twin;
                    newCrossEdgeRight = new Edge();
                    newCrossEdgeLeft->twin = newEdge;
                    newEdge->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = currentEdge;
                    currentEdge->twin = newCrossEdgeRight;

                    //Edge to Particle relations
                    newEdge->startParticle = newParticleLeft;
                    newCrossEdgeRight->startParticle = nextParticle;
                    currentEdge->startParticle = newParticleRight;
                    currentEdge->prev->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge;
                    newParticleRight->edge = currentEdge;

                    //Face to Particle relations
                    Face* rightFace = currentEdge->face;
                    for(int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    this->edge_list.push_back(newEdge);
                    this->edge_list.push_back(newCrossEdgeRight);
                }else if(nextType == 1){
                    //Last:Particle, Current: Particle, Next: Edge
                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size() - 1;

                    Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                    if(currentEdge->prev == NULL){
                        currentEdge = currentEdge->twin->prev;
                    }else{
                        if(currentEdge->prev->startParticle == newParticleLeft){
                            currentEdge = currentEdge->prev;
                        }else{
                            currentEdge = currentEdge->twin->prev;
                        }
                    }
                    //Now, currentEdge is the edge starting from the the current 
                    //intersection point and adjacent to the cut

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight, centreIndex;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        centreIndex = oldFace->indices[0];
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        centreIndex = oldFace->indices[1];
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        centreIndex = oldFace->indices[2];
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to Edge relations
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeLeft = currentEdge->next->twin;
                    newSideEdgeRight = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newSideEdgeLeft->twin = currentEdge->next;
                    currentEdge->next->twin = newSideEdgeLeft;
                    newSideEdgeRight->twin = newEdge3;
                    newEdge3->twin = newSideEdgeRight;

                    //Next/Prev Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = currentEdge->next;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->prev = newCrossEdgeLeft;
                    currentEdge->next->next = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newSideEdgeLeft->startParticle = newParticle;
                    newSideEdgeRight->startParticle = newEdge3->next->startParticle;
                    newEdge3->next->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleLeft->edge = currentEdge;
                    newParticleRight->edge = newEdge2;

                    //Face to Particle/Edge relations
                    oldFace->edge = currentEdge;
                    oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                    Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                    newFace->edge = newEdge2;

                    //Edge to Face relations
                    newCrossEdgeLeft->face = oldFace;
                    newEdge2->face = newFace;
                    newEdge3->face = newFace;
                    newEdge3->next->face = newFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFace);
                }else{
                    //Last: Particle, Current: Particle, Next: Face
                    newParticle = this->particle_list[get<2>(nextIntPt)];
                    int newIndex = get<2>(nextIntPt);
                    
                    Edge* currentEdge = newParticle->edge;
                    while(currentEdge->twin->next != NULL){
                        currentEdge = currentEdge->twin->next;
                    }

                    //Finding the face which contains the next intersection point
                    while(true){
                        Face* currentFace = currentEdge->face;
                        if(isInside(currentFace, get<0>(nextIntPt))){
                            break;
                        }else{
                            currentEdge = currentEdge->prev->twin;
                        }
                    }

                    //Resigning pointers of the last re-meshing operation for data structure consistency
                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to edge relations
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newEdge3->twin = newEdge4;
                    newEdge4->twin = newEdge3;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;

                    //Prev/Next Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = newEdge6;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    newEdge4->next = newEdge5;
                    newEdge4->prev = currentEdge->next;
                    newEdge5->next = currentEdge->next;
                    newEdge5->prev = newEdge4;
                    newEdge6->next = newCrossEdgeLeft;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->next->next = newEdge4;
                    currentEdge->next->prev = newEdge5;
                    currentEdge->next = newEdge6;
                    currentEdge->prev = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newEdge4->startParticle = this->particle_list[oldIndexRight];
                    newEdge5->startParticle = newParticle;
                    newEdge6->startParticle = this->particle_list[oldIndexLeft];

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleRight->edge = newEdge2;
                    newParticleLeft->edge = newEdge1;

                    //Face to Particle/Edge relations
                    oldFace->edge = newEdge5;
                    oldFace->setFace(newIndex,oldIndexLeft,oldIndexRight); 
                    Face* newFaceLeft = new Face(oldIndexLeft, newIndex, newIndexLeft);
                    newFaceLeft->edge = newEdge6;
                    Face* newFaceRight = new Face(newIndex, oldIndexRight, newIndexRight);
                    newFaceRight->edge = newEdge3;

                    //Edge to Face relations
                    newCrossEdgeLeft->face = newFaceLeft;
                    newCrossEdgeLeft->next->face = newFaceLeft; 
                    newEdge6->face = newFaceLeft;
                    newEdge3->face = newFaceRight;
                    newEdge3->next->face = newFaceRight;
                    newEdge2->face = newFaceRight;
                    newEdge4->face = oldFace;
                    newEdge5->face = oldFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->face_list.push_back(newFaceLeft);
                    this->face_list.push_back(newFaceRight);
                }
            }else if(currentType == 1){
                Edge* intersectingEdge = leftSideEdge;
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
                    //Last:Particle, Current: Edge, Next: Particle

                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();

                    newCrossEdgeLeft->startParticle = this->particle_list[oppIndex];
                    newCrossEdgeRight->startParticle = this->particle_list[oppIndex]; 

                    //Edge to Edge relations
                    //Two new edges on the intersecting edge
                    //Four on the path from this intersection point to the next
                    //Two of the second type are to be passed onto the next remeshing iteration
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();

                    //For new edges
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge4;

                    newEdge1->prev = intersectingEdge->prev;
                    newEdge1->next = newEdge4;
                    newEdge4->next = intersectingEdge->prev;
                    newEdge4->prev = newEdge1;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = intersectingEdge->next;

                    //For old edges
                    intersectingEdge->next->next = newCrossEdgeLeft;
                    intersectingEdge->prev->next = newEdge1;
                    intersectingEdge->prev->prev = newEdge4;
                    intersectingEdge->prev = newCrossEdgeLeft; 

                    //Edge to Particle relations
                    newEdge1->startParticle = this->particle_list[oldIndexRight];
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticleLeft;
                    newEdge4->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge3;
                    newParticleRight->edge = newEdge4;

                    //Face to Particle/Edge relations
                    Face* newFace = new Face(oppIndex,oldIndexRight,newIndexRight);
                    newFace->edge = newEdge4->next;
                    intersectingFace->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                    intersectingFace->edge = newCrossEdgeLeft;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newEdge1->face = newFace;
                    newEdge1->next->face = newFace;
                    newEdge1->next->next->face = newFace;
                    newCrossEdgeLeft->face = intersectingFace;

                    //Adding the new edges and faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFace);

                }else if(nextType == 1){
                    //Last:Particle, Current: Edge, Next: Edge
                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size()-1;

                    Edge* newEdge1 = rightSideEdge->twin;
                    Edge* newEdge2 = rightSideEdge;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeRight = new Edge();
                    newSideEdgeLeft = this->edge_list[get<2>(nextIntPt)];
                    if(newSideEdgeLeft->face == intersectingFace){
                        newSideEdgeLeft = newSideEdgeLeft->twin;
                    }
                    bool left = (newSideEdgeLeft->twin == intersectingEdge->next);

                    if(left){
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = intersectingEdge;
                        newCrossEdgeLeft->prev = intersectingEdge->next;
                        newEdge2->next = newEdge5;
                        newEdge2->prev = intersectingEdge->prev;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge6;
                        newEdge5->next = intersectingEdge->prev;
                        newEdge5->prev = newEdge2;
                        newEdge6->next = newEdge3;
                        newEdge6->prev = newEdge7;
                        newEdge7->next = newEdge6;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newEdge2;
                        intersectingEdge->prev->prev = newEdge5;
                        intersectingEdge->prev = newCrossEdgeLeft;
                        intersectingEdge->next->next = newCrossEdgeLeft;

                        //Edge to Particle relations
                        newEdge1->startParticle = newParticleRight;
                        newEdge2->startParticle = this->particle_list[oldIndexRight];
                        newEdge3->startParticle = newParticleRight;
                        newEdge4->startParticle = newParticleLeft;
                        newEdge5->startParticle = newParticleRight;
                        newEdge6->startParticle = this->particle_list[oppIndex];
                        newEdge7->startParticle = newParticle;
                        newSideEdgeRight->startParticle = this->particle_list[oppIndex];
                        newSideEdgeLeft->startParticle = newParticle;
                        newCrossEdgeLeft->startParticle = newParticle;
                        newCrossEdgeRight->startParticle = newParticle;

                        //Particle to edge relations
                        newParticle->edge = newCrossEdgeLeft;
                        newParticleLeft->edge = intersectingEdge;
                        newParticleRight->edge = newEdge5;

                        //Face to Particle/Edge relations
                        Face* newFaceTop = new Face(newIndexRight,oppIndex,oldIndexRight);
                        newFaceTop->edge = newEdge5;
                        Face* newFaceBot = new Face(newIndexRight,newIndex,oppIndex);
                        newFaceBot->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, newIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceTop;
                        newEdge3->face = newFaceBot;
                        newEdge5->face = newFaceTop;
                        newEdge6->face = newFaceBot;
                        newEdge7->face = newFaceBot;
                        newCrossEdgeLeft->face = intersectingFace;
                        newEdge5->next->face = newFaceTop;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceTop);
                        this->face_list.push_back(newFaceBot);
                    }else{
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = newEdge6;
                        newCrossEdgeLeft->prev = intersectingEdge->prev;
                        newEdge2->next = newEdge3;
                        newEdge2->prev = newEdge7;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge2;
                        newEdge5->next = intersectingEdge;
                        newEdge5->prev = intersectingEdge->next;
                        newEdge6->next = intersectingEdge->prev;
                        newEdge6->prev = newCrossEdgeLeft;
                        newEdge7->next = newEdge2;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newCrossEdgeLeft;
                        intersectingEdge->prev->prev = newEdge6;
                        intersectingEdge->prev = newEdge5;
                        intersectingEdge->next->next = newEdge5;

                        //Edge to Particle relations
                        newEdge1->startParticle = newParticleRight;
                        newEdge2->startParticle = this->particle_list[oldIndexRight];
                        newEdge3->startParticle = newParticleRight;
                        newEdge4->startParticle = newParticleLeft;
                        newEdge5->startParticle = this->particle_list[oppIndex];
                        newEdge6->startParticle = newParticleLeft;
                        newEdge7->startParticle = newParticle;
                        newSideEdgeRight->startParticle = this->particle_list[oldIndexRight];
                        newSideEdgeLeft->startParticle = newParticle;
                        newCrossEdgeLeft->startParticle = newParticle;
                        newCrossEdgeRight->startParticle = newParticle;

                        //Particle to edge relations
                        newParticle->edge = newCrossEdgeLeft;
                        newParticleLeft->edge = intersectingEdge;
                        newParticleRight->edge = newEdge3;

                        //Face to Particle/Edge relations
                        Face* newFaceLeft = new Face(newIndexLeft,oppIndex,newIndex);
                        newFaceLeft->edge = newEdge6;
                        Face* newFaceRight = new Face(newIndexRight,newIndex,oldIndexRight);
                        newFaceRight->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, oppIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceRight;
                        newEdge3->face = newFaceRight;
                        newEdge5->face = intersectingFace;
                        newEdge6->face = newFaceLeft;
                        newEdge7->face = newFaceRight;
                        newCrossEdgeLeft->face = newFaceLeft;
                        newEdge6->next->face = newFaceLeft;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceLeft);
                        this->face_list.push_back(newFaceRight);
                    }
                }else{
                    //Last: Particle, Current: Edge, Next: Face
                    newParticle = this->particle_list[get<2>(nextIntPt)];
                    int newIndex = get<2>(nextIntPt);

                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    //Edge to edge relations
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    Edge* newEdge8 = new Edge();
                    Edge* newEdge9 = new Edge();
                    Edge* newEdge10 = new Edge();

                    //Twin relations
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge3;
                    newEdge4->twin = newEdge5;
                    newEdge5->twin = newEdge4;
                    newEdge6->twin = newEdge7;
                    newEdge7->twin = newEdge6;
                    newEdge8->twin = newEdge9;
                    newEdge9->twin = newEdge8;
                    newEdge10->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge10;

                    //Next/Prev relations
                    newEdge1->next = newEdge3;
                    newEdge1->prev = newEdge4;
                    newEdge3->next = newEdge4;
                    newEdge3->prev = newEdge1;
                    newEdge4->next = newEdge1;
                    newEdge4->prev = newEdge3;
                    newEdge5->next = newEdge6;
                    newEdge5->prev = intersectingEdge->prev;
                    newEdge6->next = intersectingEdge->prev;
                    newEdge6->prev = newEdge5;
                    newEdge7->next = newEdge8;
                    newEdge7->prev = intersectingEdge->next;
                    newEdge8->next = intersectingEdge->next;
                    newEdge8->prev = newEdge7;
                    newEdge9->next = newCrossEdgeLeft;
                    newEdge9->prev = intersectingEdge;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = newEdge9;
                    intersectingEdge->prev->prev = newEdge6;
                    intersectingEdge->prev->next = newEdge5;
                    intersectingEdge->prev = newCrossEdgeLeft;
                    intersectingEdge->next->next = newEdge7;
                    intersectingEdge->next->prev = newEdge8;
                    intersectingEdge->next = newEdge9;

                    //Edge to Particle relations
                    newEdge1->startParticle = this->particle_list[oldIndexRight];
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticleRight;
                    newEdge4->startParticle = newParticle;
                    newEdge5->startParticle = this->particle_list[oldIndexRight];
                    newEdge6->startParticle = newParticle;
                    newEdge7->startParticle = this->particle_list[oppIndex];
                    newEdge8->startParticle = newParticle;
                    newEdge9->startParticle = this->particle_list[oldIndexLeft];
                    newEdge10->startParticle = newParticleLeft;
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;

                    //Particle to Edge relations
                    newParticle->edge = newEdge4;
                    newParticleRight->edge = newEdge3;
                    newParticleLeft->edge = newEdge10;

                    //Face to Particle/Edge relations
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    intersectingFace->setFace(newIndexLeft,oldIndexLeft,newIndex);
                    intersectingFace->edge = intersectingEdge;
                    Face* newFaceBotLeft = new Face(newIndex,oldIndexLeft,oppIndex);
                    newFaceBotLeft->edge = newEdge8;
                    Face* newFaceBotRight = new Face(newIndex,oppIndex,oldIndexRight);
                    newFaceBotRight->edge = newEdge6;
                    Face* newFaceTopRight = new Face(newIndex,oldIndexRight,newIndexRight);
                    newFaceTopRight->edge = newEdge4;

                    //Edge to Face relations
                    newEdge1->face = newFaceTopRight;
                    newEdge2->face = oldRightFace;
                    newEdge3->face = newFaceTopRight;
                    newEdge4->face = newFaceTopRight;
                    newEdge5->face = newFaceBotRight;
                    newEdge6->face = newFaceBotRight;
                    newEdge6->next->face = newFaceBotRight;
                    newEdge7->face = newFaceBotLeft;
                    newEdge8->face = newFaceBotLeft;
                    newEdge8->next->face = newFaceBotLeft;
                    newEdge9->face = intersectingFace;
                    newCrossEdgeLeft->face = intersectingFace;
                    newCrossEdgeLeft->next->face = intersectingFace;

                    //Adding the new edges/faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newEdge8);
                    this->edge_list.push_back(newEdge9);
                    this->edge_list.push_back(newEdge10);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFaceBotLeft);
                    this->face_list.push_back(newFaceBotRight);
                    this->face_list.push_back(newFaceTopRight);
                }
            }else{
                //Case 3(Special Case): This intersection point occurs between two planes(as a part of the curve)
                if(nextType == 0){
                    //Last: Particle, Current: Face, Next: Particle
                    Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                    Edge* currentEdge = newParticleLeft->edge;

                    //Finding the edge between the current and the next 
                    //intersection point 
                    while(true){
                        if(currentEdge->twin->startParticle == nextParticle){
                            break;
                        }else if(currentEdge->twin->next == NULL){
                            break;
                        }else{
                            currentEdge = currentEdge->twin->next;
                        }
                    }

                    while(currentEdge->twin->startParticle != nextParticle){
                        currentEdge = currentEdge->prev->twin;
                    }

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Edge to Edge relations
                    Edge* newEdge = new Edge();
                    newCrossEdgeLeft = currentEdge->twin;
                    newCrossEdgeRight = new Edge();
                    newCrossEdgeLeft->twin = newEdge;
                    newEdge->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = currentEdge;
                    currentEdge->twin = newCrossEdgeRight;

                    //Edge to Particle relations
                    newEdge->startParticle = newParticleLeft;
                    newCrossEdgeRight->startParticle = nextParticle;
                    currentEdge->startParticle = newParticleRight;
                    currentEdge->prev->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge;
                    newParticleRight->edge = currentEdge;

                    //Face to Particle relations
                    Face* rightFace = currentEdge->face;
                    for(int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    this->edge_list.push_back(newEdge);
                    this->edge_list.push_back(newCrossEdgeRight);
                }else if(nextType==1){
                    //Last: Particle, Current: Face, Next: Edge
                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size() - 1;

                    Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                    if(currentEdge->prev == NULL){
                        currentEdge = currentEdge->twin->prev;
                    }else{
                        if(currentEdge->prev->startParticle == newParticleLeft){
                            currentEdge = currentEdge->prev;
                        }else{
                            currentEdge = currentEdge->twin->prev;
                        }
                    }
                    //Now, currentEdge is the edge starting from the the current 
                    //intersection point and adjacent to the cut

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight, centreIndex;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        centreIndex = oldFace->indices[0];
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        centreIndex = oldFace->indices[1];
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        centreIndex = oldFace->indices[2];
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to Edge relations
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeLeft = currentEdge->next->twin;
                    newSideEdgeRight = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newSideEdgeLeft->twin = currentEdge->next;
                    currentEdge->next->twin = newSideEdgeRight;
                    newSideEdgeRight->twin = newEdge3;
                    newEdge3->twin = newSideEdgeRight;

                    //Next/Prev Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = currentEdge->next;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->prev = newCrossEdgeLeft;
                    currentEdge->next->next = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newSideEdgeLeft->startParticle = newParticle;
                    newSideEdgeRight->startParticle = newEdge3->next->startParticle;
                    newEdge3->next->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleLeft->edge = currentEdge;
                    newParticleRight->edge = newEdge2;

                    //Face to Particle/Edge relations
                    oldFace->edge = currentEdge;
                    oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                    Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                    newFace->edge = newEdge2;

                    //Edge to Face relations
                    newCrossEdgeLeft->face = oldFace;
                    newEdge2->face = newFace;
                    newEdge3->face = newFace;
                    newEdge3->next->face = newFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFace);
                }else{
                    //This case is not possible
                }
            }
        }else if(lastType == 1){
            if(currentType == 0){
                if(nextType == 0){
                    //Last:Edge, Current: Particle, Next: Particle

                    Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                    Edge* currentEdge = newParticleLeft->edge;

                    //Finding the edge between the current and the next 
                    //intersection point 
                    while(true){
                        if(currentEdge->twin->startParticle == nextParticle){
                            break;
                        }else if(currentEdge->twin->next == NULL){
                            break;
                        }else{
                            currentEdge = currentEdge->twin->next;
                        }
                    }

                    while(currentEdge->twin->startParticle != nextParticle){
                        currentEdge = currentEdge->prev->twin;
                    }

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Edge to Edge relations
                    Edge* newEdge = new Edge();
                    newCrossEdgeLeft = currentEdge->twin;
                    newCrossEdgeRight = new Edge();
                    newCrossEdgeLeft->twin = newEdge;
                    newEdge->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = currentEdge;
                    currentEdge->twin = newCrossEdgeRight;

                    //Edge to Particle relations
                    newEdge->startParticle = newParticleLeft;
                    newCrossEdgeRight->startParticle = nextParticle;
                    currentEdge->startParticle = newParticleRight;
                    currentEdge->prev->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge;
                    newParticleRight->edge = currentEdge;

                    //Face to Particle relations
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    Face* rightFace = currentEdge->face;
                    for(int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    this->edge_list.push_back(newEdge);
                    this->edge_list.push_back(newCrossEdgeRight);
                }else if(nextType == 1){
                    //Last:Edge, Current: Particle, Next: Edge
                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size() - 1;

                    Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                    if(currentEdge->prev == NULL){
                        currentEdge = currentEdge->twin->prev;
                    }else{
                        if(currentEdge->prev->startParticle == newParticleLeft){
                            currentEdge = currentEdge->prev;
                        }else{
                            currentEdge = currentEdge->twin->prev;
                        }
                    }
                    //Now, currentEdge is the edge starting from the the current 
                    //intersection point and adjacent to the cut

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight, centreIndex;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        centreIndex = oldFace->indices[0];
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        centreIndex = oldFace->indices[1];
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        centreIndex = oldFace->indices[2];
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to Edge relations
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeLeft = currentEdge->next->twin;
                    newSideEdgeRight = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newSideEdgeLeft->twin = currentEdge->next;
                    currentEdge->next->twin = newSideEdgeLeft;
                    newSideEdgeRight->twin = newEdge3;
                    newEdge3->twin = newSideEdgeRight;

                    //Next/Prev Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = currentEdge->next;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->prev = newCrossEdgeLeft;
                    currentEdge->next->next = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newSideEdgeLeft->startParticle = newParticle;
                    newSideEdgeRight->startParticle = newEdge3->next->startParticle;
                    newEdge3->next->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleLeft->edge = currentEdge;
                    newParticleRight->edge = newEdge2;

                    //Face to Particle/Edge relations
                    oldFace->edge = currentEdge;
                    oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                    Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                    newFace->edge = newEdge2;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newCrossEdgeLeft->face = oldFace;
                    newEdge2->face = newFace;
                    newEdge3->face = newFace;
                    newEdge3->next->face = newFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFace);
                }else{
                    //Last:Edge, Current: Particle, Next: Face
                    newParticle = this->particle_list[get<2>(nextIntPt)];
                    int newIndex = get<2>(nextIntPt);
                    
                    Edge* currentEdge = newParticle->edge;
                    while(currentEdge->twin->next != NULL){
                        currentEdge = currentEdge->twin->next;
                    }

                    //Finding the face which contains the next intersection point
                    while(true){
                        Face* currentFace = currentEdge->face;
                        if(isInside(currentFace, get<0>(nextIntPt))){
                            break;
                        }else{
                            currentEdge = currentEdge->prev->twin;
                        }
                    }

                    //Resigning pointers of the last re-meshing operation for data structure consistency
                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to edge relations
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newEdge3->twin = newEdge4;
                    newEdge4->twin = newEdge3;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;

                    //Prev/Next Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = newEdge6;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    newEdge4->next = newEdge5;
                    newEdge4->prev = currentEdge->next;
                    newEdge5->next = currentEdge->next;
                    newEdge5->prev = newEdge4;
                    newEdge6->next = newCrossEdgeLeft;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->next->next = newEdge4;
                    currentEdge->next->prev = newEdge5;
                    currentEdge->next = newEdge6;
                    currentEdge->prev = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newEdge4->startParticle = this->particle_list[oldIndexRight];
                    newEdge5->startParticle = newParticle;
                    newEdge6->startParticle = this->particle_list[oldIndexLeft];

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleRight->edge = newEdge2;
                    newParticleLeft->edge = newEdge1;

                    //Face to Particle/Edge relations
                    oldFace->edge = newEdge5;
                    oldFace->setFace(newIndex,oldIndexLeft,oldIndexRight); 
                    Face* newFaceLeft = new Face(oldIndexLeft, newIndex, newIndexLeft);
                    newFaceLeft->edge = newEdge6;
                    Face* newFaceRight = new Face(newIndex, oldIndexRight, newIndexRight);
                    newFaceRight->edge = newEdge3;

                    //Edge to Face relations
                    newCrossEdgeLeft->face = newFaceLeft;
                    newCrossEdgeLeft->next->face = newFaceLeft; 
                    newEdge6->face = newFaceLeft;
                    newEdge3->face = newFaceRight;
                    newEdge3->next->face = newFaceRight;
                    newEdge2->face = newFaceRight;
                    newEdge4->face = oldFace;
                    newEdge5->face = oldFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->face_list.push_back(newFaceLeft);
                    this->face_list.push_back(newFaceRight);
                }
            }else if(currentType == 1){
                Edge* intersectingEdge = leftSideEdge;
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
                    //Last:Edge, Current: Edge, Next: Particle

                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();

                    newCrossEdgeLeft->startParticle = this->particle_list[oppIndex];
                    newCrossEdgeRight->startParticle = this->particle_list[oppIndex]; 

                    //Edge to Edge relations
                    //Two new edges on the intersecting edge
                    //Four on the path from this intersection point to the next
                    //Two of the second type are to be passed onto the next remeshing iteration
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();

                    //For new edges
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge4;

                    newEdge1->prev = intersectingEdge->prev;
                    newEdge1->next = newEdge4;
                    newEdge4->next = intersectingEdge->prev;
                    newEdge4->prev = newEdge1;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = intersectingEdge->next;

                    //For old edges
                    intersectingEdge->next->next = newCrossEdgeLeft;
                    intersectingEdge->prev->next = newEdge1;
                    intersectingEdge->prev->prev = newEdge4;
                    intersectingEdge->prev = newCrossEdgeLeft; 

                    //Edge to Particle relations
                    newEdge1->startParticle = this->particle_list[oldIndexRight];
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticleLeft;
                    newEdge4->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge3;
                    newParticleRight->edge = newEdge4;

                    //Face to Particle/Edge relations
                    Face* newFace = new Face(oppIndex,oldIndexRight,newIndexRight);
                    newFace->edge = newEdge4->next;
                    intersectingFace->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                    intersectingFace->edge = newCrossEdgeLeft;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newEdge1->face = newFace;
                    newEdge1->next->face = newFace;
                    newEdge1->next->next->face = newFace;
                    newCrossEdgeLeft->face = intersectingFace;

                    //Adding the new edges and faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFace);

                }else if(nextType == 1){
                    //Last:Edge, Current: Edge, Next: Edge

                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size()-1;

                    Edge* newEdge1 = rightSideEdge->twin;
                    Edge* newEdge2 = rightSideEdge;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeRight = new Edge();
                    newSideEdgeLeft = this->edge_list[get<2>(nextIntPt)];
                    if(newSideEdgeLeft->face == intersectingFace){
                        newSideEdgeLeft = newSideEdgeLeft->twin;
                    }
                    bool left = (newSideEdgeLeft->twin == intersectingEdge->next);

                    if(left){
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = intersectingEdge;
                        newCrossEdgeLeft->prev = intersectingEdge->next;
                        newEdge2->next = newEdge5;
                        newEdge2->prev = intersectingEdge->prev;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge6;
                        newEdge5->next = intersectingEdge->prev;
                        newEdge5->prev = newEdge2;
                        newEdge6->next = newEdge3;
                        newEdge6->prev = newEdge7;
                        newEdge7->next = newEdge6;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newEdge2;
                        intersectingEdge->prev->prev = newEdge5;
                        intersectingEdge->prev = newCrossEdgeLeft;
                        intersectingEdge->next->next = newCrossEdgeLeft;

                        //Edge to Particle relations
                        newEdge1->startParticle = newParticleRight;
                        newEdge2->startParticle = this->particle_list[oldIndexRight];
                        newEdge3->startParticle = newParticleRight;
                        newEdge4->startParticle = newParticleLeft;
                        newEdge5->startParticle = newParticleRight;
                        newEdge6->startParticle = this->particle_list[oppIndex];
                        newEdge7->startParticle = newParticle;
                        newSideEdgeRight->startParticle = this->particle_list[oppIndex];
                        newSideEdgeLeft->startParticle = newParticle;
                        newCrossEdgeLeft->startParticle = newParticle;
                        newCrossEdgeRight->startParticle = newParticle;

                        //Particle to edge relations
                        newParticle->edge = newCrossEdgeLeft;
                        newParticleLeft->edge = intersectingEdge;
                        newParticleRight->edge = newEdge5;

                        //Face to Particle/Edge relations
                        Face* newFaceTop = new Face(newIndexRight,oppIndex,oldIndexRight);
                        newFaceTop->edge = newEdge5;
                        Face* newFaceBot = new Face(newIndexRight,newIndex,oppIndex);
                        newFaceBot->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, newIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceTop;
                        newEdge3->face = newFaceBot;
                        newEdge5->face = newFaceTop;
                        newEdge6->face = newFaceBot;
                        newEdge7->face = newFaceBot;
                        newCrossEdgeLeft->face = intersectingFace;
                        newEdge5->next->face = newFaceTop;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceTop);
                        this->face_list.push_back(newFaceBot);
                    }else{
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = newEdge6;
                        newCrossEdgeLeft->prev = intersectingEdge->prev;
                        newEdge2->next = newEdge3;
                        newEdge2->prev = newEdge7;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge2;
                        newEdge5->next = intersectingEdge;
                        newEdge5->prev = intersectingEdge->next;
                        newEdge6->next = intersectingEdge->prev;
                        newEdge6->prev = newCrossEdgeLeft;
                        newEdge7->next = newEdge2;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newCrossEdgeLeft;
                        intersectingEdge->prev->prev = newEdge6;
                        intersectingEdge->prev = newEdge5;
                        intersectingEdge->next->next = newEdge5;

                        //Edge to Particle relations
                        newEdge1->startParticle = newParticleRight;
                        newEdge2->startParticle = this->particle_list[oldIndexRight];
                        newEdge3->startParticle = newParticleRight;
                        newEdge4->startParticle = newParticleLeft;
                        newEdge5->startParticle = this->particle_list[oppIndex];
                        newEdge6->startParticle = newParticleLeft;
                        newEdge7->startParticle = newParticle;
                        newSideEdgeRight->startParticle = this->particle_list[oldIndexRight];
                        newSideEdgeLeft->startParticle = newParticle;
                        newCrossEdgeLeft->startParticle = newParticle;
                        newCrossEdgeRight->startParticle = newParticle;

                        //Particle to edge relations
                        newParticle->edge = newCrossEdgeLeft;
                        newParticleLeft->edge = intersectingEdge;
                        newParticleRight->edge = newEdge3;

                        //Face to Particle/Edge relations
                        Face* newFaceLeft = new Face(newIndexLeft,oppIndex,newIndex);
                        newFaceLeft->edge = newEdge6;
                        Face* newFaceRight = new Face(newIndexRight,newIndex,oldIndexRight);
                        newFaceRight->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, oppIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceRight;
                        newEdge3->face = newFaceRight;
                        newEdge5->face = intersectingFace;
                        newEdge6->face = newFaceLeft;
                        newEdge7->face = newFaceRight;
                        newCrossEdgeLeft->face = newFaceLeft;
                        newEdge6->next->face = newFaceLeft;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceLeft);
                        this->face_list.push_back(newFaceRight);
                    }
                }else{
                    //Last: Edge, Current: Edge, Next: Face
                    newParticle = this->particle_list[get<2>(nextIntPt)];
                    int newIndex = get<2>(nextIntPt);

                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    //Edge to edge relations
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    Edge* newEdge8 = new Edge();
                    Edge* newEdge9 = new Edge();
                    Edge* newEdge10 = new Edge();

                    //Twin relations
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge3;
                    newEdge4->twin = newEdge5;
                    newEdge5->twin = newEdge4;
                    newEdge6->twin = newEdge7;
                    newEdge7->twin = newEdge6;
                    newEdge8->twin = newEdge9;
                    newEdge9->twin = newEdge8;
                    newEdge10->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge10;

                    //Next/Prev relations
                    newEdge1->next = newEdge3;
                    newEdge1->prev = newEdge4;
                    newEdge3->next = newEdge4;
                    newEdge3->prev = newEdge1;
                    newEdge4->next = newEdge1;
                    newEdge4->prev = newEdge3;
                    newEdge5->next = newEdge6;
                    newEdge5->prev = intersectingEdge->prev;
                    newEdge6->next = intersectingEdge->prev;
                    newEdge6->prev = newEdge5;
                    newEdge7->next = newEdge8;
                    newEdge7->prev = intersectingEdge->next;
                    newEdge8->next = intersectingEdge->next;
                    newEdge8->prev = newEdge7;
                    newEdge9->next = newCrossEdgeLeft;
                    newEdge9->prev = intersectingEdge;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = newEdge9;
                    intersectingEdge->prev->prev = newEdge6;
                    intersectingEdge->prev->next = newEdge5;
                    intersectingEdge->prev = newCrossEdgeLeft;
                    intersectingEdge->next->next = newEdge7;
                    intersectingEdge->next->prev = newEdge8;
                    intersectingEdge->next = newEdge9;

                    //Edge to Particle relations
                    newEdge1->startParticle = this->particle_list[oldIndexRight];
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticleRight;
                    newEdge4->startParticle = newParticle;
                    newEdge5->startParticle = this->particle_list[oldIndexRight];
                    newEdge6->startParticle = newParticle;
                    newEdge7->startParticle = this->particle_list[oppIndex];
                    newEdge8->startParticle = newParticle;
                    newEdge9->startParticle = this->particle_list[oldIndexLeft];
                    newEdge10->startParticle = newParticleLeft;
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;

                    //Particle to Edge relations
                    newParticle->edge = newEdge4;
                    newParticleRight->edge = newEdge3;
                    newParticleLeft->edge = newEdge10;

                    //Face to Particle/Edge relations
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    intersectingFace->setFace(newIndexLeft,oldIndexLeft,newIndex);
                    intersectingFace->edge = intersectingEdge;
                    Face* newFaceBotLeft = new Face(newIndex,oldIndexLeft,oppIndex);
                    newFaceBotLeft->edge = newEdge8;
                    Face* newFaceBotRight = new Face(newIndex,oppIndex,oldIndexRight);
                    newFaceBotRight->edge = newEdge6;
                    Face* newFaceTopRight = new Face(newIndex,oldIndexRight,newIndexRight);
                    newFaceTopRight->edge = newEdge4;

                    //Edge to Face relations
                    newEdge1->face = newFaceTopRight;
                    newEdge2->face = oldRightFace;
                    newEdge3->face = newFaceTopRight;
                    newEdge4->face = newFaceTopRight;
                    newEdge5->face = newFaceBotRight;
                    newEdge6->face = newFaceBotRight;
                    newEdge6->next->face = newFaceBotRight;
                    newEdge7->face = newFaceBotLeft;
                    newEdge8->face = newFaceBotLeft;
                    newEdge8->next->face = newFaceBotLeft;
                    newEdge9->face = intersectingFace;
                    newCrossEdgeLeft->face = intersectingFace;
                    newCrossEdgeLeft->next->face = intersectingFace;

                    //Adding the new edges/faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newEdge8);
                    this->edge_list.push_back(newEdge9);
                    this->edge_list.push_back(newEdge10);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFaceBotLeft);
                    this->face_list.push_back(newFaceBotRight);
                    this->face_list.push_back(newFaceTopRight);
                }
            }else{
                //Case 3(Special Case): This intersection point occurs between two planes(as a part of the curve)
                if(nextType == 0){
                    //Last:Edge, Current: Face, Next: Particle

                    Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                    Edge* currentEdge = newParticleLeft->edge;

                    //Finding the edge between the current and the next 
                    //intersection point 
                    while(true){
                        if(currentEdge->twin->startParticle == nextParticle){
                            break;
                        }else if(currentEdge->twin->next == NULL){
                            break;
                        }else{
                            currentEdge = currentEdge->twin->next;
                        }
                    }

                    while(currentEdge->twin->startParticle != nextParticle){
                        currentEdge = currentEdge->prev->twin;
                    }

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Edge to Edge relations
                    Edge* newEdge = new Edge();
                    newCrossEdgeLeft = currentEdge->twin;
                    newCrossEdgeRight = new Edge();
                    newCrossEdgeLeft->twin = newEdge;
                    newEdge->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = currentEdge;
                    currentEdge->twin = newCrossEdgeRight;

                    //Edge to Particle relations
                    newEdge->startParticle = newParticleLeft;
                    newCrossEdgeRight->startParticle = nextParticle;
                    currentEdge->startParticle = newParticleRight;
                    currentEdge->prev->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge;
                    newParticleRight->edge = currentEdge;

                    //Face to Particle relations
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    Face* rightFace = currentEdge->face;
                    for(int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    this->edge_list.push_back(newEdge);
                    this->edge_list.push_back(newCrossEdgeRight);
                }else if(nextType == 1){
                    //Last:Edge, Current: Face, Next: Edge
                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size() - 1;

                    Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                    if(currentEdge->prev == NULL){
                        currentEdge = currentEdge->twin->prev;
                    }else{
                        if(currentEdge->prev->startParticle == newParticleLeft){
                            currentEdge = currentEdge->prev;
                        }else{
                            currentEdge = currentEdge->twin->prev;
                        }
                    }
                    //Now, currentEdge is the edge starting from the the current 
                    //intersection point and adjacent to the cut

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight, centreIndex;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        centreIndex = oldFace->indices[0];
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        centreIndex = oldFace->indices[1];
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        centreIndex = oldFace->indices[2];
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to Edge relations
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeLeft = currentEdge->next->twin;
                    newSideEdgeRight = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newSideEdgeLeft->twin = currentEdge->next;
                    currentEdge->next->twin = newSideEdgeRight;
                    newSideEdgeRight->twin = newEdge3;
                    newEdge3->twin = newSideEdgeRight;

                    //Next/Prev Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = currentEdge->next;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->prev = newCrossEdgeLeft;
                    currentEdge->next->next = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newSideEdgeLeft->startParticle = newParticle;
                    newSideEdgeRight->startParticle = newEdge3->next->startParticle;
                    newEdge3->next->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleLeft->edge = currentEdge;
                    newParticleRight->edge = newEdge2;

                    //Face to Particle/Edge relations
                    oldFace->edge = currentEdge;
                    oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                    Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                    newFace->edge = newEdge2;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newCrossEdgeLeft->face = oldFace;
                    newEdge2->face = newFace;
                    newEdge3->face = newFace;
                    newEdge3->next->face = newFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFace);
                }else{
                    //This case is not possible
                }
            }
        }else{
            if(currentType == 0){
                if(nextType == 0){
                    //Last: Face, Current: Particle, Next: Particle
                    Particle* nextParticle = this->particle_list[get<2>(nextIntPt)];
                    Edge* currentEdge = newParticleLeft->edge;

                    //Finding the edge between the current and the next 
                    //intersection point 
                    while(true){
                        if(currentEdge->twin->startParticle == nextParticle){
                            break;
                        }else if(currentEdge->twin->next == NULL){
                            break;
                        }else{
                            currentEdge = currentEdge->twin->next;
                        }
                    }

                    while(currentEdge->twin->startParticle != nextParticle){
                        currentEdge = currentEdge->prev->twin;
                    }

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Edge to Edge relations
                    Edge* newEdge = new Edge();
                    newCrossEdgeLeft = currentEdge->twin;
                    newCrossEdgeRight = new Edge();
                    newCrossEdgeLeft->twin = newEdge;
                    newEdge->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = currentEdge;
                    currentEdge->twin = newCrossEdgeRight;

                    //Edge to Particle relations
                    newEdge->startParticle = newParticleLeft;
                    newCrossEdgeRight->startParticle = nextParticle;
                    currentEdge->startParticle = newParticleRight;
                    currentEdge->prev->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge;
                    newParticleRight->edge = currentEdge;

                    //Face to Particle relations
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    Face* rightFace = currentEdge->face;
                    for(int i=0;i<3;i++){
                        if(rightFace->indices[i] == newIndexLeft){
                            rightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    this->edge_list.push_back(newEdge);
                    this->edge_list.push_back(newCrossEdgeRight);
                }else if(nextType == 1){
                    //Last: Face, Current: Particle, Next: Edge
                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size() - 1;

                    Edge* currentEdge = this->edge_list[get<2>(nextIntPt)];
                    if(currentEdge->prev == NULL){
                        currentEdge = currentEdge->twin->prev;
                    }else{
                        if(currentEdge->prev->startParticle == newParticleLeft){
                            currentEdge = currentEdge->prev;
                        }else{
                            currentEdge = currentEdge->twin->prev;
                        }
                    }
                    //Now, currentEdge is the edge starting from the the current 
                    //intersection point and adjacent to the cut

                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight, centreIndex;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        centreIndex = oldFace->indices[0];
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        centreIndex = oldFace->indices[1];
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        centreIndex = oldFace->indices[2];
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to Edge relations
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeLeft = currentEdge->next->twin;
                    newSideEdgeRight = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newSideEdgeLeft->twin = currentEdge->next;
                    currentEdge->next->twin = newSideEdgeLeft;
                    newSideEdgeRight->twin = newEdge3;
                    newEdge3->twin = newSideEdgeRight;

                    //Next/Prev Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = currentEdge->next;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->prev = newCrossEdgeLeft;
                    currentEdge->next->next = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newSideEdgeLeft->startParticle = newParticle;
                    newSideEdgeRight->startParticle = newEdge3->next->startParticle;
                    newEdge3->next->twin->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleLeft->edge = currentEdge;
                    newParticleRight->edge = newEdge2;

                    //Face to Particle/Edge relations
                    oldFace->edge = currentEdge;
                    oldFace->setFace(newIndexLeft, oldIndexLeft, newIndex);
                    Face* newFace = new Face(newIndexRight, newIndex, oldIndexRight);
                    newFace->edge = newEdge2;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newCrossEdgeLeft->face = oldFace;
                    newEdge2->face = newFace;
                    newEdge3->face = newFace;
                    newEdge3->next->face = newFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newSideEdgeRight);
                    this->face_list.push_back(newFace);
                }else{
                    //Last: Face, Current: Particle, Next: Face
                    newParticle = this->particle_list[get<2>(nextIntPt)];
                    int newIndex = get<2>(nextIntPt);
                    
                    Edge* currentEdge = newParticle->edge;
                    while(currentEdge->twin->next != NULL){
                        currentEdge = currentEdge->twin->next;
                    }

                    //Finding the face which contains the next intersection point
                    while(true){
                        Face* currentFace = currentEdge->face;
                        if(isInside(currentFace, get<0>(nextIntPt))){
                            break;
                        }else{
                            currentEdge = currentEdge->prev->twin;
                        }
                    }

                    //Resigning pointers of the last re-meshing operation for data structure consistency
                    Edge* currentCrossEdge = rightCrossEdge;
                    while(currentCrossEdge->twin != currentEdge->prev){
                        currentCrossEdge->startParticle = newParticleRight;
                        Face* oldRightFace = currentCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }
                        currentCrossEdge = currentCrossEdge->twin->next;
                    }
                    currentCrossEdge->startParticle = newParticleRight;

                    //Obtaining old Particle indices
                    int oldIndexLeft, oldIndexRight;
                    Face* oldFace = currentEdge->face;
                    if(oldFace->edge == currentEdge){
                        oldIndexLeft = oldFace->indices[1];
                        oldIndexRight = oldFace->indices[2];
                    }else if(oldFace->edge->next == currentEdge){
                        oldIndexLeft = oldFace->indices[2];
                        oldIndexRight = oldFace->indices[0];
                    }else{
                        oldIndexLeft = oldFace->indices[0];
                        oldIndexRight = oldFace->indices[1];
                    }

                    //Edge to edge relations
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    Edge* newEdge1 = new Edge();
                    Edge* newEdge2 = new Edge();
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();

                    //Twin Edges
                    newCrossEdgeLeft->twin = newEdge1;
                    newEdge1->twin = newCrossEdgeLeft;
                    newCrossEdgeRight->twin = newEdge2;
                    newEdge2->twin = newCrossEdgeRight;
                    newEdge3->twin = newEdge4;
                    newEdge4->twin = newEdge3;
                    newEdge5->twin = newEdge6;
                    newEdge6->twin = newEdge5;

                    //Prev/Next Edges
                    newCrossEdgeLeft->next = currentEdge;
                    newCrossEdgeLeft->prev = newEdge6;
                    newEdge2->next = newEdge3;
                    newEdge2->prev = currentEdge->prev;
                    newEdge3->next = currentEdge->prev;
                    newEdge3->prev = newEdge2;
                    newEdge4->next = newEdge5;
                    newEdge4->prev = currentEdge->next;
                    newEdge5->next = currentEdge->next;
                    newEdge5->prev = newEdge4;
                    newEdge6->next = newCrossEdgeLeft;
                    currentEdge->prev->next = newEdge2;
                    currentEdge->prev->prev = newEdge3;
                    currentEdge->next->next = newEdge4;
                    currentEdge->next->prev = newEdge5;
                    currentEdge->next = newEdge6;
                    currentEdge->prev = newCrossEdgeLeft;

                    //Edge to Particle relations
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;
                    newEdge1->startParticle = newParticleLeft;
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticle;
                    newEdge4->startParticle = this->particle_list[oldIndexRight];
                    newEdge5->startParticle = newParticle;
                    newEdge6->startParticle = this->particle_list[oldIndexLeft];

                    //Particle to Edge relations
                    newParticle->edge = newCrossEdgeLeft;
                    newParticleRight->edge = newEdge2;
                    newParticleLeft->edge = newEdge1;

                    //Face to Particle/Edge relations
                    oldFace->edge = newEdge5;
                    oldFace->setFace(newIndex,oldIndexLeft,oldIndexRight); 
                    Face* newFaceLeft = new Face(oldIndexLeft, newIndex, newIndexLeft);
                    newFaceLeft->edge = newEdge6;
                    Face* newFaceRight = new Face(newIndex, oldIndexRight, newIndexRight);
                    newFaceRight->edge = newEdge3;

                    //Edge to Face relations
                    newCrossEdgeLeft->face = newFaceLeft;
                    newCrossEdgeLeft->next->face = newFaceLeft; 
                    newEdge6->face = newFaceLeft;
                    newEdge3->face = newFaceRight;
                    newEdge3->next->face = newFaceRight;
                    newEdge2->face = newFaceRight;
                    newEdge4->face = oldFace;
                    newEdge5->face = oldFace;

                    //Adding the edges/faces
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->edge_list.push_back(newEdge1);
                    this->edge_list.push_back(newEdge2);
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->face_list.push_back(newFaceLeft);
                    this->face_list.push_back(newFaceRight);
                }
            }else if(currentType == 1){
                Edge* intersectingEdge = leftSideEdge;
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
                    //Last: Face, Current: Edge, Next: Particle

                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();

                    newCrossEdgeLeft->startParticle = this->particle_list[oppIndex];
                    newCrossEdgeRight->startParticle = this->particle_list[oppIndex]; 

                    //Edge to Edge relations
                    //Two new edges on the intersecting edge
                    //Four on the path from this intersection point to the next
                    //Two of the second type are to be passed onto the next remeshing iteration
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();

                    //For new edges
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge3;
                    newEdge4->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge4;

                    newEdge1->prev = intersectingEdge->prev;
                    newEdge1->next = newEdge4;
                    newEdge4->next = intersectingEdge->prev;
                    newEdge4->prev = newEdge1;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = intersectingEdge->next;

                    //For old edges
                    intersectingEdge->next->next = newCrossEdgeLeft;
                    intersectingEdge->prev->next = newEdge1;
                    intersectingEdge->prev->prev = newEdge4;
                    intersectingEdge->prev = newCrossEdgeLeft; 

                    //Edge to Particle relations
                    newEdge1->startParticle = this->particle_list[oldIndexRight];
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticleLeft;
                    newEdge4->startParticle = newParticleRight;

                    //Particle to Edge relations
                    newParticleLeft->edge = newEdge3;
                    newParticleRight->edge = newEdge4;

                    //Face to Particle/Edge relations
                    Face* newFace = new Face(oppIndex,oldIndexRight,newIndexRight);
                    newFace->edge = newEdge4->next;
                    intersectingFace->setFace(oppIndex,newIndexLeft,oldIndexLeft);
                    intersectingFace->edge = newCrossEdgeLeft;
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }

                    //Edge to Face relations
                    newEdge1->face = newFace;
                    newEdge1->next->face = newFace;
                    newEdge1->next->next->face = newFace;
                    newCrossEdgeLeft->face = intersectingFace;

                    //Adding the new edges and faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFace);
                }else if(nextType == 1){
                    //Last: Face, Current: Edge, Next: Edge
                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    newParticle = new Particle(get<0>(nextIntPt));
                    this->particle_list.push_back(newParticle);
                    int newIndex = this->particle_list.size()-1;

                    Edge* newEdge1 = rightSideEdge->twin;
                    Edge* newEdge2 = rightSideEdge;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    newSideEdgeRight = new Edge();
                    newSideEdgeLeft = this->edge_list[get<2>(nextIntPt)];
                    if(newSideEdgeLeft->face == intersectingFace){
                        newSideEdgeLeft = newSideEdgeLeft->twin;
                    }
                    bool left = (newSideEdgeLeft->twin == intersectingEdge->next);

                    if(left){
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = intersectingEdge;
                        newCrossEdgeLeft->prev = intersectingEdge->next;
                        newEdge2->next = newEdge5;
                        newEdge2->prev = intersectingEdge->prev;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge6;
                        newEdge5->next = intersectingEdge->prev;
                        newEdge5->prev = newEdge2;
                        newEdge6->next = newEdge3;
                        newEdge6->prev = newEdge7;
                        newEdge7->next = newEdge6;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newEdge2;
                        intersectingEdge->prev->prev = newEdge5;
                        intersectingEdge->prev = newCrossEdgeLeft;
                        intersectingEdge->next->next = newCrossEdgeLeft;

                        //Edge to Particle relations
                        newEdge1->startParticle = newParticleRight;
                        newEdge2->startParticle = this->particle_list[oldIndexRight];
                        newEdge3->startParticle = newParticleRight;
                        newEdge4->startParticle = newParticleLeft;
                        newEdge5->startParticle = newParticleRight;
                        newEdge6->startParticle = this->particle_list[oppIndex];
                        newEdge7->startParticle = newParticle;
                        newSideEdgeRight->startParticle = this->particle_list[oppIndex];
                        newSideEdgeLeft->startParticle = newParticle;
                        newCrossEdgeLeft->startParticle = newParticle;
                        newCrossEdgeRight->startParticle = newParticle;

                        //Particle to edge relations
                        newParticle->edge = newCrossEdgeLeft;
                        newParticleLeft->edge = intersectingEdge;
                        newParticleRight->edge = newEdge5;

                        //Face to Particle/Edge relations
                        Face* newFaceTop = new Face(newIndexRight,oppIndex,oldIndexRight);
                        newFaceTop->edge = newEdge5;
                        Face* newFaceBot = new Face(newIndexRight,newIndex,oppIndex);
                        newFaceBot->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, newIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceTop;
                        newEdge3->face = newFaceBot;
                        newEdge5->face = newFaceTop;
                        newEdge6->face = newFaceBot;
                        newEdge7->face = newFaceBot;
                        newCrossEdgeLeft->face = intersectingFace;
                        newEdge5->next->face = newFaceTop;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceTop);
                        this->face_list.push_back(newFaceBot);
                    }else{
                        //Edge to edge relations
                        newEdge1->twin = newEdge2;
                        newEdge2->twin = newEdge1;
                        newEdge3->twin = newCrossEdgeRight;
                        newCrossEdgeRight->twin = newEdge3;
                        newEdge4->twin = newCrossEdgeLeft;
                        newCrossEdgeLeft->twin = newEdge4;
                        newEdge5->twin = newEdge6;
                        newEdge6->twin = newEdge5;
                        newSideEdgeRight->twin = newEdge7;
                        newEdge7->twin = newSideEdgeRight;

                        newCrossEdgeLeft->next = newEdge6;
                        newCrossEdgeLeft->prev = intersectingEdge->prev;
                        newEdge2->next = newEdge3;
                        newEdge2->prev = newEdge7;
                        newEdge3->next = newEdge7;
                        newEdge3->prev = newEdge2;
                        newEdge5->next = intersectingEdge;
                        newEdge5->prev = intersectingEdge->next;
                        newEdge6->next = intersectingEdge->prev;
                        newEdge6->prev = newCrossEdgeLeft;
                        newEdge7->next = newEdge2;
                        newEdge7->prev = newEdge3;
                        intersectingEdge->prev->next = newCrossEdgeLeft;
                        intersectingEdge->prev->prev = newEdge6;
                        intersectingEdge->prev = newEdge5;
                        intersectingEdge->next->next = newEdge5;

                        //Edge to Particle relations
                        newEdge1->startParticle = newParticleRight;
                        newEdge2->startParticle = this->particle_list[oldIndexRight];
                        newEdge3->startParticle = newParticleRight;
                        newEdge4->startParticle = newParticleLeft;
                        newEdge5->startParticle = this->particle_list[oppIndex];
                        newEdge6->startParticle = newParticleLeft;
                        newEdge7->startParticle = newParticle;
                        newSideEdgeRight->startParticle = this->particle_list[oldIndexRight];
                        newSideEdgeLeft->startParticle = newParticle;
                        newCrossEdgeLeft->startParticle = newParticle;
                        newCrossEdgeRight->startParticle = newParticle;

                        //Particle to edge relations
                        newParticle->edge = newCrossEdgeLeft;
                        newParticleLeft->edge = intersectingEdge;
                        newParticleRight->edge = newEdge3;

                        //Face to Particle/Edge relations
                        Face* newFaceLeft = new Face(newIndexLeft,oppIndex,newIndex);
                        newFaceLeft->edge = newEdge6;
                        Face* newFaceRight = new Face(newIndexRight,newIndex,oldIndexRight);
                        newFaceRight->edge = newEdge3;
                        intersectingFace->setFace(oldIndexLeft, oppIndex, newIndexLeft);
                        intersectingFace->edge = intersectingEdge->next;
                        Face* oldRightFace = rightCrossEdge->twin->face;
                        for(int i=0;i<3;i++){
                            if(oldRightFace->indices[i] == newIndexLeft){
                                oldRightFace->indices[i] = newIndexRight;
                                break;
                            }
                        }

                        //Edge to Face relations
                        newEdge2->face = newFaceRight;
                        newEdge3->face = newFaceRight;
                        newEdge5->face = intersectingFace;
                        newEdge6->face = newFaceLeft;
                        newEdge7->face = newFaceRight;
                        newCrossEdgeLeft->face = newFaceLeft;
                        newEdge6->next->face = newFaceLeft;

                        //Adding the edges/faces
                        this->edge_list.push_back(newEdge3);
                        this->edge_list.push_back(newEdge4);
                        this->edge_list.push_back(newEdge5);
                        this->edge_list.push_back(newEdge6);
                        this->edge_list.push_back(newEdge7);
                        this->edge_list.push_back(newCrossEdgeLeft);
                        this->edge_list.push_back(newCrossEdgeRight);   
                        this->edge_list.push_back(newSideEdgeRight);
                        this->face_list.push_back(newFaceLeft);
                        this->face_list.push_back(newFaceRight);
                    }
                }else{
                    //Last: Face, Current: Edge, Next: Face
                    newParticle = this->particle_list[get<2>(nextIntPt)];
                    int newIndex = get<2>(nextIntPt);

                    rightCrossEdge->startParticle = newParticleRight;
                    rightSideEdge->twin->startParticle = newParticleRight;

                    //Edge to edge relations
                    newCrossEdgeLeft = new Edge();
                    newCrossEdgeRight = new Edge();
                    Edge* newEdge1 = rightSideEdge;
                    Edge* newEdge2 = rightSideEdge->twin;
                    Edge* newEdge3 = new Edge();
                    Edge* newEdge4 = new Edge();
                    Edge* newEdge5 = new Edge();
                    Edge* newEdge6 = new Edge();
                    Edge* newEdge7 = new Edge();
                    Edge* newEdge8 = new Edge();
                    Edge* newEdge9 = new Edge();
                    Edge* newEdge10 = new Edge();

                    //Twin relations
                    newEdge1->twin = newEdge2;
                    newEdge2->twin = newEdge1;
                    newEdge3->twin = newCrossEdgeRight;
                    newCrossEdgeRight->twin = newEdge3;
                    newEdge4->twin = newEdge5;
                    newEdge5->twin = newEdge4;
                    newEdge6->twin = newEdge7;
                    newEdge7->twin = newEdge6;
                    newEdge8->twin = newEdge9;
                    newEdge9->twin = newEdge8;
                    newEdge10->twin = newCrossEdgeLeft;
                    newCrossEdgeLeft->twin = newEdge10;

                    //Next/Prev relations
                    newEdge1->next = newEdge3;
                    newEdge1->prev = newEdge4;
                    newEdge3->next = newEdge4;
                    newEdge3->prev = newEdge1;
                    newEdge4->next = newEdge1;
                    newEdge4->prev = newEdge3;
                    newEdge5->next = newEdge6;
                    newEdge5->prev = intersectingEdge->prev;
                    newEdge6->next = intersectingEdge->prev;
                    newEdge6->prev = newEdge5;
                    newEdge7->next = newEdge8;
                    newEdge7->prev = intersectingEdge->next;
                    newEdge8->next = intersectingEdge->next;
                    newEdge8->prev = newEdge7;
                    newEdge9->next = newCrossEdgeLeft;
                    newEdge9->prev = intersectingEdge;
                    newCrossEdgeLeft->next = intersectingEdge;
                    newCrossEdgeLeft->prev = newEdge9;
                    intersectingEdge->prev->prev = newEdge6;
                    intersectingEdge->prev->next = newEdge5;
                    intersectingEdge->prev = newCrossEdgeLeft;
                    intersectingEdge->next->next = newEdge7;
                    intersectingEdge->next->prev = newEdge8;
                    intersectingEdge->next = newEdge9;

                    //Edge to Particle relations
                    newEdge1->startParticle = this->particle_list[oldIndexRight];
                    newEdge2->startParticle = newParticleRight;
                    newEdge3->startParticle = newParticleRight;
                    newEdge4->startParticle = newParticle;
                    newEdge5->startParticle = this->particle_list[oldIndexRight];
                    newEdge6->startParticle = newParticle;
                    newEdge7->startParticle = this->particle_list[oppIndex];
                    newEdge8->startParticle = newParticle;
                    newEdge9->startParticle = this->particle_list[oldIndexLeft];
                    newEdge10->startParticle = newParticleLeft;
                    newCrossEdgeLeft->startParticle = newParticle;
                    newCrossEdgeRight->startParticle = newParticle;

                    //Particle to Edge relations
                    newParticle->edge = newEdge4;
                    newParticleRight->edge = newEdge3;
                    newParticleLeft->edge = newEdge10;

                    //Face to Particle/Edge relations
                    Face* oldRightFace = rightCrossEdge->twin->face;
                    for(int i=0;i<3;i++){
                        if(oldRightFace->indices[i] == newIndexLeft){
                            oldRightFace->indices[i] = newIndexRight;
                            break;
                        }
                    }
                    intersectingFace->setFace(newIndexLeft,oldIndexLeft,newIndex);
                    intersectingFace->edge = intersectingEdge;
                    Face* newFaceBotLeft = new Face(newIndex,oldIndexLeft,oppIndex);
                    newFaceBotLeft->edge = newEdge8;
                    Face* newFaceBotRight = new Face(newIndex,oppIndex,oldIndexRight);
                    newFaceBotRight->edge = newEdge6;
                    Face* newFaceTopRight = new Face(newIndex,oldIndexRight,newIndexRight);
                    newFaceTopRight->edge = newEdge4;

                    //Edge to Face relations
                    newEdge1->face = newFaceTopRight;
                    newEdge2->face = oldRightFace;
                    newEdge3->face = newFaceTopRight;
                    newEdge4->face = newFaceTopRight;
                    newEdge5->face = newFaceBotRight;
                    newEdge6->face = newFaceBotRight;
                    newEdge6->next->face = newFaceBotRight;
                    newEdge7->face = newFaceBotLeft;
                    newEdge8->face = newFaceBotLeft;
                    newEdge8->next->face = newFaceBotLeft;
                    newEdge9->face = intersectingFace;
                    newCrossEdgeLeft->face = intersectingFace;
                    newCrossEdgeLeft->next->face = intersectingFace;

                    //Adding the new edges/faces
                    this->edge_list.push_back(newEdge3);
                    this->edge_list.push_back(newEdge4);
                    this->edge_list.push_back(newEdge5);
                    this->edge_list.push_back(newEdge6);
                    this->edge_list.push_back(newEdge7);
                    this->edge_list.push_back(newEdge8);
                    this->edge_list.push_back(newEdge9);
                    this->edge_list.push_back(newEdge10);
                    this->edge_list.push_back(newCrossEdgeLeft);
                    this->edge_list.push_back(newCrossEdgeRight);
                    this->face_list.push_back(newFaceBotLeft);
                    this->face_list.push_back(newFaceBotRight);
                    this->face_list.push_back(newFaceTopRight);
                }
            }else{
                //These cases are not possible
            }
        }
    }

    //Reallocating vertices and edges for next intersection point
    lastParticle = newParticle;
    leftCrossEdge = newCrossEdgeLeft;
    rightCrossEdge = newCrossEdgeRight;
    leftSideEdge = newSideEdgeLeft;
    rightSideEdge = newSideEdgeRight;
}
