#include "../include/path.hpp"

Path::Path(){
    this->size = 0; 
    this->currentCurve = 0;
    this->currParamVal = 0.0f; 
}        
void Path::addCurve(vector<vec3> inputPts){
    this->curves.push_back(new Curve(inputPts,0));
    if(size == 0){
        this->lastPoint = this->curves[0]->getPoint(0);
        this->lastTangent = this->curves[0]->getTangent(0);
    }   
    this->size++;
}
void Path::updatePath(){
    if(currentCurve < size){
        // this->currParamVal += 1.5f;
        this->currParamVal += 0.20f;
        if(double_gt(this->currParamVal,1.0f)){
            this->lastPoint = this->curves[currentCurve]->getPoint(1.0f);
            this->lastTangent = this->curves[currentCurve]->getTangent(1.0f);
            this->currParamVal = 0.0f;
            this->currentCurve++;
            if(currentCurve == size){
                this->isOver = true;
            }
        }else{
            this->lastPoint = this->curves[currentCurve]->getPoint(this->currParamVal);
            this->lastTangent = this->curves[currentCurve]->getTangent(this->currParamVal);
        }
    }
} 