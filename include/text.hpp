#ifndef TEXT_HPP
#define TEXT_HPP

#include "common.hpp"
#include <string>

class Text {
public:
    GLuint fontOffset;
    void initialize();
    void draw(std::string, float x, float y);
};

#endif
