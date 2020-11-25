#ifndef APP_HPP
#define APP_HPP

#include <piksel/baseapp.hpp>

class App : public piksel::BaseApp {
public:
    App() : piksel::BaseApp(1280, 900-200) {}
    void setup();
    void draw(piksel::Graphics& gcanvas);
    void keyPressed(int key);
    void mousePressed(int button);
    void mouseMoved(int x, int y);
    
};
#endif /* APP_HPP */
