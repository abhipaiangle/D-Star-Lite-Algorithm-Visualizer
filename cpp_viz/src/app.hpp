#ifndef APP_HPP
#define APP_HPP

#include <piksel/baseapp.hpp>

class App : public piksel::BaseApp {
public:
    App() : piksel::BaseApp(1080-300, 1080-300) {}
    void setup();
    void draw(piksel::Graphics& g);
    void keyPressed(int key);
    
};
#endif /* APP_HPP */
