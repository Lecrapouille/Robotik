#include "App.hpp"
#include "MainLoop.hpp"
#include "Window.hpp"

#include <filesystem>
#include <iostream>

int main(int argc, char** argv)
{
    Window window;
    if (!window.open())
    {
        std::cerr << "Cannot open an OpenGL 4.5 window\n";
        return 1;
    }

    App app;
    app.scenario_path =
        (argc > 1) ? std::filesystem::path(argv[1])
                   : std::filesystem::path("data/scenarios/pick_and_place.yml");
    app.load();

    runMainLoop(window, app);
    return 0;
}
