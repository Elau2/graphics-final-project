// main.cpp
//
// Entry point. Very thin: parse minimal CLI, build an Application, run.
//
// CLI:
//   destruct                         // default cube, 20 Voronoi fragments
//   destruct path/to/mesh.obj        // use OBJ file
//   destruct path/to/mesh.obj 35     // OBJ + fragment count

#include "app/Application.h"

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv)
{
    destruct::Application::Config cfg;

    if (argc >= 2) cfg.objPath = argv[1];
    if (argc >= 3) {
        int n = std::atoi(argv[2]);
        if (n >= 4 && n <= 300) cfg.initialFragments = n;
    }

    destruct::Application app(std::move(cfg));
    return app.run();
}
