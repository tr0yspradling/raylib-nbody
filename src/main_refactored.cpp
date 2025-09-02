#include <iostream>
#include <stdexcept>
#include <raylib.h>
#include "core/Application.hpp"

int main() {
    try {
        nbody::core::Application app;
        app.run();
        return 0;
    } catch (const std::exception& e) {
        TraceLog(LOG_ERROR, "Exception: %s", e.what());
        std::cerr << "Application error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        TraceLog(LOG_ERROR, "Unknown exception occurred");
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
}