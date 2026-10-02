#pragma once

struct App;
struct Window;

// Event poll, simulation step, 3D views, ImGui composite, swap buffers.
void runMainLoop(Window& p_window, App& p_app);
