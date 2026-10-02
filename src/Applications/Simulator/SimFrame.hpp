#pragma once

struct App;

namespace compages::world
{
struct ViewFrame;
}

//------------------------------------------------------------------------------
//! @brief Mouse input for the world panel, in panel pixels with Y up.
//! @param p_app The application.
//! @param p_elapsed The elapsed time.
//! @param p_total The total time.
//! @return The view frame.
//------------------------------------------------------------------------------
compages::world::ViewFrame
viewFrame(App const& p_app, float p_elapsed, float p_total);

//------------------------------------------------------------------------------
//! @brief Offscreen render from the wrist camera, readback, and color
//! detection.
//! @param p_app The application.
//------------------------------------------------------------------------------
void perceive(App& p_app);
