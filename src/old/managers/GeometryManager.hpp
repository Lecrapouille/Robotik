#pragma once

#include <Eigen/Dense>
#include <vector>

namespace robotik::viewer
{

// ****************************************************************************
//! \brief Geometry rendering for primitive shapes.
//!
//! Handles rendering of basic 3D shapes: box, cylinder, sphere, grid, axes.
// ****************************************************************************
class GeometryRenderer
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    GeometryRenderer();

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~GeometryRenderer();

    // ------------------------------------------------------------------------
    //! \brief Initialize geometry buffers.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Render grid.
    // ------------------------------------------------------------------------
    void renderGrid() const;

    // ------------------------------------------------------------------------
    //! \brief Render box.
    //! \param p_transform Transform matrix.
    //! \param p_size Box dimensions.
    // ------------------------------------------------------------------------
    void renderBox(const Eigen::Matrix4f& p_transform,
                   const Eigen::Vector3f& p_size) const;

    // ------------------------------------------------------------------------
    //! \brief Render cylinder.
    //! \param p_transform Transform matrix.
    //! \param p_radius Cylinder radius.
    //! \param p_length Cylinder length.
    // ------------------------------------------------------------------------
    void renderCylinder(const Eigen::Matrix4f& p_transform,
                        float p_radius,
                        float p_length) const;

    // ------------------------------------------------------------------------
    //! \brief Render sphere.
    //! \param p_transform Transform matrix.
    //! \param p_radius Sphere radius.
    // ------------------------------------------------------------------------
    void renderSphere(const Eigen::Matrix4f& p_transform, float p_radius) const;

    // ------------------------------------------------------------------------
    //! \brief Render coordinate axes.
    //! \param p_transform Transform matrix.
    //! \param p_scale Scale factor.
    // ------------------------------------------------------------------------
    void renderAxes(const Eigen::Matrix4f& p_transform,
                    double p_scale = 1.0) const;

private:

    // ------------------------------------------------------------------------
    //! \brief Initialize box geometry buffers.
    // ------------------------------------------------------------------------
    void initializeBox();

    // ------------------------------------------------------------------------
    //! \brief Initialize cylinder geometry buffers.
    // ------------------------------------------------------------------------
    void initializeCylinder();

    // ------------------------------------------------------------------------
    //! \brief Initialize sphere geometry buffers.
    // ------------------------------------------------------------------------
    void initializeSphere();

    // ------------------------------------------------------------------------
    //! \brief Initialize grid geometry buffers.
    // ------------------------------------------------------------------------
    void initializeGrid();

    // ------------------------------------------------------------------------
    //! \brief Generate box geometry vertices and indices.
    // ------------------------------------------------------------------------
    void generateBox(std::vector<float>& vertices,
                     std::vector<unsigned int>& indices) const;

    // ------------------------------------------------------------------------
    //! \brief Generate cylinder geometry vertices and indices.
    // ------------------------------------------------------------------------
    void generateCylinder(std::vector<float>& vertices,
                          std::vector<unsigned int>& indices,
                          float radius,
                          float height,
                          int segments) const;

    // ------------------------------------------------------------------------
    //! \brief Generate sphere geometry vertices and indices.
    // ------------------------------------------------------------------------
    void generateSphere(std::vector<float>& vertices,
                        std::vector<unsigned int>& indices,
                        float radius,
                        int segments) const;

private:

    // Geometry buffers
    unsigned int m_box_vao = 0;
    unsigned int m_box_vbo = 0;
    unsigned int m_box_ebo = 0;
    unsigned int m_cylinder_vao = 0;
    unsigned int m_cylinder_vbo = 0;
    unsigned int m_cylinder_ebo = 0;
    unsigned int m_sphere_vao = 0;
    unsigned int m_sphere_vbo = 0;
    unsigned int m_sphere_ebo = 0;
    unsigned int m_grid_vao = 0;
    unsigned int m_grid_vbo = 0;

    // Index counts for geometry
    size_t m_cylinder_index_count = 0;
    size_t m_sphere_index_count = 0;
};

} // namespace robotik::viewer