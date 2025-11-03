/**
 * @file GeometryManager.hpp
 * @brief Unified geometry-to-mesh management class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Renderer/Loaders/MeshLoader.hpp"

#include "Robotik/Core/Common/Path.hpp"

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik::renderer
{

// ****************************************************************************
//! \brief Unified geometry resource manager for a 3D rendering engine.
//!
//! This class manages all mesh resources (primitives and loaded files) for
//! rendering URDF robot geometries. It handles both CPU-side mesh data and
//! GPU-side OpenGL resources. Meshes are identified by unique names for
//! easy reference by kinematic tree nodes (links, joints, geometries).
// ****************************************************************************
class GeometryManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief GPU-side mesh data structure.
    //!
    //! Contains OpenGL buffer objects for rendering. This is created from
    //! CPUMesh data and uploaded to the GPU.
    // ------------------------------------------------------------------------
    struct GPUMesh
    {
        unsigned int vao = 0;   //!< Vertex Array Object
        unsigned int vbo = 0;   //!< Vertex Buffer Object
        unsigned int ebo = 0;   //!< Element Buffer Object
        size_t index_count = 0; //!< Number of indices to draw
        bool is_loaded = false; //!< Load status

        //! \brief Clear the mesh data
        void clear()
        {
            vao = 0;
            vbo = 0;
            ebo = 0;
            index_count = 0;
            is_loaded = false;
        }
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    explicit GeometryManager(Path const& p_path);

    // ------------------------------------------------------------------------
    //! \brief Destructor - frees all GPU resources.
    // ------------------------------------------------------------------------
    ~GeometryManager();

    // Prevent copying
    GeometryManager(const GeometryManager&) = delete;
    GeometryManager& operator=(const GeometryManager&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Load a mesh from file using a loader.
    //! \param p_name Unique name for the mesh (for kinematic tree reference).
    //! \param p_filename Path to the mesh file.
    //! \param p_loader Loader instance to use (e.g., STLLoader).
    //! \param p_force_reload Force reload even if already loaded.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool loadFromFile(const std::string& p_name,
                      const std::string& p_filename,
                      MeshLoader& p_loader,
                      bool p_force_reload = false);

    // ------------------------------------------------------------------------
    //! \brief Create a GPU mesh from a Geometry object.
    //! \param p_geom The geometry object to create mesh from.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createMeshFromGeometry(robotik::Geometry const& p_geom);

    // ------------------------------------------------------------------------
    //! \brief Create a box primitive mesh.
    //! \param p_name Unique name for the mesh.
    //! \param p_width Box width (X dimension).
    //! \param p_height Box height (Y dimension).
    //! \param p_depth Box depth (Z dimension).
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createBox(const std::string& p_name,
                   float p_width = 1.0f,
                   float p_height = 1.0f,
                   float p_depth = 1.0f);

    // ------------------------------------------------------------------------
    //! \brief Create a sphere primitive mesh.
    //! \param p_name Unique name for the mesh.
    //! \param p_radius Sphere radius.
    //! \param p_latitude_segments Number of latitude segments.
    //! \param p_longitude_segments Number of longitude segments.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createSphere(const std::string& p_name,
                      float p_radius = 1.0f,
                      size_t p_latitude_segments = 16,
                      size_t p_longitude_segments = 16);

    // ------------------------------------------------------------------------
    //! \brief Create a cylinder primitive mesh.
    //! \param p_name Unique name for the mesh.
    //! \param p_radius Cylinder radius.
    //! \param p_height Cylinder height.
    //! \param p_segments Number of segments around the cylinder.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createCylinder(const std::string& p_name,
                        float p_radius = 1.0f,
                        float p_height = 2.0f,
                        size_t p_segments = 16);

    // ------------------------------------------------------------------------
    //! \brief Create a grid mesh for ground plane visualization.
    //! \param p_name Unique name for the mesh.
    //! \param p_size Grid size (number of lines in each direction).
    //! \param p_spacing Spacing between grid lines.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createGrid(const std::string& p_name,
                    int p_size = 20,
                    float p_spacing = 1.0f);

    // ------------------------------------------------------------------------
    //! \brief Get a mesh by name.
    //! \param p_name Name of the mesh.
    //! \return Pointer to GPU mesh, nullptr if not found.
    // ------------------------------------------------------------------------
    const GPUMesh* getMesh(const std::string& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Check if a mesh is loaded.
    //! \param p_name Name of the mesh.
    //! \return true if mesh exists and is loaded.
    // ------------------------------------------------------------------------
    bool hasMesh(const std::string& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Unload a specific mesh and free its GPU resources.
    //! \param p_name Name of the mesh to unload.
    //! \return true if mesh was found and unloaded.
    // ------------------------------------------------------------------------
    bool unloadMesh(const std::string& p_name);

    // ------------------------------------------------------------------------
    //! \brief Clear all meshes and free all GPU resources.
    // ------------------------------------------------------------------------
    void clear();

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message string.
    // ------------------------------------------------------------------------
    const std::string& error() const
    {
        return m_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Create OpenGL buffers from CPU mesh data.
    //! \param p_cpu_mesh CPU-side mesh data.
    //! \param p_gpu_mesh Output GPU-side mesh.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createGPUMesh(const MeshLoader::CPUMesh& p_cpu_mesh,
                       GPUMesh& p_gpu_mesh) const;

    // ------------------------------------------------------------------------
    //! \brief Free GPU resources for a mesh.
    //! \param p_gpu_mesh GPU mesh to free.
    // ------------------------------------------------------------------------
    void freeGPUMesh(GPUMesh& p_gpu_mesh) const;

    // Primitive generation methods (from GeometryRenderer)
    // ------------------------------------------------------------------------

    //! \brief Generate box vertices and indices
    void generateBox(MeshLoader::CPUMesh& p_mesh,
                     float p_width,
                     float p_height,
                     float p_depth) const;

    //! \brief Generate sphere vertices and indices
    void generateSphere(MeshLoader::CPUMesh& p_mesh,
                        float p_radius,
                        size_t p_latitude_segments,
                        size_t p_longitude_segments) const;

    //! \brief Generate cylinder vertices and indices
    void generateCylinder(MeshLoader::CPUMesh& p_mesh,
                          float p_radius,
                          float p_height,
                          size_t p_segments) const;

    //! \brief Generate grid vertices (no indices, uses GL_LINES)
    void generateGrid(std::vector<float>& p_vertices,
                      int p_size,
                      float p_spacing) const;

private:

    //! \brief Path to the meshes directory
    Path const& m_path;
    //! \brief Storage for all GPU meshes (name -> GPUMesh)
    std::unordered_map<std::string, GPUMesh> m_meshes;
    //! \brief Last error message
    std::string m_error;
};

} // namespace robotik::renderer
