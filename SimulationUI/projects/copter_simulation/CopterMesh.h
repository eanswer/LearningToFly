#pragma once
#include "Mesh.h"
#include "CopterSimulationCommon.h"

namespace copter_simulation {

class CopterMesh
{
public:
    CopterMesh();
    ~CopterMesh();

    std::vector<copter_utils::SurfaceMesh>& body_mesh() { return _body_mesh; }
    copter_utils::SurfaceMesh& body_mesh(int idx) { return _body_mesh[idx]; }
    std::vector<float>& body_mesh_mass() { return _body_mesh_mass; }
    float body_mesh_mass(int idx) { return _body_mesh_mass[idx]; }

    void Scale(float scale_factor);
    void Translate(Eigen::Vector3f v);
    Eigen::Vector3f Center();
    float Mass();
    copter_utils::BoundingBox BoundingBox();

private:
    std::vector<copter_utils::SurfaceMesh> _body_mesh;
    std::vector<float> _body_mesh_mass;
};

}