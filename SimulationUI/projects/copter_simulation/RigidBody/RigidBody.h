#pragma once
#include "Eigen/Dense"

namespace copter_simulation {

class RigidBody {
public:
    RigidBody();
    ~RigidBody() {}
    
    /* get/set functions */
    double mass() const { return _mass; }
    Eigen::Matrix3d inertia_body() const { return _inertia_body; }
    Eigen::Matrix3d inertia_body_inv() const { return _inertia_body_inv; }
    Eigen::Vector3d position() const { return _position; }
    Eigen::Vector3d velocity() const { return _velocity; }
    Eigen::Vector3d acceleration() const { return _acceleration; }
    Eigen::Quaterniond orientation() const { return _orientation; }
    Eigen::Vector3d omega() const { return _omega; }

    void TouchGround(double height) {
        Initialize(Eigen::Vector3d(_position(0), _position(1), height),
            Eigen::Vector3d::Zero(),
            _orientation, // Eigen::Quaterniond(Eigen::AngleAxisd(_orientation.toRotationMatrix().eulerAngles(0, 1, 2)(2), Eigen::Vector3d::UnitZ())),
            Eigen::Vector3d::Zero()
        );
    }

    /* Initialization functions */
    void Initialize(const double mass, const Eigen::Matrix3d& inertia_body,
        const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& angular_velocity);
    void Initialize(const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, 
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& angular_velocity);

    /* Simulation Functions */
    void ApplyForceInWorldFrame(const Eigen::Vector3d& force, const Eigen::Vector3d& position);
    void ApplyForceInBodyFrame(const Eigen::Vector3d& force, const Eigen::Vector3d& position);
    /* Apply Torque in World Frame */
    void ApplyTorque(const Eigen::Vector3d& torque);
    void Clear();
    void Advance(const double dt);

    /* Get Functions */
    const Eigen::Vector3d external_force() const {
        return _external_force;
    }
    const Eigen::Vector3d external_torque() const {
        return _external_torque;
    }

    /* Point, Vector Transformations */
    const Eigen::Vector3d PointWorldToBody(
        const Eigen::Vector3d& world_point) const;
    const Eigen::Matrix3Xd PointsWorldToBody(
        const Eigen::Matrix3Xd& world_points) const;
    const Eigen::Vector3d PointBodyToWorld(
        const Eigen::Vector3d& body_point) const;
    const Eigen::Matrix3Xd PointsBodyToWorld(
        const Eigen::Matrix3Xd& body_points) const;
    const Eigen::Vector3d VectorWorldToBody(
        const Eigen::Vector3d& world_vector) const;
    const Eigen::Matrix3Xd VectorsWorldToBody(
        const Eigen::Matrix3Xd& world_vectors) const;
    const Eigen::Vector3d VectorBodyToWorld(
        const Eigen::Vector3d& body_vector) const;
    const Eigen::Matrix3Xd VectorsBodyToWolrd(
        const Eigen::Matrix3Xd& body_vectors) const;

    /* Transform Matrix */
    const Eigen::Matrix4d BodyToWorldTransform() const;
    const Eigen::Matrix4d WorldToBodyTransform() const;

    /* Common Tools */
    Eigen::Quaterniond Add(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
    Eigen::Quaterniond Multiply(const Eigen::Quaterniond& q, const double scale);

private:
    /* Constant Quantities */
    double _mass;
    Eigen::Matrix3d _inertia_body;
    Eigen::Matrix3d _inertia_body_inv;

    /* State Variables */
    Eigen::Vector3d _position;
    Eigen::Quaterniond _orientation;
    Eigen::Vector3d _linear_momentum;
    Eigen::Vector3d _angular_momentum;

    /* Derived Quantities */
    Eigen::Matrix3d _inertia_inv;
    Eigen::Matrix3d _R; // Rotation Matrix
    Eigen::Vector3d _velocity;
    Eigen::Vector3d _acceleration;
    Eigen::Vector3d _omega;
    
    /* External Input */
    Eigen::Vector3d _external_force;
    Eigen::Vector3d _external_torque;
};

}