// ============================================================
// robot.h — Robot arm definitie: joints, links, forward kinematics
// ============================================================
#pragma once

#include "gl_utils.h"
#include <vector>
#include <string>

// Rotatie-as van een joint
enum class Axis { X, Y, Z };

// Beschrijft één roterende joint + de link die erop volgt
struct RobotJoint {
    std::string name;

    // Joint properties
    Axis   axis         = Axis::Y;
    float  minAngle     = -180.0f;  // graden
    float  maxAngle     =  180.0f;
    float  currentAngle = 0.0f;     // graden
    float  defaultAngle = 0.0f;

    // Offset van parent joint naar deze joint (in parent's frame)
    Vector3f offset = {0, 0, 0};

    // Visualisatie van de link NA deze joint
    float linkLength = 0.0f;
    float linkRadius = 0.06f;

    // Kleur (RGB 0-1)
    float color[3] = {0.4f, 0.6f, 0.9f};

    // Berekende world transformatie (na FK update)
    Matrix4f worldTransform = Matrix4f::Identity();

    // Child joints
    std::vector<RobotJoint> children;

    /// Voeg een child joint toe en geef referentie terug
    RobotJoint& addChild(const std::string& name) {
        children.push_back(RobotJoint());
        children.back().name = name;
        return children.back();
    }
};

// ============================================================
// Robot class — bevat de hele kinematic chain + rendering
// ============================================================
class Robot {
public:
    Robot();
    ~Robot() = default;

    /// Reset alle joints naar default posities
    void resetJoints(RobotJoint& joint);

    /// Forward kinematics: update alle worldTransforms recursief
    void updateFK(RobotJoint& joint, const Matrix4f& parentTransform);

    /// Teken de hele robot (view en proj worden doorgegeven)
    void draw(GLuint program, const Matrix4f& view, const Matrix4f& proj,
              RobotJoint& joint);

    /// Upload geometry naar GPU
    void initMeshes();

    /// convenience
    void resetAllJoints() { resetJoints(base); }
    void updateAllFK()    { updateFK(base, Matrix4f::Identity()); }
    void drawAll(GLuint program, const Matrix4f& view, const Matrix4f& proj) {
        draw(program, view, proj, base);
    }

    /// Verzamel alle joints plat in een vector (voor UI sliders)
    std::vector<RobotJoint*> flattenJoints(RobotJoint& joint);
    std::vector<RobotJoint*> getAllJoints() { return flattenJoints(base); }

    /// De root joint (base)
    RobotJoint base;

private:
    Mesh cylinderMesh;
    Mesh sphereMesh;
    Mesh boxMesh;
    bool meshesReady = false;
};
