// ============================================================
// robot.cpp — Robot implementatie: FK update, rendering, mesh setup
// ============================================================
#include "robot.h"
#include <cmath>

// ============================================================
// Constructor: definieer een 6-DOF robot arm
//
// Structuur (zoals een typische industriële robot):
//
//   Base (vast) — grijze cilinder op de vloer
//   ├── Joint 1 "Shoulder Pan"   — Y as (draaien)
//   │   └── Joint 2 "Shoulder Lift" — Z as (omhoog/omlaag)
//   │       └── Joint 3 "Elbow"     — Z as (buigen)
//   │           └── Joint 4 "Wrist Pitch" — Z as
//   │               └── Joint 5 "Wrist Roll"  — Y as
//   │                   └── Joint 6 "Tool Rotation" — X as
//   │                       └── [End effector gripper]
//
// ============================================================

Robot::Robot() {
    base.name = "Base";
    base.axis = Axis::Y;
    base.minAngle = -180.0f;
    base.maxAngle = 180.0f;
    base.currentAngle = 0.0f;
    base.linkRadius = 0.18f;
    base.color[0] = 0.45f; base.color[1] = 0.45f; base.color[2] = 0.45f;

    // Joint 1: Base rotation (Y-axis)
    auto& j1 = base.addChild("Shoulder Pan");
    j1.axis = Axis::Y;
    j1.minAngle = -180.0f;
    j1.maxAngle = 180.0f;
    j1.offset = {0, 0.25f, 0};
    j1.linkLength = 0.25f;
    j1.linkRadius = 0.10f;
    j1.color[0] = 0.85f; j1.color[1] = 0.35f; j1.color[2] = 0.15f;

    // Joint 2: Shoulder pitch (Z-axis)
    auto& j2 = j1.addChild("Shoulder Lift");
    j2.axis = Axis::Z;
    j2.minAngle = -90.0f;
    j2.maxAngle = 120.0f;
    j2.defaultAngle = 0.0f;
    j2.offset = {0, 0.25f, 0};
    j2.linkLength = 1.2f;
    j2.linkRadius = 0.08f;
    j2.color[0] = 0.20f; j2.color[1] = 0.55f; j2.color[2] = 0.90f;

    // Joint 3: Elbow (Z-axis)
    auto& j3 = j2.addChild("Elbow");
    j3.axis = Axis::Z;
    j3.minAngle = -135.0f;
    j3.maxAngle = 135.0f;
    j3.defaultAngle = -45.0f;
    j3.offset = {0, 1.2f, 0};
    j3.linkLength = 1.0f;
    j3.linkRadius = 0.07f;
    j3.color[0] = 0.20f; j3.color[1] = 0.70f; j3.color[2] = 0.40f;

    // Joint 4: Wrist pitch (Z-axis)
    auto& j4 = j3.addChild("Wrist Pitch");
    j4.axis = Axis::Z;
    j4.minAngle = -180.0f;
    j4.maxAngle = 180.0f;
    j4.offset = {0, 1.0f, 0};
    j4.linkLength = 0.3f;
    j4.linkRadius = 0.055f;
    j4.color[0] = 0.80f; j4.color[1] = 0.75f; j4.color[2] = 0.15f;

    // Joint 5: Wrist roll (Y-axis)
    auto& j5 = j4.addChild("Wrist Roll");
    j5.axis = Axis::Y;
    j5.minAngle = -180.0f;
    j5.maxAngle = 180.0f;
    j5.offset = {0, 0.3f, 0};
    j5.linkLength = 0.15f;
    j5.linkRadius = 0.04f;
    j5.color[0] = 0.70f; j5.color[1] = 0.30f; j5.color[2] = 0.70f;

    // Joint 6: Tool rotation (X-axis)
    auto& j6 = j5.addChild("Tool Rotation");
    j6.axis = Axis::X;
    j6.minAngle = -180.0f;
    j6.maxAngle = 180.0f;
    j6.offset = {0, 0.15f, 0};
    j6.linkLength = 0.0f;
    j6.linkRadius = 0.035f;
    j6.color[0] = 0.90f; j6.color[1] = 0.90f; j6.color[2] = 0.20f;

    // Initialiseer alle angles op default
    resetAllJoints();
}

// ============================================================
// Joint controls
// ============================================================

void Robot::resetJoints(RobotJoint& joint) {
    joint.currentAngle = joint.defaultAngle;
    for (auto& child : joint.children) {
        resetJoints(child);
    }
}

// ============================================================
// Forward Kinematics
// ============================================================

void Robot::updateFK(RobotJoint& joint, const Matrix4f& parentTransform) {
    float rad = joint.currentAngle * M_PI / 180.0f;
    Matrix4f rotation;
    switch (joint.axis) {
        case Axis::X: rotation = rotationX(rad); break;
        case Axis::Y: rotation = rotationY(rad); break;
        case Axis::Z: rotation = rotationZ(rad); break;
    }

    // Offset naar de joint positie (in parent frame)
    Matrix4f offsetMat = translationMatrix(joint.offset.x(), joint.offset.y(), joint.offset.z());

    // World transformatie = parent × offset × rotatie
    Matrix4f localTransform = parentTransform * offsetMat * rotation;
    joint.worldTransform = localTransform;

    for (auto& child : joint.children) {
        updateFK(child, localTransform);
    }
}

// ============================================================
// Mesh initialisatie
// ============================================================

void Robot::initMeshes() {
    if (meshesReady) return;
    cylinderMesh = makeCylinder(1.0f, 1.0f, 1.0f, 24);
    cylinderMesh.upload();
    sphereMesh = makeSphere(1.0f, 16, 24);
    sphereMesh.upload();
    boxMesh = makeBox(1.0f, 1.0f, 1.0f);
    boxMesh.upload();
    meshesReady = true;
}

// ============================================================
// Robot drawing
// ============================================================
//
// Elke link wordt getekend als een cilinder VAN deze joint (y=0)
// TOT de positie van de child joint. De child's offset in Y
// bepaalt de link lengte. De cilinder is gecentreerd op de
// midden van de link (translate naar y = offset/2, dan scale).
//
// Joint bollen worden getekend op y=0 (deze joint positie)
// EN op y=offset (de child joint positie).
// ============================================================

void Robot::draw(GLuint program, const Matrix4f& view, const Matrix4f& proj,
                 RobotJoint& joint) {
    initMeshes();

    // ── Base: grote grijze cilinder op de vloer ──
    if (joint.name == "Base") {
        Matrix4f baseModel = joint.worldTransform
                           * translationMatrix(0, 0.12f, 0)
                           * scaleMatrix(0.18f, 0.24f, 0.18f);
        setUniforms(program, baseModel, view, proj,
                    joint.color[0], joint.color[1], joint.color[2]);
        cylinderMesh.draw();
    }

    // ── Voor elke child: teken link + bol op child joint ──
    for (auto& child : joint.children) {
        // De link lengte = de Y-offset van de child (in parent frame)
        float linkLen = child.offset.y();
        if (linkLen < 0.001f) linkLen = 0.001f;

        // Cilinder: gecentreerd op midden van de link
        // Cylinder mesh gaat van y=0 naar y=1, na scale van y=0 naar y=linkLen.
        // Geen translatie nodig — begint bij joint, eindigt bij child.
        float cylRadius = child.linkRadius;
        Matrix4f linkModel = joint.worldTransform
                           * scaleMatrix(cylRadius, linkLen, cylRadius);
        setUniforms(program, linkModel, view, proj,
                    joint.color[0], joint.color[1], joint.color[2]);
        cylinderMesh.draw();

        // Bol op de child joint positie (bovenkant van de link)
        float sphereR = cylRadius * 1.4f;
        Matrix4f sphereModel = joint.worldTransform
                            * translationMatrix(0, linkLen, 0)
                            * scaleMatrix(sphereR, sphereR, sphereR);
        setUniforms(program, sphereModel, view, proj, 0.95f, 0.25f, 0.25f);
        sphereMesh.draw();

        // Recursief de child tekenen
        draw(program, view, proj, child);
    }

    // ── End effector (gripper) op leaf joints ──
    if (joint.children.empty() && joint.name != "Base") {
        Matrix4f tool = joint.worldTransform;
        // Gripper basis
        setUniforms(program,
                    tool * translationMatrix(0, 0.03f, 0)
                        * scaleMatrix(0.10f, 0.04f, 0.06f),
                    view, proj, 0.7f, 0.7f, 0.7f);
        boxMesh.draw();
        // Gripper vinger links
        setUniforms(program,
                    tool * translationMatrix(-0.05f, 0.09f, 0)
                        * scaleMatrix(0.02f, 0.10f, 0.035f),
                    view, proj, 0.95f, 0.95f, 0.25f);
        boxMesh.draw();
        // Gripper vinger rechts
        setUniforms(program,
                    tool * translationMatrix( 0.05f, 0.09f, 0)
                        * scaleMatrix(0.02f, 0.10f, 0.035f),
                    view, proj, 0.95f, 0.95f, 0.25f);
        boxMesh.draw();
    }
}

// ============================================================
// Flatten joints tree
// ============================================================

std::vector<RobotJoint*> Robot::flattenJoints(RobotJoint& joint) {
    std::vector<RobotJoint*> result;
    result.push_back(&joint);
    for (auto& child : joint.children) {
        auto childJoints = flattenJoints(child);
        result.insert(result.end(), childJoints.begin(), childJoints.end());
    }
    return result;
}
