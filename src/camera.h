// ============================================================
// camera.h — Orbit camera voor ImGui/OpenGL (Eigen-based, geen glm)
// ============================================================
#pragma once

#include "gl_utils.h"
#include <algorithm>

class OrbitCamera {
public:
    float distance  = 8.0f;
    float yaw       = 45.0f;   // graden
    float pitch     = 25.0f;   // graden
    Vector3f target = {0.0f, 1.5f, 0.0f};

    // Muis state
    bool isDragging = false;
    bool isPanning  = false;
    float lastMouseX = 0.0f;
    float lastMouseY = 0.0f;

    // Snelheid
    float rotateSpeed = 0.4f;
    float panSpeed    = 0.01f;
    float zoomSpeed   = 0.8f;

    /// Bereken view matrix
    Matrix4f getViewMatrix() const {
        Vector3f eye = getEyePosition();
        return lookAtMatrix(eye, target, Vector3f(0, 1, 0));
    }

    /// Camera positie in world space
    Vector3f getEyePosition() const {
        float yawRad   = yaw   * (float)M_PI / 180.0f;
        float pitchRad = pitch * (float)M_PI / 180.0f;

        Vector3f eye;
        eye.x() = target.x() + distance * std::cos(pitchRad) * std::sin(yawRad);
        eye.y() = target.y() + distance * std::sin(pitchRad);
        eye.z() = target.z() + distance * std::cos(pitchRad) * std::cos(yawRad);
        return eye;
    }

    /// Handelt muis input af
    void handleInput(float mouseX, float mouseY, bool leftDown, bool rightDown, float wheel) {
        // Zoom
        if (wheel != 0.0f) {
            distance -= wheel * zoomSpeed;
            distance = std::clamp(distance, 1.0f, 50.0f);
        }

        // Rotatie (linker muis knop)
        if (leftDown && !isPanning) {
            if (!isDragging) {
                isDragging = true;
                lastMouseX = mouseX;
                lastMouseY = mouseY;
            } else {
                float dx = mouseX - lastMouseX;
                float dy = mouseY - lastMouseY;
                yaw   -= dx * rotateSpeed;
                pitch += dy * rotateSpeed;
                pitch = std::clamp(pitch, -89.0f, 89.0f);
                lastMouseX = mouseX;
                lastMouseY = mouseY;
            }
        } else {
            isDragging = false;
        }

        // Pan (rechter muis knop)
        if (rightDown && !isDragging) {
            if (!isPanning) {
                isPanning = true;
                lastMouseX = mouseX;
                lastMouseY = mouseY;
            } else {
                float dx = mouseX - lastMouseX;
                float dy = mouseY - lastMouseY;

                float yawRad = yaw * (float)M_PI / 180.0f;
                Vector3f right = Vector3f(std::cos(yawRad), 0, -std::sin(yawRad)).normalized();
                Vector3f up(0, 1, 0);

                target -= right * dx * panSpeed * distance * 0.1f;
                target += up    * dy * panSpeed * distance * 0.1f;
                lastMouseX = mouseX;
                lastMouseY = mouseY;
            }
        } else {
            isPanning = false;
        }
    }

    void reset() {
        distance = 8.0f;
        yaw      = 45.0f;
        pitch    = 25.0f;
        target   = {0.0f, 1.5f, 0.0f};
    }
};
