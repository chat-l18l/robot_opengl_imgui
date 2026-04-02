// ============================================================
// main.cpp — ImGui + OpenGL3 Robot Viewer
//
// Renders een 6-DOF robot arm in een ImGui viewport.
// Joints kunnen via sliders worden ingesteld.
// Muis: links-drag=rotatie, rechts-drag=pan, scroll=zoom
//
// Dependencies: Eigen3, GLFW3, OpenGL 3.3+
// Build: cmake + make (zie CMakeLists.txt)
// ============================================================

#include "gl_core.h"
#include "gl_utils.h"
#include "robot.h"
#include "camera.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <cmath>

// ============================================================
// Vertex & Fragment Shaders (inline, geen externe bestanden)
// ============================================================

static const char* VERTEX_SHADER = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

out vec3 FragPos;
out vec3 Normal;

void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    FragPos = worldPos.xyz;
    Normal  = normalize(uNormalMatrix * aNormal);
    gl_Position = uProjection * uView * worldPos;
}
)";

static const char* FRAGMENT_SHADER = R"(
#version 330 core

in vec3 FragPos;
in vec3 Normal;

uniform vec3 uColor;
uniform vec3 uLightPos;
uniform vec3 uViewPos;

out vec4 FragColor;

void main() {
    // Ambient
    float ambientStrength = 0.35f;
    vec3 ambient = ambientStrength * uColor;

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(uLightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * uColor * 0.8f;

    // Specular (Blinn-Phong)
    vec3 viewDir = normalize(uViewPos - FragPos);
    vec3 halfDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfDir), 0.0), 32.0);
    vec3 specular = spec * vec3(0.3f);

    // Tweede lichtbron (van achter, fill light)
    vec3 lightDir2 = normalize(vec3(-3.0, 5.0, -5.0) - FragPos);
    float diff2 = max(dot(norm, lightDir2), 0.0);
    vec3 diffuse2 = diff2 * uColor * 0.25f;

    // Outline effect voor robot look
    float edgeFactor = 1.0 - abs(dot(norm, viewDir));
    vec3 rimColor = vec3(0.1, 0.1, 0.15) * pow(edgeFactor, 3.0) * 0.5f;

    vec3 result = ambient + diffuse + diffuse2 + specular + rimColor;
    FragColor = vec4(result, 1.0);
}
)";

// ============================================================
// Global state
// ============================================================

static GLuint  gShaderProgram = 0;
static Robot   gRobot;
static OrbitCamera gCamera;
static Mesh    gGridMesh;
static bool    gShowGrid = true;
static bool    gShowAxes = true;
static bool    gWireframe = false;

// ============================================================
// OpenGL setup
// ============================================================

static void initOpenGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.12f, 0.13f, 0.16f, 1.0f);

    // Shader
    gShaderProgram = createProgramFromSource(VERTEX_SHADER, FRAGMENT_SHADER);
    if (!gShaderProgram) {
        std::cerr << "FATAL: Shader compilation failed!" << std::endl;
        std::exit(1);
    }

    // Grid
    gGridMesh = makeGrid(10.0f, 20);
    gGridMesh.upload();
}

// ============================================================
// 3D Viewport rendering (binnen ImGui window)
// ============================================================

/// Bewaar viewport state voordat ImGui het window opbouwt
struct ViewportInfo {
    ImVec2 pos;
    ImVec2 size;
    bool   isHovered;
};

static ViewportInfo render3DViewport() {
    ViewportInfo vp;
    vp.pos = ImVec2(0, 0);
    vp.size = ImVec2(0, 0);
    vp.isHovered = false;

    // ImGui window flags: geen scrollbar, geen background (we tekenen zelf 3D)
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar
                           | ImGuiWindowFlags_NoScrollWithMouse
                           | ImGuiWindowFlags_NoBackground;

    ImGui::Begin("Robot 3D View", nullptr, flags);
    vp.pos = ImGui::GetCursorScreenPos();
    vp.size = ImGui::GetContentRegionAvail();
    vp.isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);

    // Voorkom nul-grootte
    if (vp.size.x < 1 || vp.size.y < 1) {
        ImGui::End();
        return vp;
    }

    // ── Viewport instellen (scherm-coördinaten → GL viewport) ──
    // ImGui y=0 is bovenaan, GL y=0 is onderaan
    float fbHeight = ImGui::GetIO().DisplaySize.y;
    GLint glX      = (GLint)vp.pos.x;
    GLint glY      = (GLint)(fbHeight - vp.pos.y - vp.size.y);
    GLint glW      = (GLint)vp.size.x;
    GLint glH      = (GLint)vp.size.y;

    glViewport(glX, glY, glW, glH);
    glScissor(glX, glY, glW, glH);
    glEnable(GL_SCISSOR_TEST);

    // Clear deze viewport regio (zowel kleur als depth)
    glClearColor(0.09f, 0.10f, 0.13f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ── Projection ──
    float aspect = vp.size.x / vp.size.y;
    Matrix4f proj = perspectiveMatrix(45.0f, aspect, 0.1f, 100.0f);

    // ── View ──
    Matrix4f view = gCamera.getViewMatrix();
    Vector3f eyePos = gCamera.getEyePosition();

    // ── Shader instellen ──
    glUseProgram(gShaderProgram);
    GLuint loc;

    loc = glGetUniformLocation(gShaderProgram, "uLightPos");
    if (loc >= 0) glUniform3f(loc, 5.0f, 8.0f, 5.0f);
    loc = glGetUniformLocation(gShaderProgram, "uViewPos");
    if (loc >= 0) glUniform3f(loc, eyePos.x(), eyePos.y(), eyePos.z());

    // Wireframe?
    if (gWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // ── Grid tekenen ──
    if (gShowGrid) {
        Matrix4f gridModel = Matrix4f::Identity();
        setUniforms(gShaderProgram, gridModel, view, proj, 0.25f, 0.25f, 0.28f);
        gGridMesh.draw();
    }

    // ── Robot tekenen ──
    gRobot.drawAll(gShaderProgram, view, proj);

    // Wireframe terugzetten
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glDisable(GL_SCISSOR_TEST);

    // ── Overlay info ──
    ImGui::SetCursorPos(ImVec2(10, 10));
    ImGui::TextColored(ImVec4(0.7f, 0.8f, 1.0f, 0.8f),
                       "LMB: Rotate | RMB: Pan | Scroll: Zoom");

    ImGui::End();

    return vp;
}

// ============================================================
// Control Panel UI (rechterkant)
// ============================================================

static void renderControlPanel() {
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 320, 0),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(310, ImGui::GetIO().DisplaySize.y),
                             ImGuiCond_FirstUseEver);

    ImGui::Begin("Joint Controls", nullptr, 0);

    // ── Robot controls ──
    if (ImGui::CollapsingHeader("Robot", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Reset All Joints")) {
            gRobot.resetAllJoints();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Camera")) {
            gCamera.reset();
        }
    }

    // ── Joint sliders ──
    auto joints = gRobot.getAllJoints();

    if (ImGui::CollapsingHeader("Joints", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (size_t i = 0; i < joints.size(); i++) {
            auto* j = joints[i];

            // Kleur indicator
            ImGui::PushStyleColor(ImGuiCol_Text,
                ImVec4(j->color[0], j->color[1], j->color[2], 1.0f));

            // As label
            const char* axisLabel = "?";
            switch (j->axis) {
                case Axis::X: axisLabel = "X"; break;
                case Axis::Y: axisLabel = "Y"; break;
                case Axis::Z: axisLabel = "Z"; break;
            }

            // Header
            ImGui::Separator();
            ImGui::Text("%s [%s]", j->name.c_str(), axisLabel);

            ImGui::PopStyleColor();

            // Slider (graden)
            float angle = j->currentAngle;
            if (ImGui::SliderFloat(
                    ("##angle" + std::to_string(i)).c_str(),
                    &angle, j->minAngle, j->maxAngle, "%.1f°")) {
                j->currentAngle = angle;
            }

            // Reset deze joint
            ImGui::SameLine();
            if (ImGui::SmallButton(("R##" + std::to_string(i)).c_str())) {
                j->currentAngle = j->defaultAngle;
            }
        }
    }

    // ── Display settings ──
    if (ImGui::CollapsingHeader("Display")) {
        ImGui::Checkbox("Show Grid", &gShowGrid);
        ImGui::Checkbox("Wireframe", &gWireframe);

        ImGui::Separator();
        ImGui::Text("Camera");
        ImGui::SliderFloat("Distance", &gCamera.distance, 2.0f, 20.0f);
        ImGui::SliderFloat("Yaw",      &gCamera.yaw,    -180.0f, 180.0f);
        ImGui::SliderFloat("Pitch",    &gCamera.pitch,  -89.0f, 89.0f);
    }

    // ── Info ──
    if (ImGui::CollapsingHeader("Info")) {
        ImGui::Text("Robot: 6-DOF manipulator");
        ImGui::Text("Links: Base → Shoulder → Elbow");
        ImGui::Text("       → Wrist Pitch → Roll → Tool");
        ImGui::Separator();
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    }

    ImGui::End();
}

// ============================================================
// GLFW callbacks
// ============================================================

static GLFWwindow* gWindow = nullptr;

static void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

// ============================================================
// Main
// ============================================================

int main(int argc, char** argv) {
    // ── GLFW init ──
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        std::cerr << "glfwInit failed!" << std::endl;
        return 1;
    }

    // OpenGL 3.3 Core Profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x MSAA

    // Window
    int w = 1280, h = 800;
    GLFWwindow* window = glfwCreateWindow(w, h, "Robot Viewer — ImGui", nullptr, nullptr);
    if (!window) {
        std::cerr << "glfwCreateWindow failed!" << std::endl;
        glfwTerminate();
        return 1;
    }
    gWindow = window;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // VSync

    // ── OpenGL context is al actief (via GLFW) ──
    // GL functies worden geladen door ImGui's built-in loader

    // ── ImGui setup ──
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    // Donkere theme met wat aanpassingen
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding    = 4.0f;
    style.FrameRounding     = 3.0f;
    style.GrabRounding      = 2.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.11f, 0.14f, 1.00f);

    // Platform & Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    // Nu zijn alle GL functies geladen — we kunnen ze veilig gebruiken
    std::cout << "OpenGL: " << glGetString(GL_VERSION) << std::endl;

    // ── OpenGL init ──
    initOpenGL();

    // ── Main loop ──
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // ── ImGui NewFrame ──
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ── Framebuffer clear (VOORDAT we iets tekenen) ──
        int displayW, displayH;
        glfwGetFramebufferSize(window, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.12f, 0.13f, 0.16f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // ── Camera input ──
        // We checken of de muis over het 3D viewport hangt
        // (wordt bepaald in render3DViewport, dus doen we het na de eerste frame)

        // ── Update forward kinematics ──
        gRobot.updateAllFK();

        // ── Render 3D viewport ──
        ViewportInfo vp = render3DViewport();

        // ── Camera input (na viewport, zodat we isHovered weten) ──
        if (vp.isHovered) {
            ImVec2 m = ImGui::GetMousePos();
            bool leftDown  = ImGui::IsMouseDown(ImGuiMouseButton_Left);
            bool rightDown = ImGui::IsMouseDown(ImGuiMouseButton_Right);
            float wheel = ImGui::GetIO().MouseWheel;

            gCamera.handleInput(m.x, m.y, leftDown, rightDown, wheel);
        }

        // ── Render control panel ──
        renderControlPanel();

        // ── ImGui Render (geen clear meer — framebuffer is al gecleerd) ──
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // ── Cleanup ──
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
