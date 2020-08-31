/*
    g++ src/* -o App -I ./includes -lGL -lglfw -ldl -lassimp -I ./includes/bullet/ ./includes/bullet/BulletDynamics/libBulletDynamics.a ./includes/bullet/BulletCollision/libBulletCollision.a ./includes/bullet/LinearMath/libLinearMath.a
*/

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <utils/Shader.hpp>
#include <utils/Camera.hpp>
#include <utils/Model.hpp>
#include <utils/Physics.hpp>

#include <gtk/gtk.h>

#include <iostream>

const unsigned int SCR_WIDTH    = 960;
const unsigned int SCR_HEIGHT   = 540;
const char* APP_NAME            = "OpenGL Car Physics demo";

// General callback functions
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// GUI callback functions
void mass_callback(GtkWidget *widget, gpointer callback_data);
void stiffness_callback(GtkWidget *widget, gpointer callback_data);
void damping_callback(GtkWidget *widget, gpointer callback_data);
void friction_callback(GtkWidget *widget, gpointer callback_data);
void steering_callback(GtkWidget *widget, gpointer callback_data);
void acceleration_callback(GtkWidget *widget, gpointer callback_data);
void stability_callback(GtkWidget *widget, gpointer callback_data);
void preset0_callback(GtkWidget *widget, gpointer callback_data);
void preset1_callback(GtkWidget *widget, gpointer callback_data);
void preset2_callback(GtkWidget *widget, gpointer callback_data);
void preset3_callback(GtkWidget *widget, gpointer callback_data);

// Support functions
void processInput(GLFWwindow* window);
unsigned int loadCubeMap();

// Camera controls
Camera camera(glm::vec3(0.0f, 2.5f, 8.0f), GL_FALSE);
bool cameraFollow = TRUE;
glm::vec3 cameraFollowPos(0.0f);
float cameraFollowP = 0.0f, cameraFollowY = 0.0f;
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = TRUE;
bool rotating = FALSE;
float cameraRadius = 8.0f;
bool switched = FALSE;

// Car controls
short acceleration = 0;
float steering = 0.0f;
bool handbrake = FALSE;
float maxAcceleration = 800.0f;     // torque
float maxVelocity = 50.0f;          // max limit
bool getUp = FALSE, gotUp = FALSE;
bool jump = FALSE, jumped = FALSE;
float basePitch = 0.0f, baseYaw = 0.0f;

// Car properties
float car_mass = 1250.0f;
float tyre_mass_1 = 15.0f;          // front wheels
float tyre_mass_2 = 20.0f;          // rear wheels
float tyre_friction = 2.25f;
float tyre_stiffness = 120000.0f;   // suspensions
float tyre_damping = 0.0000200f;    // suspensions
float tyre_steering_angle = 0.5f;
float assist = 0.5f;
float lowLim = 0.0f;
float upLim = 0.1f;
const float cLinDamp = 0.02f;
const float cAngDamp = 0.4f;
const float tLinDamp = 0.01f;
const float tAngDamp = 0.2f;

// Car components
btRigidBody *car, *t1, *t2, *t3, *t4;
btGeneric6DofSpringConstraint *c1, *c2, *c3, *c4;

// UI widgets
GtkWidget *panel;
GtkWidget *vgrid;

GtkWidget *mass_text;
GtkWidget *mass;
GtkWidget *stiffness_text;
GtkWidget *stiffness;
GtkWidget *damping_text;
GtkWidget *damping;
GtkWidget *friction_text;
GtkWidget *friction;
GtkWidget *steer_text;
GtkWidget *steer;
GtkWidget *accelerate_text;
GtkWidget *accelerate;
GtkWidget *speedometer_text;
GtkWidget *speedometer;
GtkWidget *stability_text;
GtkWidget *stability;
GtkWidget *preset_text;
GtkWidget *preset0;
GtkWidget *preset1;
GtkWidget *preset2;
GtkWidget *preset3;

// Delta time
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main() {
    // Setup panel
    gtk_init(0, NULL);

    panel = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_position(GTK_WINDOW(panel), GTK_WIN_POS_CENTER);
    //gtk_window_set_default_size(GTK_WINDOW(panel), 230, 250);
    gtk_window_set_resizable(GTK_WINDOW(panel), FALSE);
    gtk_window_set_title(GTK_WINDOW(panel), "Vehicle settings");
    gtk_container_set_border_width(GTK_CONTAINER(panel), 5);

    vgrid = gtk_grid_new();
    gtk_grid_set_row_homogeneous(GTK_GRID(vgrid), FALSE);
    gtk_grid_set_column_homogeneous(GTK_GRID(vgrid), TRUE);
    gtk_grid_set_row_spacing(GTK_GRID(vgrid), 2);
    gtk_grid_set_column_spacing(GTK_GRID(vgrid), 2);
    gtk_container_add(GTK_CONTAINER(panel), vgrid);

    mass_text = gtk_label_new("Vehicle mass");
    mass = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 500, 3000, 1);
    //gtk_scale_set_draw_value(GTK_SCALE(stiffness), FALSE);
    gtk_scale_add_mark(GTK_SCALE(mass), 500, GTK_POS_TOP, "Light");
    gtk_scale_add_mark(GTK_SCALE(mass), 1250, GTK_POS_TOP, "Average");
    gtk_scale_add_mark(GTK_SCALE(mass), 3000, GTK_POS_TOP, "Heavy");
    gtk_range_set_value(GTK_RANGE(mass), 1250);

    stiffness_text = gtk_label_new("Suspension stiffness");
    stiffness = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 50000, 150000, 1);
    //gtk_scale_set_draw_value(GTK_SCALE(stiffness), FALSE);
    gtk_scale_add_mark(GTK_SCALE(stiffness), 50000, GTK_POS_TOP, "Weak");
    gtk_scale_add_mark(GTK_SCALE(stiffness), 100000, GTK_POS_TOP, "Medium");
    gtk_scale_add_mark(GTK_SCALE(stiffness), 150000, GTK_POS_TOP, "Strong");
    gtk_range_set_value(GTK_RANGE(stiffness), 95000);

    damping_text = gtk_label_new("Suspension damping");
    damping = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 50, 500, 1);
    //gtk_scale_set_draw_value(GTK_SCALE(damping), FALSE);
    gtk_scale_add_mark(GTK_SCALE(damping), 50, GTK_POS_TOP, "Soft");
    gtk_scale_add_mark(GTK_SCALE(damping), 200, GTK_POS_TOP, "Balanced");
    gtk_scale_add_mark(GTK_SCALE(damping), 500, GTK_POS_TOP, "Hard");
    gtk_range_set_value(GTK_RANGE(damping), 200);

    friction_text = gtk_label_new("Tyre friction coefficient");
    friction = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0, 10, 0.01);
    //gtk_scale_set_draw_value(GTK_SCALE(damping), FALSE);
    gtk_scale_add_mark(GTK_SCALE(friction), 0, GTK_POS_TOP, "None");
    gtk_scale_add_mark(GTK_SCALE(friction), 2.25, GTK_POS_TOP, "Normal");
    gtk_scale_add_mark(GTK_SCALE(friction), 5, GTK_POS_TOP, "High");
    gtk_scale_add_mark(GTK_SCALE(friction), 10, GTK_POS_TOP, "Extreme");
    gtk_range_set_value(GTK_RANGE(friction), 2.25);

    steer_text = gtk_label_new("Tyre steering angle");
    steer = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0, 1, 0.01);
    //gtk_scale_set_draw_value(GTK_SCALE(damping), FALSE);
    gtk_scale_add_mark(GTK_SCALE(steer), 0, GTK_POS_TOP, "Locked");
    gtk_scale_add_mark(GTK_SCALE(steer), 0.25, GTK_POS_TOP, "Mild");
    gtk_scale_add_mark(GTK_SCALE(steer), 0.5, GTK_POS_TOP, "Normal");
    gtk_scale_add_mark(GTK_SCALE(steer), 0.7, GTK_POS_TOP, "Loose");
    gtk_scale_add_mark(GTK_SCALE(steer), 1, GTK_POS_TOP, "Sharp");
    gtk_range_set_value(GTK_RANGE(steer), 0.5);

    accelerate_text = gtk_label_new("Acceleration power");
    accelerate = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0, 1000, 1);
    //gtk_scale_set_draw_value(GTK_SCALE(damping), FALSE);
    gtk_scale_add_mark(GTK_SCALE(accelerate), 0, GTK_POS_TOP, "None");
    gtk_scale_add_mark(GTK_SCALE(accelerate), 250, GTK_POS_TOP, "Low");
    gtk_scale_add_mark(GTK_SCALE(accelerate), 500, GTK_POS_TOP, "Medium");
    gtk_scale_add_mark(GTK_SCALE(accelerate), 700, GTK_POS_TOP, "High");
    gtk_scale_add_mark(GTK_SCALE(accelerate), 1000, GTK_POS_TOP, "Max");
    gtk_range_set_value(GTK_RANGE(accelerate), 350);

    stability_text = gtk_label_new("Stability assist");
    stability = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0, 1, 0.05);
    //gtk_scale_set_draw_value(GTK_SCALE(damping), FALSE);
    gtk_scale_add_mark(GTK_SCALE(stability), 0, GTK_POS_TOP, "Absent");
    gtk_scale_add_mark(GTK_SCALE(stability), 0.25, GTK_POS_TOP, "Weak");
    gtk_scale_add_mark(GTK_SCALE(stability), 0.5, GTK_POS_TOP, "Aware");
    gtk_scale_add_mark(GTK_SCALE(stability), 0.75, GTK_POS_TOP, "Strong");
    gtk_scale_add_mark(GTK_SCALE(stability), 1, GTK_POS_TOP, "Dramatic");
    gtk_range_set_value(GTK_RANGE(stability), 0.5);

    speedometer_text = gtk_label_new("Speedometer");
    speedometer = gtk_level_bar_new_for_interval(0, 100);
    gtk_level_bar_set_mode(GTK_LEVEL_BAR(speedometer), GTK_LEVEL_BAR_MODE_CONTINUOUS);

    preset_text = gtk_label_new("Presets");
    preset0 = gtk_button_new_with_label("Normal");
    preset1 = gtk_button_new_with_label("Muscle");
    preset2 = gtk_button_new_with_label("Pimp");
    preset3 = gtk_button_new_with_label("Sport");

    gtk_grid_attach(GTK_GRID(vgrid), mass_text, 0, 0, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), mass, 0, 1, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), stiffness_text, 0, 2, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), stiffness, 0, 3, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), damping_text, 0, 4, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), damping, 0, 5, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), friction_text, 0, 6, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), friction, 0, 7, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), steer_text, 0, 8, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), steer, 0, 9, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), accelerate_text, 0, 10, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), accelerate, 0, 11, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), stability_text, 0, 12, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), stability, 0, 13, 4, 1);
    //gtk_grid_attach(GTK_GRID(vgrid), speedometer_text, 0, 12, 4, 1);
    //gtk_grid_attach(GTK_GRID(vgrid), speedometer, 0, 13, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), preset_text, 0, 14, 4, 1);
    gtk_grid_attach(GTK_GRID(vgrid), preset0, 0, 15, 2, 1);
    gtk_grid_attach(GTK_GRID(vgrid), preset1, 2, 15, 2, 1);
    gtk_grid_attach(GTK_GRID(vgrid), preset2, 0, 16, 2, 1);
    gtk_grid_attach(GTK_GRID(vgrid), preset3, 2, 16, 2, 1);

    gtk_grid_remove_row(GTK_GRID(vgrid), 17);

    g_signal_connect(G_OBJECT(mass), "value-changed", G_CALLBACK(mass_callback), G_OBJECT(mass));
    g_signal_connect(G_OBJECT(stiffness), "value-changed", G_CALLBACK(stiffness_callback), G_OBJECT(stiffness));
    g_signal_connect(G_OBJECT(damping), "value-changed", G_CALLBACK(damping_callback), G_OBJECT(damping));
    g_signal_connect(G_OBJECT(friction), "value-changed", G_CALLBACK(friction_callback), G_OBJECT(friction));
    g_signal_connect(G_OBJECT(steer), "value-changed", G_CALLBACK(steering_callback), G_OBJECT(steer));
    g_signal_connect(G_OBJECT(accelerate), "value-changed", G_CALLBACK(acceleration_callback), G_OBJECT(accelerate));
    g_signal_connect(G_OBJECT(stability), "value-changed", G_CALLBACK(stability_callback), G_OBJECT(stability));
    g_signal_connect(G_OBJECT(preset0), "clicked", G_CALLBACK(preset0_callback), G_OBJECT(preset0));
    g_signal_connect(G_OBJECT(preset1), "clicked", G_CALLBACK(preset1_callback), G_OBJECT(preset1));
    g_signal_connect(G_OBJECT(preset2), "clicked", G_CALLBACK(preset2_callback), G_OBJECT(preset2));
    g_signal_connect(G_OBJECT(preset3), "clicked", G_CALLBACK(preset3_callback), G_OBJECT(preset3));

    g_signal_connect(G_OBJECT(panel), "destroy", G_CALLBACK(gtk_main_quit), G_OBJECT(panel));

    gtk_widget_show_all(panel);

    // Setup OpenGL environment
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, APP_NAME, NULL, NULL);
    if (window == NULL) {
        std::cout << "ERROR: failed to create GLFW window" << std::endl;
        glfwTerminate();
        return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        std::cout << "ERROR: failed to initialise GLAD" << std::endl;
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Our game
    glm::vec3 lightPos(0.0, 2.0, -1.0);

    // Car
    Shader mShader("shaders/car.vert", "shaders/car.frag");
    Model mModel((char*) "models/car/car.obj");
    Model t1Model((char*) "models/car/tyref.obj");
    Model t2Model((char*) "models/car/tyreb.obj");

    // Terrain
    Shader tShader("shaders/terrain.vert", "shaders/terrain.frag");
    Model tModel0((char*) "models/terrain/grass.obj");
    Model tModel1((char*) "models/terrain/asphalt.obj");

    // Skybox
    Shader sShader("shaders/skybox.vert", "shaders/skybox.frag");
    float skyboxVertices[] = {
        // positions
        -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

        -1.0f,  1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f,  1.0f
    };
    unsigned int skyboxVAO, skyboxVBO;
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    unsigned int cubemapTexture = loadCubeMap();

    // Physics world
    Physics simulation;

    maxAcceleration = 500.0f;
    maxVelocity = 50.0f;

    car_mass = 1250.0f;           // 500 <-> 2k
    tyre_mass_1 = 20.0f;          // 20.0
    tyre_mass_2 = 25.0f;          // 25.0
    tyre_friction = 2.35f;        // 2.35
    tyre_stiffness = 100000.0f;   // 80k +/- 30k <-> 120k +/- 30k
    tyre_damping = 0.0000225f;    // 0.0000100 <-> 0.0000300 +/- 200 (depending on mass), 0.0000250 is stable
    tyre_steering_angle = 0.5f;   // 0.5 <-> 1.0
    lowLim = 0.0f;
    upLim = 0.1f;

    // Terrain
    const unsigned int grid_width = 5;
    const unsigned int grid_height = 8;
    const unsigned int tiles = grid_width * grid_height;
    const unsigned int track[grid_height][grid_width] = {
        { 0, 0, 0, 0, 0 },
        { 0, 1, 1, 1, 0 },
        { 0, 1, 0, 1, 0 },
        { 0, 1, 0, 1, 0 },
        { 0, 1, 0, 1, 0 },
        { 0, 1, 0, 1, 0 },
        { 0, 1, 1, 1, 0 },
        { 0, 0, 0, 0, 0 }
    };
    btRigidBody *plane[tiles];
    glm::vec3 plane_pos[tiles];
    glm::vec3 plane_size[tiles];
    const float plane_edge = 20.0f;

    for (unsigned int i = 0; i < grid_width; i++) {
        for (unsigned int j = 0; j < grid_height; j++) {
            plane_pos[i*(grid_height)+j] = glm::vec3(2*plane_edge*i - plane_edge*(grid_width-1), -0.0f*(i*(grid_height)+j), 2*plane_edge*j - plane_edge*(grid_height-1));
            plane_size[i*(grid_height)+j] = glm::vec3(plane_edge, 0.0f, plane_edge);
            //cout << i << ", " << j << ": " << i+j << ". " << i*(grid_height)+j << endl;
            glm::vec3 plane_rot = glm::vec3(0.0f, 0.0f, 0.0f);
            if (track[j][i] == 0) {
                // Grass
                plane[i*(grid_height)+j] = simulation.createRigidBody(BOX, plane_pos[i*(grid_height)+j], plane_size[i*(grid_height)+j], plane_rot, 0.0f, 0.25f, 0.25f, COLL_TERRAIN, COLL_EVERYTHING);
            } else if (track[j][i] == 1) {
                // Asphalt
                plane[i*(grid_height)+j] = simulation.createRigidBody(BOX, plane_pos[i*(grid_height)+j] + glm::vec3(0.0f, 0.05f, 0.0f), plane_size[i*(grid_height)+j] + glm::vec3(0.0f, 0.05f, 0.0f), plane_rot, 0.0f, 0.5f, 0.5f, COLL_TERRAIN, COLL_EVERYTHING);
            }

        }
    }

    // Invisible walls
    const unsigned int walls = 4;
    float side;
    glm::vec3 wall_pos;
    glm::vec3 wall_size;
    btRigidBody *wall;

    side = plane_edge * grid_height;
    wall_size = glm::vec3(2*side, 5.0f, 0.0f);

    wall_pos = glm::vec3(0.0f, 2.5f, -side);
    wall = simulation.createRigidBody(BOX, wall_pos, wall_size, glm::vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f, COLL_TERRAIN, COLL_EVERYTHING);
    wall_pos = glm::vec3(0.0f, 2.5f, side);
    wall = simulation.createRigidBody(BOX, wall_pos, wall_size, glm::vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f, COLL_TERRAIN, COLL_EVERYTHING);

    side = plane_edge * grid_width;
    wall_size = glm::vec3(0.0f, 5.0f, 2*side);

    wall_pos = glm::vec3(-side, 2.5f, 0.0f);
    wall = simulation.createRigidBody(BOX, wall_pos, wall_size, glm::vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f, COLL_TERRAIN, COLL_EVERYTHING);
    wall_pos = glm::vec3(side, 2.5f, 0.0f);
    wall = simulation.createRigidBody(BOX, wall_pos, wall_size, glm::vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f, COLL_TERRAIN, COLL_EVERYTHING);

    // Muscle car
    glm::vec3 spawn = glm::vec3(-40.0f, 0.0f, 0.0f);  // start position in world

    glm::vec3 car_pos = glm::vec3(0.0f, 1.0f, 0.0f) + spawn;
    glm::vec3 car_size = glm::vec3(1.0f, 0.6f, 3.0f);
    glm::vec3 car_rot = glm::vec3(0.0f, 0.0f, 0.0f);
    car = simulation.createRigidBody(BOX, car_pos, car_size, car_rot, car_mass, 1.75f, 0.2f, COLL_CHASSIS, COLL_EVERYTHING^COLL_CAR);
    car->setSleepingThresholds(0.0, 0.0);   // never stop simulating
    car->setDamping(cLinDamp*assist, cAngDamp*assist);

    btTransform frameA, frameB;

    glm::vec3 t1_pos = glm::vec3(-1.0f, 0.5f, -2.1f) + spawn;
    glm::vec3 t1_size = glm::vec3(0.4f, 0.35f, 0.35f);
    glm::vec3 t1_rot = glm::vec3(0.0f, 0.0f, glm::radians(-90.0f));
    t1 = simulation.createRigidBody(CYLINDER, t1_pos, t1_size, t1_rot, tyre_mass_1, tyre_friction, 0.0f, COLL_TYRE, COLL_EVERYTHING^COLL_CAR);
    t1->setSleepingThresholds(0.0, 0.0);    // never stop simulating
    t1->setDamping(tLinDamp*assist, tAngDamp*assist);
    frameA = btTransform::getIdentity();
    frameB = btTransform::getIdentity();
    frameA.getBasis().setEulerZYX(0, 0, 0);
    frameB.getBasis().setEulerZYX(0, 0, glm::radians(90.0f));
    frameA.setOrigin(btVector3(-1.0, -0.5, -2.1));
    frameB.setOrigin(btVector3(0.0, 0.0, 0.0));
    c1 = new btGeneric6DofSpringConstraint(*car, *t1, frameA, frameB, TRUE);
    c1->setLinearLowerLimit(btVector3(0, -lowLim, 0));
    c1->setLinearUpperLimit(btVector3(0, -upLim, 0));
    c1->setAngularLowerLimit(btVector3(1, -0.5, 0));
    c1->setAngularUpperLimit(btVector3(-1, 0.5, 0));
    c1->enableSpring(1, TRUE);
    //c1->enableMotor(1, TRUE);
    c1->setStiffness(1, tyre_stiffness);
    c1->setDamping(1, tyre_damping);
    c1->setEquilibriumPoint();

    glm::vec3 t2_pos = glm::vec3(1.0f, 0.5f, -2.1f) + spawn;
    glm::vec3 t2_size = glm::vec3(0.4f, 0.35f, 0.35f);
    glm::vec3 t2_rot = glm::vec3(0.0f, 0.0f, glm::radians(90.0f));
    t2 = simulation.createRigidBody(CYLINDER, t2_pos, t2_size, t2_rot, tyre_mass_1, tyre_friction, 0.0f, COLL_TYRE, COLL_EVERYTHING^COLL_CAR);
    t2->setSleepingThresholds(0.0, 0.0);
    t2->setDamping(tLinDamp*assist, tAngDamp*assist);
    frameA = btTransform::getIdentity();
    frameB = btTransform::getIdentity();
    frameA.getBasis().setEulerZYX(0, 0, 0);
    frameB.getBasis().setEulerZYX(0, 0, glm::radians(-90.0f));
    frameA.setOrigin(btVector3(1.0, -0.5, -2.1));
    frameB.setOrigin(btVector3(0.0, 0.0, 0.0));
    c2 = new btGeneric6DofSpringConstraint(*car, *t2, frameA, frameB, TRUE);
    c2->setLinearLowerLimit(btVector3(0, -lowLim, 0));
    c2->setLinearUpperLimit(btVector3(0, -upLim, 0));
    c2->setAngularLowerLimit(btVector3(1, -0.5, 0));
    c2->setAngularUpperLimit(btVector3(-1, 0.5, 0));
    c2->enableSpring(1, TRUE);
    //c2->enableMotor(1, TRUE);
    c2->setStiffness(1, tyre_stiffness);
    c2->setDamping(1, tyre_damping);
    c2->setEquilibriumPoint();

    glm::vec3 t3_pos = glm::vec3(-1.0f, 0.5f, 1.6f) + spawn;
    glm::vec3 t3_size = glm::vec3(0.45f, 0.4f, 0.4f);
    glm::vec3 t3_rot = glm::vec3(0.0f, 0.0f, glm::radians(-90.0f));
    t3 = simulation.createRigidBody(CYLINDER, t3_pos, t3_size, t3_rot, tyre_mass_2, tyre_friction, 0.0f, COLL_TYRE, COLL_EVERYTHING^COLL_CAR);
    t3->setSleepingThresholds(0.0, 0.0);
    t3->setDamping(tLinDamp*assist, tAngDamp*assist);
    frameA = btTransform::getIdentity();
    frameB = btTransform::getIdentity();
    frameA.getBasis().setEulerZYX(0, 0, 0);
    frameB.getBasis().setEulerZYX(0, 0, glm::radians(90.0f));
    frameA.setOrigin(btVector3(-1.0, -0.5, 1.6));
    frameB.setOrigin(btVector3(0.0, 0.0, 0.0));
    c3 = new btGeneric6DofSpringConstraint(*car, *t3, frameA, frameB, TRUE);
    c3->setLinearLowerLimit(btVector3(0, -lowLim, 0));
    c3->setLinearUpperLimit(btVector3(0, -upLim, 0));
    c3->setAngularLowerLimit(btVector3(1, 0, 0));
    c3->setAngularUpperLimit(btVector3(-1, 0, 0));
    c3->enableSpring(1, TRUE);
    c3->setStiffness(1, tyre_stiffness);
    c3->setDamping(1, tyre_damping);
    c3->setEquilibriumPoint();

    glm::vec3 t4_pos = glm::vec3(1.0f, 0.5f, 1.6f) + spawn;
    glm::vec3 t4_size = glm::vec3(0.45f, 0.4f, 0.4f);
    glm::vec3 t4_rot = glm::vec3(0.0f, 0.0f, glm::radians(90.0f));
    t4 = simulation.createRigidBody(CYLINDER, t4_pos, t4_size, t4_rot, tyre_mass_2, tyre_friction, 0.0f, COLL_TYRE, COLL_EVERYTHING^COLL_CAR);
    t4->setSleepingThresholds(0.0, 0.0);
    t4->setDamping(tLinDamp*assist, tAngDamp*assist);
    frameA = btTransform::getIdentity();
    frameB = btTransform::getIdentity();
    frameA.getBasis().setEulerZYX(0, 0, 0);
    frameB.getBasis().setEulerZYX(0, 0, glm::radians(-90.0f));
    frameA.setOrigin(btVector3(1.0, -0.5, 1.6));
    frameB.setOrigin(btVector3(0.0, 0.0, 0.0));
    c4 = new btGeneric6DofSpringConstraint(*car, *t4, frameA, frameB, TRUE);
    c4->setLinearLowerLimit(btVector3(0, -lowLim, 0));
    c4->setLinearUpperLimit(btVector3(0, -upLim, 0));
    c4->setAngularLowerLimit(btVector3(1, 0, 0));
    c4->setAngularUpperLimit(btVector3(-1, 0, 0));
    c4->enableSpring(1, TRUE);
    c4->setStiffness(1, tyre_stiffness);
    c4->setDamping(1, tyre_damping);
    c4->setEquilibriumPoint();

    simulation.dynamicsWorld->addConstraint(c1);
    simulation.dynamicsWorld->addConstraint(c2);
    simulation.dynamicsWorld->addConstraint(c3);
    simulation.dynamicsWorld->addConstraint(c4);

    GLfloat maxSecPerFrame = 1.0f / 50.0f;


    // Game loop
    while (!glfwWindowShouldClose(window)) {
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //glClearColor(0.2, 0.4, 0.4, 1.0);

        while (gtk_events_pending())
        gtk_main_iteration();

        processInput(window);

        btMatrix3x3 rot = car->getWorldTransform().getBasis();
        short braking = 1;

        // Acceleration
        float linearVelocity = car->getLinearVelocity().length();
        gtk_level_bar_set_value(GTK_LEVEL_BAR(speedometer), linearVelocity);
        if (acceleration < 0 && linearVelocity > maxVelocity/10) {
            braking = 0;
        } else {
            if (linearVelocity < maxVelocity/(1 + 9*(acceleration < 0))) {
                float torque = -maxAcceleration * acceleration * (1-(abs(steering)*(linearVelocity>10))/2);
                t1->applyTorque(rot * btVector3(torque, 0, 0));
                t2->applyTorque(rot * btVector3(torque, 0, 0));
                if (!handbrake) {
                    t3->applyTorque(rot * btVector3(torque, 0, 0));
                    t4->applyTorque(rot * btVector3(torque, 0, 0));
                }
            }
        }

        // Braking / steering
        c1->setAngularLowerLimit(btVector3(braking, tyre_steering_angle*steering, 0));
        c1->setAngularUpperLimit(btVector3(-braking, tyre_steering_angle*steering, 0));
        c2->setAngularLowerLimit(btVector3(braking, tyre_steering_angle*steering, 0));
        c2->setAngularUpperLimit(btVector3(-braking, tyre_steering_angle*steering, 0));

        // Handbrake
        if (handbrake) {
            c3->setAngularLowerLimit(btVector3(0, 0, 0));
            c3->setAngularUpperLimit(btVector3(0, 0, 0));
            c4->setAngularLowerLimit(btVector3(0, 0, 0));
            c4->setAngularUpperLimit(btVector3(0, 0, 0));
        } else {
            c3->setAngularLowerLimit(btVector3(braking, 0, 0));
            c3->setAngularUpperLimit(btVector3(-braking, 0, 0));
            c4->setAngularLowerLimit(btVector3(braking, 0, 0));
            c4->setAngularUpperLimit(btVector3(-braking, 0, 0));
        }


        // Get up
        if (getUp) {
            car->applyTorqueImpulse(rot * btVector3(0, 0, 12000));
        }

        // Jump
        if (jump) {
            car->applyCentralImpulse(btVector3(0, 10000, 0));
        }

        // Step physics forward
        simulation.dynamicsWorld->stepSimulation((deltaTime < maxSecPerFrame ? deltaTime : maxSecPerFrame), 10);

        // Update camera position
        if (cameraFollow) {
            btTransform temp;
            btVector3 newPos;

            car->getMotionState()->getWorldTransform(temp);
            float aVelocity = -car->getAngularVelocity().y();
            newPos = temp.getBasis() * btVector3(glm::cos(glm::radians(-10*glm::sqrt(glm::abs(steering))*aVelocity+90 + baseYaw/4))*cameraRadius, 0, glm::sin(glm::radians(-10*glm::sqrt(glm::abs(steering))*aVelocity + 90 + baseYaw/4))*cameraRadius);

            cameraFollowPos.x = temp.getOrigin().getX() + newPos.x();
            cameraFollowPos.y = temp.getOrigin().getY() - glm::sin(glm::radians(camera.Pitch))*cameraRadius +1.5;
            cameraFollowPos.z = temp.getOrigin().getZ() + newPos.z();

            //camera.Yaw = glm::degrees(temp.getBasis().getColumn(2).length())
            camera.Position = cameraFollowPos;// - glm::vec3(glm::cos(glm::radians(Y))*8, glm::sin(glm::radians(P))*8-1.5, glm::sin(glm::radians(Y))*8);
            //camera.Pitch -= 3.5f;
            camera.LookAt(-newPos.x(), newPos.y(), -newPos.z());
        }

        // Transforms
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 10000.0f);
        glm::mat4 view = camera.GetViewMatrix();

        // Terrain
        tShader.Use();
        tShader.setMat4("projection", projection);
        tShader.setMat4("view", view);
        tShader.setVec3("viewPos", camera.Position);

        tShader.setVec3("light.direction", 1.0f, -0.5f, -0.5f);
        tShader.setVec3("light.ambient", 0.473f, 0.428f, 0.322f);

        glm::mat4 model = glm::mat4(1.0f);
        //model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene

        glm::mat4 planeModelMatrix = glm::mat4(1.0f);
        for (unsigned int i = 0; i < grid_width; i++) {
            for (unsigned int j = 0; j < grid_height; j++) {
                planeModelMatrix = glm::translate(planeModelMatrix, plane_pos[i*(grid_height)+j]);
                glUniformMatrix4fv(glGetUniformLocation(tShader.Program, "model"), 1, GL_FALSE, glm::value_ptr(planeModelMatrix));

        if (track[j][i] == 0) {
            // Grass
            tShader.setFloat("material.shininess", 4.0f);
            tShader.setVec3("light.diffuse", 1.195f, 1.105f, 0.893f);
            tShader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);
            tModel0.Draw(tShader);
        } else if (track[j][i] == 1) {
            // Asphalt
            tShader.setFloat("material.shininess", 16.0f);
            tShader.setVec3("light.diffuse", 0.945f, 0.855f, 0.643f);
            tShader.setVec3("light.specular", 2.75f, 2.75f, 2.75f);
            tModel1.Draw(tShader);
        }

                planeModelMatrix = glm::mat4(1.0f);
            }
        }

        // Car
        mShader.Use();
        mShader.setMat4("projection", projection);
        mShader.setMat4("view", view);

        model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(0.0f, 1.0f, 0.0f)); // translate it down so it's at the center of the scene

        glm::mat4 objModelMatrix;
        glm::mat3 objNormalMatrix;

        GLfloat matrix[16];
        btTransform transform;

        glm::vec3 obj_size(1.0f);
        Model* objectModel;

        int num_cobjs = simulation.dynamicsWorld->getNumCollisionObjects();

        for (unsigned int i=tiles+walls; i<num_cobjs;i++)
        {
            switch (i) {
                case tiles+walls: objectModel = &mModel; break;
                case tiles+walls+1: case tiles+walls+2: objectModel = &t1Model; break;
                case tiles+walls+3: case tiles+walls+4: objectModel = &t2Model; break;
                default: return(EXIT_FAILURE);
            }
            // we take the Collision Object from the list
            btCollisionObject* obj = simulation.dynamicsWorld->getCollisionObjectArray()[i];

            // we upcast it in order to use the methods of the main class RigidBody
            btRigidBody* body = btRigidBody::upcast(obj);

            // we take the transformation matrix of the rigid boby, as calculated by the physics engine
            body->getMotionState()->getWorldTransform(transform);

            // we convert the Bullet matrix (transform) to an array of floats
            transform.getOpenGLMatrix(matrix);

            // we create the GLM transformation matrix
            objModelMatrix = glm::make_mat4(matrix) * glm::scale(objModelMatrix, obj_size);
            objNormalMatrix = glm::transpose(glm::inverse(glm::mat3(objModelMatrix)));

            // we create the normal matrix
            glUniformMatrix4fv(glGetUniformLocation(mShader.Program, "model"), 1, GL_FALSE, glm::value_ptr(objModelMatrix));
            glUniformMatrix3fv(glGetUniformLocation(mShader.Program, "normal"), 1, GL_FALSE, glm::value_ptr(objNormalMatrix));

            mShader.setVec3("lightColor", glm::vec3(1.0));
            mShader.setVec3("lightPos", lightPos);
            mShader.setVec3("viewPos", camera.Position);

            mShader.setFloat("material.shininess", 128.0f);

            mShader.setVec3("light.direction", 1.0f, -0.5f, -0.5f);
            mShader.setVec3("light.ambient", 0.5f, 0.5f, 0.5f);
            mShader.setVec3("light.diffuse", 0.945f, 0.855f, 0.643f);
            mShader.setVec3("light.specular", 4.0f, 4.0f, 4.0f);

            glActiveTexture(GL_TEXTURE3);
            mShader.setInt("skybox", 3);
            glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);

            // we render the model
            // N.B.) if the number of models is relatively low, this approach (we render the same mesh several time from the same buffers) can work. If we must render hundreds or more of copies of the same mesh, there are more advanced techniques to manage Instanced Rendering (see https://learnopengl.com/#!Advanced-OpenGL/Instancing for examples).
            objectModel->Draw(mShader);
            // we "reset" the matrix
            objModelMatrix = glm::mat4(1.0f);
            objNormalMatrix = glm::mat4(1.0f);
        }

        //mShader.setMat4("model", model);

        //mModel.Draw(mShader);

        view = glm::mat4(glm::mat3(camera.GetViewMatrix()));

        // Skybox
        glDepthFunc(GL_LEQUAL);
        sShader.Use();
        sShader.setMat4("projection", projection);
        sShader.setMat4("view", view);
        glBindVertexArray(skyboxVAO);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glDepthFunc(GL_LESS);

        glfwPollEvents();
        glfwSwapBuffers(window);
    }
    glfwTerminate();
    return EXIT_SUCCESS;
}

void processInput(GLFWwindow* window) {
    // Exit application
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }

    // Switch between free-movement and following camera
    if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS && !switched) {
        cameraFollow = !cameraFollow;
        switched = TRUE;
        if (cameraFollow) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        } else {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }
    }
    if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_RELEASE) {
        switched = FALSE;
    }

    // Control free-movement camera
    if (!cameraFollow) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    } else {
        //if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS))
    }

    float steering_limit = 1.0f;
    float steering_speed = 0.05f;

    // Car controls - steering
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        if (steering > -steering_limit)
            steering -= steering_speed;
    } else if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        if (steering < steering_limit)
            steering += steering_speed;
    } else {
        steering -= steering_speed * ((steering > 0) - (steering < 0));
        if (steering < steering_speed && steering > -steering_speed)
            steering = 0.0f;
    }

    // Car controls - acceleration
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        acceleration = 1;
    } else if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        acceleration = -1;
    } else {
        acceleration = 0;
        handbrake = TRUE;
    }

    // Car controls - handbrake
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        handbrake = TRUE;
    } else {
        handbrake = FALSE;
    }

    // Car controls - get up
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS && !gotUp) {
        getUp = TRUE;
        gotUp = TRUE;
    } else {
        getUp = FALSE;
    }
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_RELEASE) {
        gotUp = FALSE;
    }

    // Car controls - jump upwards
    if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && !jumped) {
        jump = TRUE;
        jumped = TRUE;
    } else {
        jump = FALSE;
    }
    if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE) {
        jumped = FALSE;
    }
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = FALSE;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    if (!cameraFollow) {
        camera.ProcessMouseMovement(xoffset, yoffset);
    } else if (rotating) {
        baseYaw += xoffset;
        basePitch += yoffset;
        if (basePitch > 89.0f)
            basePitch = 89.0f;
        if (basePitch < -89.0f)
            basePitch = -89.0f;
        camera.ProcessMouseMovement(0, yoffset);
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mod) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
        rotating = TRUE;
    else
        rotating = FALSE;
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cameraRadius -= yoffset/2;
    if (basePitch > 20.0f)
        basePitch = 20.0f;
    if (basePitch < 0.0f)
        basePitch = 0.0f;
}

unsigned int loadCubeMap() {
    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, channels;
    unsigned char *data;
    std::vector<std::string> txt_faces;
    txt_faces.push_back("textures/clouds1/clouds1_east.bmp");
    txt_faces.push_back("textures/clouds1/clouds1_west.bmp");
    txt_faces.push_back("textures/clouds1/clouds1_up.bmp");
    txt_faces.push_back("textures/clouds1/clouds1_down.bmp");
    txt_faces.push_back("textures/clouds1/clouds1_north.bmp");
    txt_faces.push_back("textures/clouds1/clouds1_south.bmp");
    for (unsigned int i = 0; i < 6; i++) {
        data = stbi_load(txt_faces[i].c_str(), &width, &height, &channels, 0);
        glTexImage2D(
            GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB,
            width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
        );
        stbi_image_free(data);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    return textureID;
}

// GUI callback functions
void mass_callback(GtkWidget *widget, gpointer callback_data) {
    car_mass = gtk_range_get_value(GTK_RANGE(widget));
    btVector3 inertia;
    car->getCollisionShape()->calculateLocalInertia(car_mass, inertia);
    car->setMassProps(car_mass, inertia);
    cout << "Mass: " << car_mass << endl;
}

void stiffness_callback(GtkWidget *widget, gpointer callback_data) {
    tyre_stiffness = gtk_range_get_value(GTK_RANGE(widget));
    c1->setStiffness(1, tyre_stiffness);
    c2->setStiffness(1, tyre_stiffness);
    c3->setStiffness(1, tyre_stiffness);
    c4->setStiffness(1, tyre_stiffness);
    cout << "Stiffness: " << tyre_stiffness << endl;
}

void damping_callback(GtkWidget *widget, gpointer callback_data) {
    tyre_damping = gtk_range_get_value(GTK_RANGE(widget))/10000000;
    c1->setDamping(1, tyre_damping);
    c2->setDamping(1, tyre_damping);
    c3->setDamping(1, tyre_damping);
    c4->setDamping(1, tyre_damping);
    cout << "Damping: " << tyre_damping << endl;
}

void friction_callback(GtkWidget *widget, gpointer callback_data) {
    tyre_friction = gtk_range_get_value(GTK_RANGE(widget));
    t1->setFriction(tyre_friction);
    t2->setFriction(tyre_friction);
    t3->setFriction(tyre_friction);
    t4->setFriction(tyre_friction);
    cout << "Friction: " << tyre_friction << endl;
}

void steering_callback(GtkWidget *widget, gpointer callback_data) {
    tyre_steering_angle = gtk_range_get_value(GTK_RANGE(widget));
    cout << "Steering angle: " << tyre_steering_angle << endl;
}

void acceleration_callback(GtkWidget *widget, gpointer callback_data) {
    maxAcceleration = gtk_range_get_value(GTK_RANGE(widget));
    cout << "Acceleration: " << maxAcceleration << endl;
}

void stability_callback(GtkWidget *widget, gpointer callback_data) {
    assist = gtk_range_get_value(GTK_RANGE(widget));
    car->setDamping(tLinDamp*assist, tAngDamp*assist);
    t1->setDamping(tLinDamp*assist, tAngDamp*assist);
    t2->setDamping(tLinDamp*assist, tAngDamp*assist);
    t3->setDamping(tLinDamp*assist, tAngDamp*assist);
    t4->setDamping(tLinDamp*assist, tAngDamp*assist);
    cout << "Stability: " << assist << endl;
}

void preset0_callback(GtkWidget *widget, gpointer callback_data) {
    gtk_range_set_value(GTK_RANGE(mass), 1250);
    gtk_range_set_value(GTK_RANGE(stiffness), 95000);
    gtk_range_set_value(GTK_RANGE(damping), 200);
    gtk_range_set_value(GTK_RANGE(friction), 2.25);
    gtk_range_set_value(GTK_RANGE(steer), 0.5);
    gtk_range_set_value(GTK_RANGE(accelerate), 350);
    cout << "Preset: Normal" << endl;
}

void preset1_callback(GtkWidget *widget, gpointer callback_data) {
    gtk_range_set_value(GTK_RANGE(mass), 1440);
    gtk_range_set_value(GTK_RANGE(stiffness), 70000);
    gtk_range_set_value(GTK_RANGE(damping), 130);
    gtk_range_set_value(GTK_RANGE(friction), 1.95);
    gtk_range_set_value(GTK_RANGE(steer), 0.7);
    gtk_range_set_value(GTK_RANGE(accelerate), 480);
    cout << "Preset: Muscle Car" << endl;
}

void preset2_callback(GtkWidget *widget, gpointer callback_data) {
    gtk_range_set_value(GTK_RANGE(mass), 1560);
    gtk_range_set_value(GTK_RANGE(stiffness), 80000);
    gtk_range_set_value(GTK_RANGE(damping), 420);
    gtk_range_set_value(GTK_RANGE(friction), 1.75);
    gtk_range_set_value(GTK_RANGE(steer), 0.69);
    gtk_range_set_value(GTK_RANGE(accelerate), 420);
    cout << "Preset: Pimp My Ride" << endl;
}

void preset3_callback(GtkWidget *widget, gpointer callback_data) {
    gtk_range_set_value(GTK_RANGE(mass), 1780);
    gtk_range_set_value(GTK_RANGE(stiffness), 13000);
    gtk_range_set_value(GTK_RANGE(damping), 190);
    gtk_range_set_value(GTK_RANGE(friction), 2.45);
    gtk_range_set_value(GTK_RANGE(steer), 0.73);
    gtk_range_set_value(GTK_RANGE(accelerate), 680);
    cout << "Preset: Sport" << endl;
}
