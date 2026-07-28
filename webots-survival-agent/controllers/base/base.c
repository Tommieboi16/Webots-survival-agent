#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/receiver.h>

#define TIME_STEP 32
#define DEBUG_MODE 1
#define DEFAULT_SPEED 6

// State machine states
typedef enum {
    STATE_EXPLORE,
    STATE_SEEK_WATER,
    STATE_SEEK_FOOD,
    STATE_AVOID_PREDATOR,
    STATE_DRINK,
    STATE_EAT
} RobotState;

// Enhanced resource location structure
typedef struct {
    double x;      // x position in the image
    double y;      // y position in the image
    double size;   // size of the detected area
    int found;     // whether resource is currently visible
    double last_seen_time; // when resource was last detected
} ResourceLocation;

// Device tags
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag camera, camera_1;  // camera is overhead, camera_1 is forward-facing
static WbDeviceTag receiver;

// Resource variables
static double hunger = 100;
static double thirst = 100;
static double health = 100;
static const double HUNGER_DECAY = 1.0;
static const double THIRST_DECAY = 1.0;
static const double HEALTH_DECAY = 10.0;

// Global resource locations
static ResourceLocation water_location = {0, 0, 0, 0, 0};
static ResourceLocation food_location = {0, 0, 0, 0, 0};

// State machine variables
static RobotState current_state = STATE_EXPLORE;
static int state_timer = 0;
static int search_counter = 0;

// Random movement variables
static int random_move_timer = 0;
static int random_direction = 1;  // 1 for right, -1 for left
static const int RANDOM_MOVE_INTERVAL = 100;  // Change direction every 100 steps

// Survival time tracking
static double survival_time = 0.0;

static void initialize() {
    wb_robot_init();
    
    // Initialize random seed
    srand(time(NULL));
    
    // Initialize motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    
    // Initialize cameras
    camera = wb_robot_get_device("camera");      // Overhead camera
    camera_1 = wb_robot_get_device("camera(1)"); // Forward-facing camera
    wb_camera_enable(camera, TIME_STEP);
    wb_camera_enable(camera_1, TIME_STEP);
    
    // Initialize receiver
    receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, TIME_STEP);
    
    if (DEBUG_MODE) {
        printf("=== Robot Starting ===\n");
        printf("Initial Status: H:%.2f T:%.2f F:%.2f\n\n", health, thirst, hunger);
    }
}

static void move(double l, double r) {
    wb_motor_set_velocity(left_motor, l);
    wb_motor_set_velocity(right_motor, r);
}

// Check for radio signals from predator
static int check_radio_signals() {
    while (wb_receiver_get_queue_length(receiver) > 0) {
        wb_receiver_next_packet(receiver);
        if (DEBUG_MODE) {
            printf("Received radio signal from predator!\n");
        }
        return 1;
    }
    return 0;
}

// Enhanced predator detection with direction using forward camera
static int detect_predator(double *red_percentage, int *predator_side) {
    if (check_radio_signals()) {
        *red_percentage = 1.0;
        *predator_side = 0;
        return 1;
    }
    
    const unsigned char *image = wb_camera_get_image(camera_1);
    int width = wb_camera_get_width(camera_1);
    int height = wb_camera_get_height(camera_1);
    
    int red_pixels_left = 0;
    int red_pixels_right = 0;
    int total_pixels = width * height;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = wb_camera_image_get_red(image, width, x, y);
            int g = wb_camera_image_get_green(image, width, x, y);
            int b = wb_camera_image_get_blue(image, width, x, y);
            if (r > 100 && r > (g * 1.2) && r > (b * 1.2)) {
                if (x < width/2) red_pixels_left++;
                else red_pixels_right++;
            }
        }
    }
    
    *red_percentage = (double)(red_pixels_left + red_pixels_right) / total_pixels;
    *predator_side = (red_pixels_left > red_pixels_right) ? -1 : 1;
    
    if (DEBUG_MODE && *red_percentage > 0.001) {
        printf("Predator detected! Red percentage: %.3f%%, Side: %d\n", 
               *red_percentage * 100, *predator_side);
    }
    
    return (*red_percentage > 0.001);
}

// Function to scan environment with overhead camera
static void scan_environment() {
    const unsigned char *image = wb_camera_get_image(camera);
    int width = wb_camera_get_width(camera);
    int height = wb_camera_get_height(camera);
    
    // Reset locations
    water_location.found = 0;
    food_location.found = 0;
    
    // Variables for centroid calculation
    double water_total_x = 0, water_total_y = 0, water_pixels = 0;
    double food_total_x = 0, food_total_y = 0, food_pixels = 0;
    
    // Scan entire image from overhead camera
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = wb_camera_image_get_red(image, width, x, y);
            int g = wb_camera_image_get_green(image, width, x, y);
            int b = wb_camera_image_get_blue(image, width, x, y);
            
            // Detect water (cyan)
            if (b > 180 && g > 100 && r < 100 && b > g) {
                water_total_x += x;
                water_total_y += y;
                water_pixels++;
            }
            
            // Detect food (dark green)
            if (g > 140 && g > r * 1.2 && g > b * 1.2) {
                food_total_x += x;
                food_total_y += y;
                food_pixels++;
            }
        }
    }
    
    // Update water location if found
    if (water_pixels > 10) {
        water_location.x = water_total_x / water_pixels;
        water_location.y = water_total_y / water_pixels;
        water_location.size = water_pixels / (width * height);
        water_location.found = 1;
        water_location.last_seen_time = wb_robot_get_time();
        
        if (DEBUG_MODE) {
            printf("Water detected at: (%.1f, %.1f), size: %.3f\n", 
                   water_location.x, water_location.y, water_location.size);
        }
    }
    
    // Update food location if found
    if (food_pixels > 10) {
        food_location.x = food_total_x / food_pixels;
        food_location.y = food_total_y / food_pixels;
        food_location.size = food_pixels / (width * height);
        food_location.found = 1;
        food_location.last_seen_time = wb_robot_get_time();
        
        if (DEBUG_MODE) {
            printf("Food detected at: (%.1f, %.1f), size: %.3f\n", 
                   food_location.x, food_location.y, food_location.size);
        }
    }
}
// Function to check if resource is centered in forward camera
static int is_resource_centered(int is_water) {
    const unsigned char *image = wb_camera_get_image(camera_1);
    int width = wb_camera_get_width(camera_1);
    int height = wb_camera_get_height(camera_1);
    int center_x = width / 2;
    int resource_x = 0;
    int resource_pixels = 0;
    
    // Scan bottom third of image for resources
    for (int y = height * 2/3; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = wb_camera_image_get_red(image, width, x, y);
            int g = wb_camera_image_get_green(image, width, x, y);
            int b = wb_camera_image_get_blue(image, width, x, y);
            
            int is_target = 0;
            if (is_water && b > 180 && g > 100 && r < 100 && b > g) {
                is_target = 1;
            } else if (!is_water && g > 140 && g > r * 1.2 && g > b * 1.2) {
                is_target = 1;
            }
            
            if (is_target) {
                resource_x += x;
                resource_pixels++;
            }
        }
    }
    
    if (resource_pixels > 10) {
        double avg_x = resource_x / resource_pixels;
        double offset = fabs(avg_x - center_x);
        return offset < (width * 0.1); // Resource is centered if within 10% of center
    }
    
    return 0;
}

// Enhanced priority system with weighted urgency
static RobotState determine_state() {
    // Dynamic weighting based on health
    double health_factor = (health < 50) ? 1.5 : 1.0;
    double thirst_urgency = (100 - thirst) * 1.2 * health_factor;
    double hunger_urgency = (100 - hunger) * health_factor;
    
    double red_percentage;
    int predator_side;
    
    // Check for predator first
    if (check_radio_signals() || detect_predator(&red_percentage, &predator_side)) {
        if (DEBUG_MODE) printf("DANGER! Predator detected - switching to avoidance!\n");
        return STATE_AVOID_PREDATOR;
    }
    
    // Regular state determination
    if (health < 70) return STATE_AVOID_PREDATOR;
    if (thirst < 30) return STATE_SEEK_WATER;
    if (hunger < 30) return STATE_SEEK_FOOD;
    
    if (thirst_urgency > hunger_urgency && thirst < 60) return STATE_SEEK_WATER;
    if (hunger_urgency > thirst_urgency && hunger < 60) return STATE_SEEK_FOOD;
    
    return STATE_EXPLORE;
}

// Modified update_resources function with exact color matching
static void update_resources() {
    double red_percentage;
    int predator_side;
    
    // Update scan of environment
    scan_environment();
    
    // Basic decay
    hunger -= HUNGER_DECAY / TIME_STEP;
    thirst -= THIRST_DECAY / TIME_STEP;
    
    // Check forward camera for resources
    const unsigned char *image = wb_camera_get_image(camera_1);
    int width = wb_camera_get_width(camera_1);
    int height = wb_camera_get_height(camera_1);
    int water_pixels = 0;
    int food_pixels = 0;
    
    // Check camera for resources
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = wb_camera_image_get_red(image, width, x, y);
            int g = wb_camera_image_get_green(image, width, x, y);
            int b = wb_camera_image_get_blue(image, width, x, y);
            
            // Water detection (cyan/blue)
            if (b > 200 && g > 180 && r < 100) {
                water_pixels++;
            }
            
            // Food detection (dark green)
            if (g > 140 && g > (r * 1.5) && g > (b * 1.5)) {
                food_pixels++;
            }
        }
    }
    
    // Resource recovery
    if (water_pixels > 100) {
        thirst = 100.0;
        if (DEBUG_MODE) printf("Water detected! Thirst reset to: 100.0\n");
    }

    if (food_pixels > 100) {
        hunger = 100.0;
        if (DEBUG_MODE) printf("Food detected! Hunger reset to: 100.0\n");
    }
    
    // Predator damage
    if (detect_predator(&red_percentage, &predator_side)) {
        double damage = HEALTH_DECAY * (red_percentage * 2) / TIME_STEP;
        health -= damage;
        if (DEBUG_MODE && damage > 0.1) {
            printf("!!! HEALTH WARNING !!! Predator damage: -%.2f (Health: %.2f)\n", damage, health);
        }
    }
    
    // Clamp values
    if (hunger > 100) hunger = 100;
    if (thirst > 100) thirst = 100;
    if (health > 100) health = 100;
    
    if (DEBUG_MODE) {
        printf("Status: H:%.2f T:%.2f F:%.2f State: %d\n", health, thirst, hunger, current_state);
    }
}
// Enhanced behaviour execution with dual camera system and random exploration
static void execute_behaviour() {
    // First scan environment with overhead camera
    scan_environment();
    
    // Update state
    RobotState new_state = determine_state();
    if (new_state != current_state) {
        state_timer = 0;
        current_state = new_state;
        if (DEBUG_MODE) printf("State changed to: %d\n", current_state);
    }
    
    // Execute behaviour based on state
    switch (current_state) {
        case STATE_AVOID_PREDATOR:
            {
                double red_percentage;
                int predator_side;
                detect_predator(&red_percentage, &predator_side);
                
                if (check_radio_signals() || red_percentage > 0.05) {
                    move(-DEFAULT_SPEED, -DEFAULT_SPEED);  // Full retreat
                    if (DEBUG_MODE) printf("DANGER - RETREATING!\n");
                } else {
                    // Turn away from last known predator position
                    move(predator_side * -DEFAULT_SPEED / 2, predator_side * DEFAULT_SPEED / 2);
                }
                state_timer = 0;
            }
            break;
            
        case STATE_SEEK_WATER:
            if (water_location.found) {
                // Calculate turn direction based on water location
                double center_x = wb_camera_get_width(camera) / 2;
                if (is_resource_centered(1)) {
                    current_state = STATE_DRINK;
                } else if (water_location.x < center_x - 10) {
                    move(DEFAULT_SPEED - 2, DEFAULT_SPEED + 2); // Turn left
                } else if (water_location.x > center_x + 10) {
                    move(DEFAULT_SPEED + 2, DEFAULT_SPEED - 2); // Turn right
                } else {
                    move(DEFAULT_SPEED, DEFAULT_SPEED); // Move forward
                }
            } else {
                // Search pattern when water not visible
                if (search_counter++ % 20 < 10) {
                    move(DEFAULT_SPEED - 1, DEFAULT_SPEED + 1);
                } else {
                    move(DEFAULT_SPEED + 1, DEFAULT_SPEED - 1);
                }
            }
            break;
            
        case STATE_SEEK_FOOD:
            if (food_location.found) {
                // Calculate turn direction based on food location
                double center_x = wb_camera_get_width(camera) / 2;
                if (is_resource_centered(0)) {
                    current_state = STATE_EAT;
                } else if (food_location.x < center_x - 10) {
                    move(DEFAULT_SPEED - 2, DEFAULT_SPEED + 2); // Turn left
                } else if (food_location.x > center_x + 10) {
                    move(DEFAULT_SPEED + 2, DEFAULT_SPEED - 2); // Turn right
                } else {
                    move(DEFAULT_SPEED, DEFAULT_SPEED); // Move forward
                }
            } else {
                // Search pattern when food not visible
                if (search_counter++ % 20 < 10) {
                    move(DEFAULT_SPEED + 1, DEFAULT_SPEED - 1);
                } else {
                    move(DEFAULT_SPEED - 1, DEFAULT_SPEED + 1);
                }
            }
            break;
            
        case STATE_DRINK:
            if (thirst >= 100) {
                current_state = STATE_EXPLORE;
                if (DEBUG_MODE) printf("Finished drinking, thirst satisfied.\n");
            } else if (!is_resource_centered(1)) {
                current_state = STATE_SEEK_WATER;
                if (DEBUG_MODE) printf("Lost water source, returning to seek state.\n");
            } else {
                move(0, 0);  // Stop to drink
                if (DEBUG_MODE) printf("At water source. Drinking...\n");
            }
            break;
            
        case STATE_EAT:
            if (hunger >= 100) {
                current_state = STATE_EXPLORE;
                if (DEBUG_MODE) printf("Finished eating, hunger satisfied.\n");
            } else if (!is_resource_centered(0)) {
                current_state = STATE_SEEK_FOOD;
                if (DEBUG_MODE) printf("Lost food source, returning to seek state.\n");
            } else {
                move(0, 0);  // Stop to eat
                if (DEBUG_MODE) printf("At food source. Eating...\n");
            }
            break;
            
        case STATE_EXPLORE:
        default:
            {
                // Update random movement timer
                random_move_timer++;
                
                // Change random direction periodically or based on random chance
                if (random_move_timer >= RANDOM_MOVE_INTERVAL || (rand() % 100 < 5)) {
                    random_move_timer = 0;
                    random_direction = (rand() % 2) * 2 - 1;  // Random -1 or 1
                    
                    if (DEBUG_MODE) {
                        printf("Changing random direction: %d\n", random_direction);
                    }
                }
                
                // Random movement patterns
                int movement_pattern = rand() % 4;
                
                switch (movement_pattern) {
                    case 0:  // Move forward
                        move(DEFAULT_SPEED, DEFAULT_SPEED);
                        break;
                        
                    case 1:  // Turn slightly
                        move(DEFAULT_SPEED + random_direction, DEFAULT_SPEED - random_direction);
                        break;
                        
                    case 2:  // Turn more sharply
                        move(DEFAULT_SPEED + (2 * random_direction), DEFAULT_SPEED - (2 * random_direction));
                        break;
                        
                    case 3:  // Random spin with forward motion
                        move(DEFAULT_SPEED + (3 * random_direction), DEFAULT_SPEED - (3 * random_direction));
                        break;
                }
            }
            break;
    }
    
    state_timer++;
}

int main() {
    initialize();
    
    while (wb_robot_step(TIME_STEP) != -1 && hunger > 0 && thirst > 0 && health > 0) {
        survival_time += (double)TIME_STEP / 1000.0;
        update_resources();
        execute_behaviour();
    }
    
    move(0, 0);
    if (DEBUG_MODE) {
        printf("=== Simulation Ended ===\n");
        printf("Total Survival Time: %.2f seconds\n", survival_time);
        printf("Final Status: H:%.2f T:%.2f F:%.2f\n", health, thirst, hunger);
    }
    
    wb_robot_cleanup();
    return 0;
}