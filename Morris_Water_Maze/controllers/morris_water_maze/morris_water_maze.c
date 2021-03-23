
/*
 * Jeff Krichmar
 *
 * University of California, Irvine
 *
 * Description:   Demonstration of Morris Water Maze (MWM). Builds on the demo code for obstacle avoidance 
 * on Firebird 6 robot by Anant Malewar; Nex Robotics.
 * 
 * This is a Webots implementation of Foster, Dayan, Morris, "A Model of Hippocampally Dependent Navigation, 
 * Using the Temporal Difference Learning Rule", Hippocampus, 10:1-16, 2000.
 * 
 * An actor-critic model using the temporal difference rule learns the location of a platform. It uses the 
 * robot's compass to calculate its heading.  Proximity sensors are used to avoid the walls of the arena.
 * The actor-critic model learns a mapping between a place cell and a heading pointing towards the platform.
 * 
 */

#include <webots/distance_sensor.h>
#include <webots/compass.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

// Definitions for Webots environment
#define TIME_STEP 64
#define TURN_RATE 100
#define DIRECTIONS 8
#define HD_RES 1.0
#define PLATFORM_RADIUS 0.33
#define PLATFORM_X 0.33
#define PLATFORM_Z 0.67
#define START_DIST 1.5

// Definitions for MWM Actor-Critic model
#define RADIUS 1.4
#define SIGMA 0.32
#define REWARD 2.0
#define LEARNING_RATE 0.01
#define BETA 2.0              // For the Softmax function
#define PI 3.141592653589793238
#define TRIALS 32
#define TIMEOUT 1000000
#define SEED 1

// The MWM model evenly distributes 2D Gaussian place cells (PC). 
//   across the arena.
const int pcDim = (int)(2.0*RADIUS/(SIGMA/2.0)+0.5);
const int numPC = pcDim*pcDim;
double pcX[numPC];
double pcY[numPC];
double pc[numPC];
double z[DIRECTIONS][numPC]; // actor weights
double w[numPC]; // critic weights
double a[DIRECTIONS];  // actions

// Each trial, the robot starts in one of four different
// positions at the edge of the arena
double startingTranslation [4][3] = {
  {-1.6, 0.13, 0},
  {-1.0, 0.13, -1.3},
  {1.0, 0.13, -1.4},
  {1.6, 0.13, -0.60}
};

double startingRotation [4][4] = {
  {0, 1, 0, -1.8},
  {0, 1, 0, -2.0},
  {0, 1, 0, -3.4},
  {0, 1, 0, 2.0}
};

// Creates a population of place cells evenly distributed across
// the arena.  The place cell is a 2D Gaussian basis function.
void init_place_cells () {
    int inx = 0;
    double x = -RADIUS;
    double y;
    FILE *fpPlace;
    
    fpPlace = fopen("mwm_place.txt", "w");
    for (int i = 0; i < pcDim; i++) {
        y = -RADIUS;
        for (int j = 0; j < pcDim; j++) {
            pcX[inx] = x;
            pcY[inx] = y;
            fprintf(fpPlace, "%f\t%f\n", x, y);
            inx++;
            y += SIGMA/2.0;
        }
        x += SIGMA/2.0;
    }
    fclose (fpPlace);
    
    // initialize the critic (w) and actor (z) weights.
    for (int i = 0; i < numPC; i++) {
        w[i] = 0.0;
        for (int j = 0; j < DIRECTIONS; j++) {
            z[j][i] = 0.0;
        }
    }
}

// place cell response is a 2D Gaussian based on the distance from the agent
// to the place cell center
double place_cell (double ratX, double ratY, double pcX, double pcY) {
    
    double d = sqrt(pow(ratX-pcX,2.0) + pow(ratY-pcY,2.0));
    return exp(-pow(d,2.0)/(2*pow(SIGMA,2.0)));
}

// choose an action based on the actor weights
int action_select (bool printP) {
    
    double p[DIRECTIONS];
    double r = (double)rand()/(double)RAND_MAX;
    double sum = 0.0;
    int act;
 
    // lines below calculate a softmax function   
    for (int i = 0; i < DIRECTIONS; i++) {
        sum += exp(BETA*a[i]);
    }
    
    // get softmax probabilities
    for (int i = 0; i < DIRECTIONS; i++) {
        p[i] = exp(BETA*a[i])/sum;
        if (printP) {
            fprintf(stdout,"%f\t", p[i]);
        }
    }
    sum = 0.0;
    
    
    // choose an action based on the probability distribution
    for (int i = 0; i < DIRECTIONS; i++) {
        sum += p[i];
        if (sum > r) {
            act = i;
            i = DIRECTIONS;
        }
    }
    if (printP) {
        fprintf(stdout, "%i\n", act);
    }
    return act;
}

// get the distance between two vectors. assuming these are 2D vectors
double distance(double x1, double x2, double y1, double y2) {
    return sqrt(pow(x1-y1,2.0) + pow(x2-y2,2.0));
}

// get all the place cell activities based on the rat/robot position
void update_place_cells (double *rat) {

    for (int i = 0; i < numPC; i++) {
        // get place cell activity
        pc[i] = place_cell(rat[0], rat[2], pcX[i], pcY[i]);
    }

}

void dump_place_cells (double *rat) {

    FILE *fp;
    double pcDump[numPC];
        
    for (int i = 0; i < numPC; i++) {
        // get place cell activity
        pcDump[i] = place_cell(rat[0], rat[2], pcX[i], pcY[i]);
    }
    fp = fopen("mwm_pcact.txt", "w");
    for (int i = 0; i < numPC; i++) {
        fprintf(fp,"%f\t%f\t%f\n",pcX[i],pcY[i],pcDump[i]);
    }
    fclose(fp);
}

// Update the weights, calculate actor and critic value
double update_weights (int act, double vpre, bool reward) {
    double v = 0.0;
    double delta;
    
    for (int i = 0; i < DIRECTIONS; i++) {
        a[i] = 0.0;
    }
    
    for (int i = 0; i < numPC; i++) {
        // calculate current value for critic based on weight and place cell activity
        v += w[i]*pc[i];
        
        // calculate actor values based on weights and place cell activity
        for (int j = 0; j < DIRECTIONS; j++) {
            a[j] += z[j][i]*pc[i];
        }
    }
    
    // Calculate the Temporal Difference delta rule
    delta = reward*REWARD + (v - vpre);
    
    for (int i = 0; i < numPC; i++) {
        
        // update the critic's weights
        w[i] += LEARNING_RATE*delta*pc[i];
        
        // update the actor's weights
        z[act][i] += LEARNING_RATE*delta*pc[i];
    }
    
    return v;
}


int main(int argc, char **argv) {
    wb_robot_init();
    
    // do this once only
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("FireBird");
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
    
    // Set the initial position of the robot using the supervisor
    wb_supervisor_field_set_sf_vec3f(trans_field, startingTranslation[0]);
    wb_supervisor_field_set_sf_rotation(rot_field, startingRotation[0]);
    
    printf("Morris Water Maze using Nex Fire Bird 6 robot\n");
    
    int i;
    WbDeviceTag ps[8], left_motor, right_motor;
    WbDeviceTag cmpXY1, cmpZ1;
    
    char ps_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    double head_direction[DIRECTIONS] = {0, 45, 90, 135, 180, 225, 270, 315};
    
    for (i = 0; i < 8; ++i) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }
    
    // Enable compass
    cmpXY1 = wb_robot_get_device("compassXY_01");
    wb_compass_enable(cmpXY1, TIME_STEP);
    cmpZ1 = wb_robot_get_device("compassZ_01");
    wb_compass_enable(cmpZ1, TIME_STEP);
    
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    
    int t = 0;
    bool turning = false;
    int dir = 0;
    double vpre = 0.0;
    init_place_cells ();
    double rad;
    double bearing;
    
    int trial = 0;
    int tLatency = 0;
    FILE *fpLatency;
    char fnLatency[256];
    FILE *fpZ;
    bool found = false;
    double rat[3];
    
    for (int i = 0; i < 3; i++) {
        rat[i] = startingTranslation[0][i];
    }
    srand(SEED);
    
    fpLatency = fopen("mwm_latency.txt", "w");
    
    while ((wb_robot_step(TIME_STEP) != -1) && (trial < TRIALS)) {
        
        double ps_values[8];
        for (i = 0; i < 8; ++i) {
            ps_values[i] = wb_distance_sensor_get_value(ps[i]);
            // printf("%f ", ps_values[i]);
        }
        // printf("\n");
        

        // printf("MX %f ", cmpXY[0]);
        // printf("MY %f ", cmpXY[1]);
        // printf("MZ %f ", cmpZ[2]);
        // printf("rad = %f bearing = %f", rad, bearing);
        // printf("\n");

 
        // check if the robot hit the arena wall.       
        double threshold = 0.25;       
        bool left_obstacle = ps_values[1] < threshold || ps_values[0] < threshold;
        bool right_obstacle = ps_values[3] < threshold || ps_values[4] < threshold;
        bool front_obstacle = ps_values[2] < threshold;
        
        // init speeds
        double left_speed = 0.0;
        double right_speed = 0.0;
        
        double heading_error;

        // use the supervisor to get the current position of the robot. 
        // calculate a distance from the robot to the platform        
        const double *trans_value = wb_supervisor_field_get_sf_vec3f(trans_field);
        double distance = sqrt(pow(PLATFORM_X-trans_value[0],2.0)+pow(PLATFORM_Z-trans_value[2],2.0));
          
        // check if the robot found the platform          
        if (distance < PLATFORM_RADIUS) {
            found = true;
        }
        // check if the robot hit a wall
        else if (front_obstacle || left_obstacle || right_obstacle) {
            // printf("obstacle avoidance\n", bearing, head_direction[dir]);
            // modify speeds according to obstacles
            if (front_obstacle) {
                // turn back, but slightly right to not block the robot
                left_speed = 0.0;
                right_speed = -1.0;
            } else if (left_obstacle) {
                // turn right
                left_speed = 3.0;
                right_speed = -1.0;
            } else if (right_obstacle) {
                // turn left
                left_speed = -1.0;
                right_speed = 3.0;
            }
            t = 1;
        }
        // check if the robot is in the middle of a turn. Use the compass to rotate to the 
        // desired heading. rotate in the direction that is shortest to the desired heading
        else if (turning) {
                // Read compass
            const double *cmpXY, *cmpZ;
            cmpXY = wb_compass_get_values(cmpXY1);
            cmpZ = wb_compass_get_values(cmpZ1);
        
            // calculate bearing
            rad = atan2(cmpXY[0], cmpZ[2]);
            bearing = (rad) / 3.1428 * 180.0;
            if (bearing < 0.0) {
                bearing = bearing + 360.0;
            }
            // pointing close to the desired heading
            heading_error = head_direction[dir] - bearing;
            if (fabs(heading_error) < HD_RES) {
                turning = false;
                t = 1;
            }
            else if ((head_direction[dir] == 0) && (bearing > 355.0)) {
                turning = false;
                t = 1;
            }
            else if (heading_error > 0) {
                if (fabs(heading_error) < 180.0) {
                    // turn right
                    left_speed = 3.0;
                    right_speed = -1.0;
                }
                else {
                    // turn left
                    left_speed = -1.0;
                    right_speed = 3.0;
                }
            }
            else {
                if (fabs(heading_error) < 180.0) {
                    // turn left
                    left_speed = -1.0;
                    right_speed = 3.0;
                }
                else {
                    // turn right
                    left_speed = 3.0;
                    right_speed = -1.0;
                }
            }
        }
        // move in the desired heading. update counter for forward movement
        else {
            left_speed = 2.0;
            right_speed = 2.0;
            t++;
        }
        
        // write actuators inputs
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
        
        // if the platform was found or it is time to select a new heading
        // update the actor and critic, choose a new heading
        if (found || (!turning && ((t % TURN_RATE) == 0))) {
            turning = true;
            for (int i = 0; i < 3; i++) {
                rat[i] = trans_value[i];
            }
            update_place_cells (rat);                    
            vpre = update_weights (dir, vpre, found); 
            
            if (found) {
                fprintf(stdout, "\nFound platform on trial %i at time %i, ", trial, tLatency);
                fprintf(stdout, "rat (%f, %f)\n", rat[0], rat[2]);
                
                // set new starting position for the rat/robot
                wb_supervisor_field_set_sf_vec3f(trans_field, startingTranslation[trial % 4]);
                wb_supervisor_field_set_sf_rotation(rot_field, startingRotation[trial % 4]);
                fprintf(fpLatency,"%i\n", tLatency);
                tLatency = 0;
                trial++;
                found = false;
                t = 1;
                vpre = 0.0;
            }
            else {
                dir = action_select(false);
            }          
        }
        tLatency++;
    };
    fclose(fpLatency);
        
    fpZ = fopen("mwm_z.txt", "w");
    for (int i = 0; i < DIRECTIONS; i++) {
        for (int j = 0; j < numPC; j++) {
            fprintf(fpZ,"%f\t", z[i][j]);
        }
        fprintf(fpZ, "\n");
    }
    fclose(fpZ);
  
    wb_robot_cleanup();
    
    return 0;
}
