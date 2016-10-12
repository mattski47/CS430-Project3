/* 
 * File:   raycast.c
 * Author: Matthew
 *
 * Created on October 11, 2016, 6:57 PM
 */


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#define SPHERE 0
#define PLANE 1
#define SPOTLIGHT 0
#define POINT 1
#define MAXCOLOR 255

// struct that stores object data
typedef struct {
    int kind;
    double color[3];
    double position[3];
    union {
        struct {
            double radius;
        } sphere;
        struct {
            double normal[3];
        } plane;
    };
} Object;

typedef struct {
    int kind;
    double color[3];
    double position[3];
    double radial_a2;
    union {
        struct {
            double direction[3];
            double radial_a1;
            double radial_a0;
            double angular_a0;
        } spotlight;
    };
} Light;

// struct that stores color data
typedef struct {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} Pixel;

// inline function for square
static inline double sqr(double v) {
    return v*v;
}

// inline function for normalizing a vector
static inline void normalize(double* v) {
    double len = sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
    
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
}

typedef double* v3;
// add vector "a" with vector "b" and puts result in vector "c"
static inline void v3_add(v3 a, v3 b, v3 c) {
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

// subtract vector "b" from vector "a" and puts result in vector "c"
static inline void v3_subtract(v3 a, v3 b, v3 c) {
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}

// scale vector "a" by value "s" and puts result in vector "c"
static inline void v3_scale(v3 a, double s, v3 c) {
    c[0] = s * a[0];
    c[1] = s * a[1];
    c[2] = s * a[2];
}

void read_scene(FILE*);
void set_camera(FILE*);
void parse_sphere(FILE*, Object*);
void parse_plane(FILE*, Object*);
void parse_light(FILE*, Light*);
double sphere_intersect(double*, double*, double*, double);
double plane_intersect(double*, double*, double*, double*);
void skip_ws(FILE*);
void expect_c(FILE*, int);
int next_c(FILE*);
char* next_string(FILE*);
double next_number(FILE*);
double* next_vector(FILE*);
void output_p6(FILE*);

// initialize input file line counter
int line = 1;

// create arrays for storing objects and pixels
Object** objects;
Light** lights;
Pixel* pixmap;

// default camera
double h = 1;
double w = 1;
double cx = 0;
double cy = 0;

// height, width of output image
int M;
int N;

int main(int argc, char** argv) {
    // check for correct number of inputs
    if (argc != 5) {
        fprintf(stderr, "Error: Arguments should be in format: 'width' 'height' 'source' 'dest'.\n");
        exit(1);
    }
    
    // check that 'width' is a non-zero number
    N = atoi(argv[1]);
    if (N == 0) {
        fprintf(stderr, "Error: Argument 1, 'width' must be a non-zero integer.\n");
        exit(1);
    }
    
    // check that 'height' is a non-zero number
    M = atoi(argv[2]);
    if (M == 0) {
        fprintf(stderr, "Error: Argument 2, 'height' must be a non-zero integer.\n");
        exit(1);
    }
    
    // open and check input file
    FILE* json = fopen(argv[3], "r");
    if (json == NULL) {
        fprintf(stderr, "Error: Could not open file '%s'.\n", argv[3]);
        exit(1);
    }
    
    // allocate space for 128 objects/lights
    objects = malloc(sizeof(Object*)*129);
    lights = malloc(sizeof(Light*)*129);
    
    read_scene(json);
    
    // calculate pixel height, width
    double pixheight = h/M;
    double pixwidth = w/N;
    
    // allocate space for number of pixels needed
    pixmap = malloc(sizeof(Pixel)*M*N);
    // initialize pixmap index
    int index = 0;
    double Ro[3];
    double Rd[3];
    
    // got through each spot pixel by pixel to see what color it should be
    for (int y=0; y<M; y++) {
        for (int x=0; x<N; x++) {
            // ray origin
            Ro = {cx, cy, 0};
            // ray destination
            Rd = {cx - (w/2) + pixwidth*(x + 0.5),
                  cy - (h/2) + pixheight*(y + 0.5),
                  1};
            normalize(Rd);
            
            double t = 0;
            double best_t = INFINITY;
            Object* closest_object;
            // look for intersection of an object 
            for (int i=0; objects[i] != NULL; i++) {
                switch (objects[i]->kind) {
                    case SPHERE:
                        t = sphere_intersect(Ro, Rd, objects[i]->position, objects[i]->sphere.radius);
                        break;
                    case PLANE:
                        t = plane_intersect(Ro, Rd, objects[i]->position, objects[i]->plane.normal);
                        break;
                    default:
                        fprintf(stderr, "Error: Unknown object.\n");
                        exit(1);
                }
                
                // save object if it intersects closer to the camera
                if (t > 0 && t < best_t) {
                    best_t = t;
                    closest_object = malloc(sizeof(Object));
                    memcpy(closest_object, objects[i], sizeof(Object));
                }
            }
            
            double* color = malloc(sizeof(double)*3);
            color[0] = 0;
            color[1] = 0;
            color[2] = 0;
            
            double Ron[3];
            double Rdn[3];
            double temp[3];
            for (int j=0; lights[j] != NULL; j++) {
                v3_scale(Rd, best_t, temp);
                v3_add(temp, Ro, Ron);
                v3_subtract(lights[j]->position, Ron, Rdn);
                
                for (int k=0; objects[k] != NULL; k++) {
                    if (objects[k] == closest_object)
                        continue;
                    
                    switch (objects[k]->kind) {
                        case SPHERE:
                            t = sphere_intersect(Ron, Rdn, objects[k]->position, objects[k]->sphere.radius);
                            break;
                        case PLANE:
                            t = plane_intersect(Ron, Rdn, objects[k]->position, objects[k]->plane.normal);
                            break;
                        default:
                            fprintf(stderr, "Error: Unknown object.\n");
                            exit(1);
                    }
                }
            }
            
            // write color of object with the best intersection or black if there was none at this pixel
            if (best_t > 0 && best_t != INFINITY) {
                pixmap[index].r = (unsigned char)(object->color[0]*MAXCOLOR);
                pixmap[index].g = (unsigned char)(object->color[1]*MAXCOLOR);
                pixmap[index].b = (unsigned char)(object->color[2]*MAXCOLOR);
            } else {
                pixmap[index].r = 0;
                pixmap[index].g = 0;
                pixmap[index].b = 0;
            }
            
            index++;
        }
    }
    
    // open and check output file location
    FILE* output = fopen(argv[4], "w");
    if (output == NULL) {
        fprintf(stderr, "Error: Could not create file '%s'.\n", argv[4]);
        exit(1);
    }
    
    // write pixel data to output file then close it
    output_p6(output);
    fclose(output);
    
    return (EXIT_SUCCESS);
}

// reads object data from a json file
void read_scene(FILE* json) {
    // next char in file
    int c;
    
    // look for start of json file ([)
    skip_ws(json);
    expect_c(json, '[');
    skip_ws(json);
    
    // initialize object array index
    int i = 0;
    int j = 0;
    // read all objects found in file
    while (1) {
        c = next_c(json);
        // check for empty scene
        if (c == ']') {
            fprintf(stderr, "Error: This scene is empty.\n");
            fclose(json);
            objects[i] = NULL;
            return;
        }
        
        // checks for start of an object
        if (c == '{') {
            skip_ws(json);
            
            // check that 'type' field is first
            char* key = next_string(json);
            if (strcmp(key, "type") != 0) {
                fprintf(stderr, "Error: Expected 'type' key. (Line %d)\n", line);
                exit(1);
            }
            
            skip_ws(json);
            expect_c(json, ':');
            skip_ws(json);
            
            // check what object is being read and calls a function depending on what it is
            char* value = next_string(json);
            if (strcmp(value, "camera") == 0) {
                set_camera(json);
            } else if (strcmp(value, "sphere") == 0) {
                // allocate space for an object
                objects[i] = malloc(sizeof(Object));
                parse_sphere(json, objects[i]);
                i++;
            } else if (strcmp(value, "plane") == 0) {
                // allocate space for an object
                objects[i] = malloc(sizeof(Object));
                parse_plane(json, objects[i]);
                i++;
            } else if (strcmp(value, "light") == 0) {
                lights[j] = malloc(sizeof(Light));
                parse_light(json, lights[j]);
                j++;
            } else {
                fprintf(stderr, "Error: Unknown type '%s'. (Line %d)\n", value, line);
                exit(1);
            }
            
            // check for more objects
            skip_ws(json);
            c = next_c(json);
            if (c == ',') {
                skip_ws(json);
            } else if (c == ']') {
                objects[i] = NULL;
                lights[j] = NULL;
                fclose(json);
                return;
            } else {
                fprintf(stderr, "Error: Expecting ',' or ']'. (Line %d)\n", line);
                exit(1);
            }
            
            // check if scene has too many objects in it
            if (i == 129 || j == 129) {
                objects[i] = NULL;
                lights[j] = NULL;
                fclose(json);
                fprintf(stderr, "Error: Too many objects in file.\n");
                return;
            }
        } else {
            fprintf(stderr, "Error: Expecting '{'. (Line %d)\n", line);
            exit(1);
        }
    }
}

// reads camera data from json file
void set_camera(FILE* json) {
    int c;
    skip_ws(json);
    
    // checks fields in camera
    while (1) {
        c = next_c(json);
        if (c == '}') {
            break;
        } else if (c == ',') {
            skip_ws(json);
            char* key = next_string(json);
            
            skip_ws(json);
            expect_c(json, ':');
            skip_ws(json);
            
            double value = next_number(json);
            // checks that field can be in camera and sets 'w' or 'h' if found
            if (strcmp(key, "width") == 0) {
                w = value;
            } else if (strcmp(key, "height") == 0) {
                h = value;
            } else {
                fprintf(stderr, "Error: Unknown property '%s' for 'camera'. (Line %d)\n", key, line);
                exit(1);
            }
        }
    }
}

// gets sphere information and stores it into an object
void parse_sphere(FILE* json, Object* object) {
    int c;
    
    // used to check that all fields for a sphere are present
    int hasradius = 0;
    int hascolor = 0;
    int hasposition = 0;
    
    // set object kind to sphere
    object->kind = SPHERE;
    
    skip_ws(json);
    
    // check fields for this sphere
    while(1) {
        c = next_c(json);
        if (c == '}') {
            break;
        } else if (c == ',') {
            skip_ws(json);
            char* key = next_string(json);

            skip_ws(json);
            expect_c(json, ':');
            skip_ws(json);
            
            // set values for this sphere depending on what key was read and sets its 'boolean' to reflect the found field
            if (strcmp(key, "radius") == 0) {
                object->sphere.radius = next_number(json);
                hasradius = 1;
            } else if (strcmp(key, "color") == 0) {
                double* value = next_vector(json);
                object->color[0] = value[0];
                object->color[1] = value[1];
                object->color[2] = value[2];
                hascolor = 1;
            } else if (strcmp(key, "position") == 0) {
                double* value = next_vector(json);
                object->position[0] = value[0];
                object->position[1] = -value[1];
                object->position[2] = value[2];
                hasposition = 1;
            } else {
                fprintf(stderr, "Error: Unknown property '%s' for 'sphere'. (Line %d)\n", key, line);
                exit(1);
            }
        }
    }
    
    // check for missing fields
    if (!hasradius) {
        fprintf(stderr, "Error: Sphere missing 'radius' field. (Line %d)\n", line);
        exit(1);
    }
    
    if (!hascolor) {
        fprintf(stderr, "Error: Sphere missing 'color' field. (Line %d)\n", line);
        exit(1);
    }
    
    if (!hasposition) {
        fprintf(stderr, "Error: Sphere missing 'position' field. (Line %d)\n", line);
        exit(1);
    }
}

// gets plane information and stores it into an object
void parse_plane(FILE* json, Object* object) {
    int c;
    
    // used to check that all fields for a plane are present
    int hasnormal = 0;
    int hascolor = 0;
    int hasposition = 0;
    
    // set object kind to plane
    object->kind = PLANE;
    
    skip_ws(json);
    
    // check fields for this plane
    while(1) {
        c = next_c(json);
        if (c == '}') {
            break;
        } else if (c == ',') {
            skip_ws(json);
            char* key = next_string(json);

            skip_ws(json);
            expect_c(json, ':');
            skip_ws(json);
            
            // set values for this plane depending on what key was read and sets its 'boolean' to reflect the found field
            double* value = next_vector(json);
            if (strcmp(key, "normal") == 0) {
                object->plane.normal[0] = value[0];
                object->plane.normal[1] = value[1];
                object->plane.normal[2] = value[2];
                hasnormal = 1;
            } else if (strcmp(key, "color") == 0) {
                object->color[0] = value[0];
                object->color[1] = value[1];
                object->color[2] = value[2];
                hascolor = 1;
            } else if (strcmp(key, "position") == 0) {
                object->position[0] = value[0];
                object->position[1] = value[1];
                object->position[2] = value[2];
                hasposition = 1;
            } else {
                fprintf(stderr, "Error: Unknown property '%s' for 'sphere'. (Line %d)\n", key, line);
                exit(1);
            }
        }
    }
    
    // check for missing fields
    if (!hasnormal) {
        fprintf(stderr, "Error: Plane missing 'normal' field. (Line %d)\n", line);
        exit(1);
    }
    
    if (!hascolor) {
        fprintf(stderr, "Error: Plane missing 'color' field. (Line %d)\n", line);
        exit(1);
    }
    
    if (!hasposition) {
        fprintf(stderr, "Error: Plane missing 'position' field. (Line %d)\n", line);
        exit(1);
    }
}

void parse_light(FILE* json, Light* light) {
    int c;
    
    int hascolor = 0;
    int hasposition = 0;
    int hasdirection = 0;
    int hasr2 = 0;
    int hasr1 = 0;
    int hasr0 = 0;
    int hasa0 = 0;
    
    skip_ws(json);
    
    while(1) {
        c = next_c(json);
        if (c == '}') {
            break;
        } else if (c == ',') {
            skip_ws(json);
            char* key = next_string(json);

            skip_ws(json);
            expect_c(json, ':');
            skip_ws(json);
            
            // set values for this light depending on what key was read and sets its 'boolean' to reflect the found field
            if (strcmp(key, "color") == 0) {
                double* value = next_vector(json);
                light->color[0] = value[0];
                light->color[1] = value[1];
                light->color[2] = value[2];
                hascolor = 1;
            } else if (strcmp(key, "position") == 0) {
                double* value = next_vector(json);
                light->position[0] = value[0];
                light->position[1] = value[1];
                light->position[2] = value[2];
                hasposition = 1;
            } else if (strcmp(key, "direction") == 0) {
                double* value = next_vector(json);
                light->spotlight.direction[0] = value[0];
                light->spotlight.direction[1] = value[1];
                light->spotlight.direction[2] = value[2];
                hasdirection = 1;
            } else if (strcmp(key, "radial-a2") == 0) {
                light->radial_a2 = next_number(json);
                hasr2 = 1;
            } else if (strcmp(key, "radial-a1") == 0) {
                light->spotlight.radial_a1 = next_number(json);
                hasr1 = 1;
            } else if (strcmp(key, "radial-a0") == 0) {
                light->spotlight.radial_a0 = next_number(json);
                hasr0 = 1;
            } else if (strcmp(key, "angular-a0") == 0) {
                light->spotlight.angular_a0 = next_number(json);
                hasa0 = 1;
            } else {
                fprintf(stderr, "Error: Unknown property '%s' for 'light'. (Line %d)\n", key, line);
                exit(1);
            }
        }
    }
    
    if (!hascolor) {
        fprintf(stderr, "Error: Light missing 'color' field. (Line %d)\n", line);
        exit(1);
    }
    
    if (!hasposition) {
        fprintf(stderr, "Error: Light missing 'position' field. (Line %d)\n", line);
        exit(1);
    }
    
    if (!hasr2) {
        fprintf(stderr, "Error: Light missing 'radial-a2' field. (Line %d)\n", line);
        exit(1);
    }
    
    if (hascolor && hasposition && hasdirection && hasr2 && hasr1 && hasr0 && hasa0) {
        light->kind = SPOTLIGHT;
    } else {
        light->kind = POINT;
    }
}

// calculate the sphere intersect
double sphere_intersect(double* Ro, double* Rd, double* C, double r) {
    double a = sqr(Rd[0]) + sqr(Rd[1]) + sqr(Rd[2]);
    double b = 2*(Rd[0]*(Ro[0]-C[0]) + Rd[1]*(Ro[1]-C[1]) + Rd[2]*(Ro[2]-C[2]));
    double c = sqr(Ro[0]-C[0]) + sqr(Ro[1]-C[1]) + sqr(Ro[2]-C[2]) - sqr(r);
    
    // check determinant
    double det = sqr(b) - 4*a*c;
    if (det < 0)
        return -1;
    
    det = sqrt(det);
    
    // return t value if an intersect was found, otherwise return -1
    double t0 = (-b - det) / (2*a);
    if (t0 > 0)
        return t0;
    
    double t1 = (-b + det) / (2*a);
    if (t1 > 0)
        return t1;
    
    return -1;
}

// calculate the plane intersect
double plane_intersect(double* Ro, double* Rd, double* P, double* N) {
    // dot product of normal and position to find distance 
    double d = N[0]*P[0] + N[1]*P[1] + N[2]*P[2]; 
    double t = -(N[0]*Ro[0] + N[1]*Ro[1] + N[2]*Ro[2] + d) / (N[0]*Rd[0] + N[1]*Rd[1] + N[2]*Rd[2]);
    
    // return t value if an intersect was found, otherwise return -1
    if (t > 0)
        return t;
    
    return -1;
}

// skips white space in file
void skip_ws(FILE* json) {
    int c = next_c(json);
    
    while(isspace(c))
        c = next_c(json);
    
    ungetc(c, json);
}

// check that a certain character is next in file
void expect_c(FILE* json, int d) {
    int c = next_c(json);
    
    if (c == d)
        return;
    
    // error if the character found was not what was expected
    fprintf(stderr, "Error: Expected '%c'. (Line %d)\n", d, line);
    exit(1);
}

// get the next character in file
int next_c(FILE* json) {
    int c = fgetc(json);
    
    // increment line if newline was found in file
    if (c == '\n')
        line++;
    
    // error if end of file found when another character was expected
    if (c == EOF) {
        fprintf(stderr, "Error: Unexpected end of file. (Line %d)\n", line);
        exit(1);
    }
    
    return c;
}

// get next string in file
char* next_string(FILE* json) {
    char buffer[129];
    int c = next_c(json);
    
    // look for start of a string
    if (c != '"') {
        fprintf(stderr, "Error: Expected string. (Line %d)\n", line);
        exit(1);
    }
    
    c = next_c(json);
    
    int i = 0;
    // get characters until the end of the string is found or string becomes bigger than 128 characters
    while (c != '"') {
        // checks length
        if (i >= 128) {
            fprintf(stderr, "Error: Strings longer than 128 characters in length are not supported. (Line %d)\n", line);
            exit(1);
        }
        
        // checks for escape codes
        if (c == '\\') {
            fprintf(stderr, "Error: Strings with escape codes are not supported. (Line %d)\n", line);
            exit(1);
        }
        
        // checks that all characters in file are ASCII
        if (c < 32 || c > 126) {
            fprintf(stderr, "Error: Strings may only contain ASCII characters. (Line %d)\n", line);
            exit(1);
        }
        
        // saves character to buffer and increment index
        buffer[i] = c;
        i++;
        c = next_c(json);
    }
    
    // null terminate string
    buffer[i] = 0;
    return strdup(buffer);
}

// gets next number in file
double next_number(FILE* json) {
    double value;
    // look for number, error if one is not found
    if (fscanf(json, "%lf", &value) != 1) {
        fprintf(stderr, "Error: Number value not found. (Line %d)\n", line);
        exit(1);
    }
    
    return value;
}

// get next 3 number vector in file
double* next_vector(FILE* json) {
    double* v = malloc(3*sizeof(double));
    
    // check for start of vector and first number
    expect_c(json, '[');
    skip_ws(json);
    v[0] = next_number(json);
    skip_ws(json);
    
    // check for second number
    expect_c(json, ',');
    skip_ws(json);
    v[1] = next_number(json);
    skip_ws(json);
    
    // check for third number
    expect_c(json, ',');
    skip_ws(json);
    v[2] = next_number(json);
    skip_ws(json);
    
    // check for end of vector
    expect_c(json, ']');
    return v;
}

// outputs data in buffer to output file
void output_p6(FILE* outputfp) {
    // create header
    fprintf(outputfp, "P6\n%d %d\n%d\n", M, N, MAXCOLOR);
    // writes buffer to output Pixel by Pixel
    fwrite(pixmap, sizeof(Pixel), M*N, outputfp);
}
