#ifndef CONTROLS_HPP
#define CONTROLS_HPP

double computeMatricesFromInputs();
double computeMatricesFromInputs_n(float yaw,float posx,float posz);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();

#endif