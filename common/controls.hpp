#ifndef CONTROLS_HPP
#define CONTROLS_HPP

double computeMatricesFromInputs();
void computeMatricesFromInputs_n(float yaw,float pitch,float posx,float posy,float posz);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();

#endif