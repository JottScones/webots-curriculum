/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*************************************
 *
 * This header defines the properties
 * of the artificial neural network
 *
 **************************************/

#ifndef ANN_H
#define ANN_H

#define SAMPLE_WIDTH 7   // width of the sampled image input
#define SAMPLE_HEIGHT 7  // height of the sampled image input

#define IN_SIZE SAMPLE_WIDTH *SAMPLE_HEIGHT  // network input layer size
#define HID_SIZE 49                          // network hidden layer size
#define HID_SIZE_2 15
#define OUT_SIZE 4                           // network output layer size

// network learning rate (use small number
// for more stable training, but longer
// training time. Use number near 1 for
// faster training, but network may not converge.
#define LEARNING_RATE 0.03

#include "../../lib/backprop.h"  // definitions of the ANN functions

/* GLOBAL DATA FOR NETWORK */
float i1[IN_SIZE];
float W1[HID_SIZE][IN_SIZE];
float o1[HID_SIZE];
float g1[HID_SIZE];

float i2[HID_SIZE];
float W2[HID_SIZE_2][HID_SIZE];
float o2[HID_SIZE_2];
float g2[HID_SIZE_2];

float i3[HID_SIZE_2];
float W3[OUT_SIZE][HID_SIZE_2];
float o3[OUT_SIZE];
float g3[OUT_SIZE];

// Create both layers.  Note that this intializes the layers
// with references to the memory allocated above.
layer_t layers[3] = {{IN_SIZE, HID_SIZE, i1, &W1[0][0], o1, g1}, 
                     {HID_SIZE, HID_SIZE_2, i2, &W2[0][0], o2, g2},
                     {HID_SIZE_2, OUT_SIZE, i3, &W3[0][0], o3, g3}
                     };

// Create the network and initialize with the layers previously
// allocated.
network_t network = {3, LEARNING_RATE, layers};

#endif
