// BSD 3-Clause License

// Copyright (c) 2017, qbrobotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>

#include "qbInterface.h"

//-----------------------------------------------------
//                                          qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbInterface class
/ *****************************************************
/   argument:
/       - id, ID of the board
/   return:
/
*/

qbInterface::qbInterface(const int id) {

    // Set axis direction

    if (id >= 0)
        axis_dir_ = 1;
    else
        axis_dir_ = -1;

    id_ = abs(id);

    cube_comm_ = NULL;
}


//-----------------------------------------------------
//                                          qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbInterface class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer
/       - id, ID of the board
/   return:
/
*/

qbInterface::qbInterface(comm_settings* cs, const int id) {

    // Set axis direction

    if (id >= 0)
        axis_dir_ = 1;
    else
        axis_dir_ = -1;

    id_ = abs(id);

    cube_comm_ = cs;
}


//-----------------------------------------------------
//                                         ~qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbInterface class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbInterface::~qbInterface() {

    close();

}


//======================= OTHER FUNCTIONS ======================================

//-----------------------------------------------------
//                                                 open
//-----------------------------------------------------

/*
/ *****************************************************
/ Open serial communication with cube
/ *****************************************************
/   arguments:
/       port, the communication port
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::open(const char* port) {

    // Check Connection State

    if(cube_comm_ != NULL) {
        cerr << "WARNING: Port already opened" << endl;
        return false;
    }

    cube_comm_ = new comm_settings;

    // Establish serial connection

    openRS485(cube_comm_, port);

    if (cube_comm_->file_handle == INVALID_HANDLE_VALUE) {
        cerr << "ERROR: Unable to open port" << endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                                close
//-----------------------------------------------------

/*
/ *****************************************************
/ Close serial communication
/ *****************************************************
/   arguments:
/   return:
/
*/

void qbInterface::close() {

    deactivate();

    // close commnication
    if (cube_comm_ != NULL) {
        closeRS485(cube_comm_);
        delete cube_comm_;
        cube_comm_ = NULL;
    }
}


//-----------------------------------------------------
//                                             activate
//-----------------------------------------------------

/*
/ *****************************************************
/ Activate the cube
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::activate() {

    char status = 0;

    // Check connection status
    if (cube_comm_ == NULL) {
        cerr << "ERROR:[qbInterface activate()] Port not opened" << endl;
        return false;
    }

    // Activate board
    commActivate(cube_comm_, id_, 1);

    // Wait setting time

    usleep(1000);

    // Check if board is active
    commGetActivate(cube_comm_, id_, &status);

    // Check status
    if (!status){
        cerr << "Unable to activate ID: " << id_ <<  endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                           deactivate
//-----------------------------------------------------

/*
/ *****************************************************
/ Active the cube
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::deactivate() {

    char status = 0;

    // Check connection status
    if (cube_comm_ == NULL){
        //cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    // Deactivate board
    commActivate(cube_comm_, id_, 0);

    // Wait setting time
    usleep(1000);

    // Check if the board is inactive
    commGetActivate(cube_comm_, id_, &status);

    if (status) {
        cerr << "Unable to deactivate" << endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                              getMeas
//-----------------------------------------------------

/*
/ *****************************************************
/ Get measurement of positions [1, 2, 3]  in ticks
/ *****************************************************
/   arguments:
/       - meas, 3 elements array pointer for measurements
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::getMeas(short int* meas) {

    if (cube_comm_ == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    if (commGetMeasurements(cube_comm_, id_, meas) < 0)
        return false;

    // Axis direction
    
    meas[0] *= axis_dir_;
    meas[1] *= axis_dir_;
    meas[2] *= axis_dir_;

    return true;
}


//-----------------------------------------------------
//                                            setInputs
//-----------------------------------------------------

/*
/ *****************************************************
/ Set board inputs in ticks [1, 2]
/ *****************************************************
/   arguments:
/       - inputs, 2 elements array pointer for inputs
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::setInputs(short int* inputs) {

    if (cube_comm_ == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    // Axis direction

    inputs[0] *= axis_dir_;
    inputs[1] *= axis_dir_;

    commSetInputs(cube_comm_, id_, inputs);

    return true;
}

//-----------------------------------------------------
//                                                getID
//-----------------------------------------------------

/*
/ *****************************************************
/ Getter for variable ID
/ *****************************************************
/   arguments:
/   return:
/       ID
/
*/

int qbInterface::getID() {

    return id_;
}

//-----------------------------------------------------
//                                       getMeasAndCurr
//-----------------------------------------------------

/*
/ *****************************************************
/ Get measurement of positions [1, 2, 3]  in ticks and
/ currents of [1, 2] motors
/ *****************************************************
/   arguments:
/       - curr, 3 elements array pointer for measurements
/       - meas, 3 elements array pointer for measurements
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::getMeasAndCurr(short int* meas, short int* curr) {

    short int aux[5];

    if (cube_comm_ == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    if (commGetCurrAndMeas(cube_comm_, id_, aux) < 0)
        return false;

    // Current

    curr[0] = aux[0];
    curr[1] = aux[1];

    // Motor pos and axis direction
    
    meas[0] = aux[2] * axis_dir_;
    meas[1] = aux[3] * axis_dir_;
    meas[2] = aux[4] * axis_dir_;

    return true;
}


/* END OF FILE */
