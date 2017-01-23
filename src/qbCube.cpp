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

#include "qbCube.h"

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

//-----------------------------------------------------
//                                               qbCube
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbCube class
/ *****************************************************
/   argument:
/       - id, ID of the cube
/   return:
/
*/

qbCube::qbCube(const int id) : qbInterface(id) {}

//-----------------------------------------------------
//                                               qbCube
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbCube class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer to the cube
/       - id, ID of the cube
/   return:
/
*/

qbCube::qbCube(comm_settings* cs, const int id) : qbInterface(cs, id) {}


//-----------------------------------------------------
//                                              ~qbCube
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbCube class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbCube::~qbCube() {}


//========================== OTHER FUNCTIONS ===================================


//-----------------------------------------------------
//                                      setPosAndPreset
//-----------------------------------------------------

/*
/ *****************************************************
/ Set position and stiffness preset of the cube, default
/ input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPreset, stiffness preset
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

void qbCube::setPosAndPreset(float position, float stiffPreset, angular_unit unit) {

    // Check input values consistence

    if (stiffPreset > DEFAULT_STIFFNESS) {

        stiffPreset = DEFAULT_STIFFNESS;
        cerr << "WARNING: Preset saturated to " << stiffPreset << endl;

    } else if (stiffPreset < 0) {

        stiffPreset = 0;
        cerr << "WARNING: Preset saturated to " << stiffPreset << endl;

    }

    // Convert position in 'deg' if needed

    if (unit == RAD){
        position *= (180.0 / M_PI);
        stiffPreset *= (180.0 / M_PI);
    }else{
        if (unit == TICK){
            position /= DEG_TICK_MULTIPLIER;
            stiffPreset /= DEG_TICK_MULTIPLIER;
        }
    }

    // Axis Direction

    position *= axis_dir_;

    // Check Max Position available

    if (position > (DEFAULT_SUP_LIMIT / DEG_TICK_MULTIPLIER) - stiffPreset) {

        position = (DEFAULT_SUP_LIMIT / DEG_TICK_MULTIPLIER) - stiffPreset;
        cerr << "WARNING: Position saturated to " << position << endl;

    } else if (position < (DEFAULT_INF_LIMIT / DEG_TICK_MULTIPLIER) + stiffPreset) {

            position = (DEFAULT_INF_LIMIT / DEG_TICK_MULTIPLIER) + stiffPreset;
            cerr << "WARNING: Position saturated to " << position << endl;
    }

    short int pos, sPreset;
    short int curr_ref[2];

    pos = position * DEG_TICK_MULTIPLIER;
    sPreset = stiffPreset * DEG_TICK_MULTIPLIER;

    // Set position for the 2 motors of the cube

    curr_ref[0] = pos - sPreset;
    curr_ref[1] = pos + sPreset;

    // Call API function

    commSetInputs(cube_comm_, id_, curr_ref);
}


//-----------------------------------------------------
//                                      getPosAndPreset
//-----------------------------------------------------

/*
/ *****************************************************
/ Get position and stiffness preset of the cube, default
/ input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPreset, stiffness preset
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPosAndPreset(float* position, float* preset, angular_unit unit) {
 
    short int meas[3];

    // Get measurements
    if(!getMeas(meas))
        return false;

    // Return position in the right unit

    if (unit == DEG){
        *position = (((float) meas[2]) / DEG_TICK_MULTIPLIER);
        *preset = fabs(((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER);
    }
    else{
        if (unit == RAD){
            *position = (((float) meas[2]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
            *preset = fabs((((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER) * (M_PI / 180));
        }
        else{
            *position = (float) meas[2];
            *preset = fabs(((float)(meas[0] - meas[1]) / 2));
        }
    }

    // Compute preset value
    *preset = fabs(((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER);

    return true;
}


//-----------------------------------------------------
//                                  setPosAndPresetPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Set position and stiffness percentage of the cube,
/ default input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPerc, stiffness percentage; range: [0 - 32767]
/       - unit, measurement unit for position [rad or deg]
/
/   return:
/       true  on success
/       false on failure
/
*/

void qbCube::setPosAndPresetPerc(float position, float stiffPerc, angular_unit unit) {

    // Check input values consistence

    if (stiffPerc > 100) {

        stiffPerc = 100;
        cerr << "WARNING: Stiffness percentage saturated to " << stiffPerc << endl;

    } else if (stiffPerc < 0) {

        stiffPerc = 0;
        cerr << "WARNING: Stiffness percentage saturated to " << stiffPerc << endl;

    }

    // Convert position in 'deg' if needed

    if (unit == RAD) 
        position *= (180.0 / M_PI);
    else
        if (unit == TICK)
            position /= DEG_TICK_MULTIPLIER;

    // Axis Direction

    position *= axis_dir_;

    // XXX TODO: Check Max Position available

    short int curr_ref[2];

    curr_ref[0] = (short int)(position * DEG_TICK_MULTIPLIER);
    curr_ref[1] = (short int)((stiffPerc * 32767.0) / 100.0);

    // Call API function

    commSetPosStiff(cube_comm_, id_, curr_ref);

}


//-----------------------------------------------------
//                                  getPosAndPresetPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Set position and stiffness percentage of the cube,
/ default input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPerc, stiffness percentage; range: [0 - 32767]
/       - unit, measurement unit for position [rad or deg]
/
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPosAndPresetPerc(float* position, float* stiffPerc, angular_unit unit) {

    return true;
}

//-----------------------------------------------------
//                                               getPos
//-----------------------------------------------------

/*
/ *****************************************************
/ Get position angle in the chosen unit of measure
/ *****************************************************
/   arguments:
/       - angle, getted position in angle
/       - unit, unit of measure [rad o deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPos(float* angle, angular_unit unit) {

    short int meas[3];

    // Get measurements
    if(!getMeas(meas))
        return false;

    // Return position in the right unit

    if (unit == DEG)
        *angle = (((float) meas[2]) / DEG_TICK_MULTIPLIER);
    else
        if (unit == RAD)
            *angle = (((float) meas[2]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
        else
            *angle = (float) meas[2];

    return true;
}

//-----------------------------------------------------
//                                            getPreset
//-----------------------------------------------------

/*
/ *****************************************************
/ Get preset of the cube
/ *****************************************************
/   parameters:
/   return:
/       - preset value
/
*/

bool qbCube::getPreset(float* preset, angular_unit unit) {

    short int meas[3];

    // Get position
    if(!getMeas(meas))
        return false;

    if (unit == DEG)
        *preset = fabs(((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER);
    else
        if (unit == RAD)
            *preset = fabs((((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER) * (M_PI / 180.0));
        else
            *preset = fabs(((float)(meas[0] - meas[1]) / 2));    

    return true;
}


//-----------------------------------------------------
//                                        getPresetPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Get preset of the cube
/ *****************************************************
/   parameters:
/   return:
/       - preset value
/
*/

bool qbCube::getPresetPerc(float* preset) {

    // in realta non so come ottenerla

    return true;
}


//-----------------------------------------------------
//                                         getPPAndCurr
//-----------------------------------------------------

/*
/ *****************************************************
/ Get positions (Pos and Preset) and current for each 
/ motor, default input is set to radiants
/ *****************************************************
/   argumnets:
/       - position, reference angle
/       - stiffPreset, stiffness preset
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPPAndCurr(float* position, float* preset, float* current, angular_unit unit) {
     
    short int meas[3], curr[2];
     
    // Get position measurements and currents
    if(!getMeasAndCurr(meas, curr))
        return false;

    // Return position in the right unit
    if (unit == DEG){
        *position = (((float) meas[2]) / DEG_TICK_MULTIPLIER);
        *preset = fabs(((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER);
    }
    else{
        if (unit == RAD){
            *position = (((float) meas[2]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
            *preset = fabs((((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER) * (M_PI / 180));
        }
        else{
            *position = (float) meas[2];
            *preset = fabs(((float)(meas[0] - meas[1]) / 2));
        }
    }

    // Compute preset value
    *preset = fabs(((float)(meas[0] - meas[1]) / 2) / DEG_TICK_MULTIPLIER);

    current[0] = curr[0];
    current[1] = curr[1];

    return true;
}

//-----------------------------------------------------
//                                       getMeasAndCurr
//-----------------------------------------------------

/*
/ *****************************************************
/ Get positions (Pos and Preset) and current for each 
/ motor, default input is set to radiants
/ *****************************************************
/   arguments:
/       - position, measured angle
/       - current, current measured
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbCube::getPosAndCurr(float* position, float* current, angular_unit unit) {
     
    short int meas[3], curr[2];     

    // Get position measurements and currents
    if(!getMeasAndCurr(meas, curr))
        return false;

    // Return position in the right unit
 
    if (unit == DEG){

        position[0] = (((float) meas[0]) / DEG_TICK_MULTIPLIER);
        position[1] = (((float) meas[1]) / DEG_TICK_MULTIPLIER);
        position[2] = (((float) meas[2]) / DEG_TICK_MULTIPLIER);

    }
    else
        if (unit == RAD){
            position[0] = (((float) meas[0]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
            position[1] = (((float) meas[1]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
            position[2] = (((float) meas[2]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
        }else {
            position[0] = (float) meas[0];
            position[1] = (float) meas[1];
            position[2] = (float) meas[2];
        }

    current[0] = curr[0];
    current[1] = curr[1];

    return true;
}


/* END OF FILE */