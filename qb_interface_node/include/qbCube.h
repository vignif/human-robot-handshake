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

#ifndef QBCUBE_H
#define QBCUBE_H

#include <iostream>

#include "qbInterface.h"

class qbCube : public qbInterface
{
    public:

        //------------------------------ qbCube
        // Costructor of qbCube class
        qbCube(const int id = 1);

        //------------------------------ qbCube
        // External costructor of qbCube class
        qbCube(comm_settings*, const int id = 1);

        //------------------------------ ~qbCube
        // Distructor of qbCube class
        ~qbCube();

        // ------------------------------
        //      Other Functions
        // ------------------------------

        //------------------------------ setPosAndPreset
        // Set position and stiffness preset of the cube, default input is set to radiants
        void setPosAndPreset(float, float, angular_unit unit = RAD);

        //------------------------------ getPosAndPreset
        // Get position and stiffness preset of the cube, default input is set to radiants
        bool getPosAndPreset(float*, float*, angular_unit unit = RAD);

        //------------------------------ setPosAndPresetPerc
        // Set position and stiffness percentage of the cube, default input is set to radiants
        void setPosAndPresetPerc(float, float, angular_unit unit = RAD);

        //------------------------------ setPosAndPresetPerc
        // Get position and stiffness percentage of the cube, default input is set to radiants
        bool getPosAndPresetPerc(float*, float*, angular_unit unit = RAD);

        //------------------------------ getPos
        // Get position angle in the chosen unit of measure
        bool getPos(float*, angular_unit unit = RAD);

        //------------------------------ getPreset
        // Get preset of the cube
        bool getPreset(float*, angular_unit = RAD);

        //------------------------------ getPresetPerc
        // Get preset of the cube
        bool getPresetPerc(float*);

        //------------------------------ getPosAndCurr
        // Get P1/P2/PL measurements and currents of the cube
        bool getPosAndCurr(float*, float*, angular_unit = RAD);

        //------------------------------ getPPAndCurr
        // Get Eq. position/Preset and currents of the cube
        bool getPPAndCurr(float*, float*, float*, angular_unit = RAD);


    protected:

    private:

};

#endif // QBCUBE_H