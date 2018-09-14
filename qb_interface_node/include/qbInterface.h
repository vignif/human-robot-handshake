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

#ifndef QBINTERFACE_H
#define QBINTERFACE_H

#include <string.h>
#include <iostream>
#include <math.h>
#include <unistd.h>

#include "qbmove_communications.h"
#include "definitions.h"

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

enum angular_unit {RAD, DEG, TICK};
enum linear_unit {MM, M};

class qbInterface
{
    public:

        //------------------------------ qbInterface
        // Costructor of qbInterface class
        qbInterface(const int id = 1);


        //------------------------------ qbInterface
        // External costructor of qbInterface class
        qbInterface(comm_settings*, const int id = 1);


        //------------------------------ ~qbInterface
        // Distructor of qbInterface class
        ~qbInterface();


        // ------------------------------
        //      Other Functions
        // ------------------------------

        //------------------------------ open
        // Open serial communication
        bool open(const char*);


        //------------------------------ close
        // Close serial communication
        void close();


        //------------------------------ activate
        // Active the cube
        bool activate();


        //------------------------------ deactivate
        // Active the cube
        bool deactivate();

        //------------------------------ getMeas
        // Get measurement of positions [1, 2, 3] in ticks
        bool getMeas(short int*);

        //------------------------------ setInputs
        // Set board inputs in ticks [1, 2]
        bool setInputs(short int*);

        //------------------------------ getID
        // Getter of variable ID
        int getID();

        //------------------------------ getMeasAndCurr
        // Get measurements and current of the motors
        bool getMeasAndCurr(short int*, short int*);


        //------------------------------ getCurrents
        // Get motor currents
        template <typename T>
        bool getCurrents(T*);

        comm_settings* cube_comm_;

    protected:
        int id_;
        int axis_dir_;

    private:

};

#include "../src/qbInterface.tpp"

#endif // QBINTERFACE_H
