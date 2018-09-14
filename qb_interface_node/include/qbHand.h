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

#ifndef QBHAND_H
#define QBHAND_H

#include <qbInterface.h>

#define NUM_OF_TRIALS 5


class qbHand : public qbInterface
{
    public:
        //------------------------------ qbHand
        // Costructor of qbHand class
        qbHand(int);

        //------------------------------ qbHand
        // External costructor of qbHand class
        qbHand(comm_settings*, int);

        //------------------------------ ~qbHand
        // Destructor of qbHand class
        ~qbHand();


        // ------------------------------
        //      Other Functions
        // ------------------------------

        //------------------------------ setPosition
        // Set closure of the hand
        bool setPosPerc(float);

        //------------------------------ getAngle
        // Get Measurement in angle of the hand closure
        bool getPosPerc(float*);

        //------------------------------ getAngle
        // Get Position and Current of the hand closure
        bool getPosAndCurr(float*, float*, angular_unit);

        //------------------------------ init
        // Inizialize default values for the cube
        void retrieveParams();

    protected:

        // Position limit for motors
        int POS_LIMIT_M1_[2], POS_LIMIT_M2_[2];

    private:


};

#endif // QBHAND_H