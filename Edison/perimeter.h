/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz

Private-use only! (you need to ask for a commercial-use)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Private-use only! (you need to ask for a commercial-use)
*/


#ifndef PERIMETER_H
#define PERIMETER_H
// EmpfÃ¤ngt die Perimeterdaten Ã¼ber die SerDueMot read Schnittstelle vom due.

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//#include "BufferedSerial.h"
#include "Thread.h"
#include "helpers.h"

enum EPerSignal {
    SIGNAL_INSIDE = -1, // optional, -1 is the initial state of the fsm
    SIGNAL_NA = 0,
    SIGNAL_OUTSIDE = 1
};

enum EPerReceiveState {
    SPR_OFF = -1, // optional, -1 is the initial state of the fsm
    SPR_HEADER1,
    SPR_HEADER2,
    SPR_MAGNETUDEL,SPR_MAGNETUDEL1,
    SPR_MAGNETUDER,SPR_MAGNETUDER1,
    SPR_MAGNETUDEB,SPR_MAGNETUDEB1,
    SPR_CHECKSUM
};

union uFloat {
    uint8_t uBytes[4];
    float sFloat;
} ;
union uInt16 {
    uint8_t uBytes[2];
    int16_t sIn16t;
} ;
union uInt8 {
    uint8_t uBytes[1];
    int8_t sInt8;
};

union uBool {
    uint8_t uBytes[1];
    bool uBool;
};

class TPerimeterThread: public Thread, public FSM<EPerReceiveState>
{
private:


    float arcToPerimeter;
    

    virtual void UpdateState( EPerReceiveState t );

    // Variable  used by CaluculateInsideOutside
    uint8_t header1, header2;
    uint8_t checksumm;
    uint8_t checksummcalculated;
    // Werte hier negative fÃ¼r inside
    uInt16 _magnetudeL;
    uInt16 _magnetudeR;
    uInt16 _magnetudeB;
    // Werte hier positiv fÃ¼r inside
    void CaluculateInsideOutside( int16_t magl, int16_t magr,  int16_t magb);
    void CaluculateArcToPerimeter();


public:

    //-----------------------------------------------------
    // Empfangsdaten von Perimetersensoren Links und Rechts
    //-----------------------------------------------------

    //  Achtung, vor dem Verwenden  der Daten erstmal prÃ¼fen ob neueDatenEmpfangen == true ist. Wenn Daten verarbeitet wurden, neueDatenEmpfangen=false setzen.
    // Funktioniert nicht, wenn mehrer Prozesse gleichzeitig auf die Daten zugreifen
    //  checksumError gibt an, ob die empfangenen Daten richtig sind oder fehlerhaft. Die Daten werden in die Variablen eingetragen, neueDatenEmpfangen wird aber auf false gesetzt.
    //  seriellesSignalHeaderError gibt an, dass Daten auf der Seriellen Leitung emfangen wurden, aber der Header nicht gefunden wurde. Variablen Daten werden nicht verÃ¤ndert.


    // Achtung, Werte sind positive fÃ¼r inside!!!
    int magnetudeL;  // nimmt nur beim start 0 an. danach wird immer der letzte wert gelatched, wenn signal verloren
    int magnetudeR;
    int magnetudeB;
    unsigned long perimeterLastTransitionTimeB;

    int16_t signalCounterL;    // 2 outside >=  wert  >=-2 inside
    int16_t signalCounterR;
    int16_t signalCounterB;


    unsigned long lastTimeSignalReceivedL;
    unsigned long lastTimeSignalReceivedR;
    unsigned long lastTimeSignalReceivedB;
    
    int count;


    long magMax;
    long magMaxB;
    
    // Achtung, Linefollowing arbeitet mit positiven werten!!!
    //long magLineFollow;
    //long magLineFollowMin;
    //long magLineFollowMax;
       
    bool showValuesOnConsole;

    bool isNearPerimeter();

    bool isLeftInside();
    bool isRightInside();
    bool isBackInside();
    bool isLeftOutside();
    bool isRightOutside();
    bool isBackOutside();


    unsigned int checksumError;  // 0 wenn Sensordaten von Perimeter richtig empfangen wurden. Ansonsten wir fÃ¼r jedes mal eines falsch empfangen Datenpakets der wert erhÃ¶ht. Wenn checksum wieder ok wird Wert auf 0 gesetzt.

    void setup();
    virtual void run(void);
};

#endif

