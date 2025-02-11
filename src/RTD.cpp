/*
  This file is part of the NHB_AD7124 library.

  Class for dealing with thermocouple calculations.

  MIT License

  Copyright (C) 2021  Jaimy Juliano
  Copyright (C) 2025  Aaron Neal

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "RTD.h"


float RTD::adcRawToResistance(float adc, float gain, float rRef){
    return ((adc-zero) * rRef) / (fullScale * gain - adc);
}


float RTD::resistanceToTemperature(float resistance, RtdTypes type = AUTO_DETECT){

    float resPerDegree = 0;
    float resistanceAtZero = 0;
    switch (type)
    {
    case PT100:
        resPerDegree = 0.385;
        resistanceAtZero = 100;
        break;
    case PT1000:
        resPerDegree = 3.85;
        resistanceAtZero = 1000;
        break;
    case AUTO_DETECT:
    default:
        //if resistance is > 400 ohms it's a PT1000 (PT100 @ 390.26 = 850°C)
        resPerDegree = resistance > 400 ? 0.385 : 3.85; 
        resistanceAtZero = resistance > 400 ? 1000 : 100;
        break;
    }

    return (resistance - resistanceAtZero) / resPerDegree;
}

