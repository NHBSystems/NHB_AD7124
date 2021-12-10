#ifndef NHB_AD7124_THERMOCOUPLE
#define NHB_AD7124_THERMOCOUPLE

/*
  This file is part of the NHB_AD7124 library.

  Class for dealing with thermocouple calculations.

  MIT License

  Copyright (C) 2021  Jaimy Juliano

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


#include <math.h>

// Only Type K supported right now
enum TcTypes{
    Type_K = 0, 
    //Type_J, 
    //Type_T,     
};

class Thermocouple{
    public:

        float voltageToTempDegC(float voltage, float junctionTemp, TcTypes type = Type_K);

        float voltageToTempDegC(float voltage, TcTypes type = Type_K);

        float tempToVoltageDegC(float temperature, TcTypes type = Type_K);

    private:
        float power_series(int n, float input, float coef[]);
};


 #endif //NHB_AD7124_THERMOCOUPLE