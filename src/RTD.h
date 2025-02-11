#ifndef NHB_AD7124_RTD
#define NHB_AD7124_RTD

/*
  This file is part of the NHB_AD7124 library.

  Class for dealing with RTD (PT100/PT1000) calculations.

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


#include <math.h>

enum RtdTypes{
    AUTO_DETECT = 0,
    PT100 = 100, 
    PT1000 = 1000,    
};

class RTD{
    public:

        float adcRawToResistance(float adc, float gain, float rRef);

        float resistanceToTemperature(float resistance, RtdTypes type = AUTO_DETECT);

    private:
        const long zero = 1L << 23;
        const long fullScale = 1L << 24;
};


 #endif //NHB_AD7124_RTD