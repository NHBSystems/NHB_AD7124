#ifndef NHB_AD7124_THERMOCOUPLE
#define NHB_AD7124_THERMOCOUPLE

//Class for dealing with thermocouple calculations.
//TODO: Documentation

#include <math.h>

enum TcTypes{
    Type_K = 0, 
    Type_J,
    Type_T,     
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