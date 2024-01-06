#include "GainFunction.h"

GainFunction::GainFunction() 
{
    degree = 0; // Default degree is 0
    coefficients = new float[1]; // Allocate memory for one coefficient
    coefficients[0] = 1.0f; // Set the default coefficient to 1
}

GainFunction::~GainFunction() 
{
    delete[] coefficients;
}

void GainFunction::setGainFn(float* input, int order) 
{
    if (coefficients != nullptr) 
    {
        delete[] coefficients; 
    }
    
    degree = order - 1; 
    coefficients = new float[order];

    for (int i = 0; i < order; i++) 
    {
        coefficients[i] = input[i];
    }
}


String GainFunction::getGainFn() 
{
    String functionStr = "";
    bool isFirstTerm = true;

    for (int i = 0; i <= degree; i++) 
    {
        if (coefficients[i] != 0) 
        {
            if (!isFirstTerm) 
            {
                functionStr += " + ";
            }
            functionStr += String(coefficients[i]) + "x^" + String(degree - i);
            isFirstTerm = false;
        }
    }
    return functionStr;
}

int GainFunction::apply_gain_function(int value) 
{
    float result = 0.0;
    for (int i = 0; i <= degree; i++) 
    {
        result += coefficients[i] * pow(value, degree - i);
    }
    return (int)result;
}

