#ifndef GainFunction_h
#define GainFunction_h

#include "Arduino.h"

class GainFunction 
{
private:
    float* coefficients; // Pointer to an array holding the coefficients of the gain function
    int degree; // Degree of the polynomial representing the gain function

public:
    GainFunction(); // Constructor, initializes the gain function with default values
    ~GainFunction(); // Destructor, deallocates the dynamic memory used for coefficients

    void setGainFn(float* input, int order); // Sets the gain function with given coefficients and order
    String getGainFn(); // Returns a string representation of the gain function
    int apply_gain_function(int value); // Applies the gain function to a given value and returns the result
};

#endif

