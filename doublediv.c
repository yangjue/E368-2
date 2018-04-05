#include "doublediv.h"
/// \file doublediv.c fixed point division
double_int doubleDiv(int i, int div, int digits) {
    int c;
    //Create value to return
    double_int value;
    int accuracy_coefficient = 1;
    //Make Accuracy Coefficient correct value to display
    //The requested digits
    for(c = 0; c<digits; c++) {
        accuracy_coefficient = accuracy_coefficient*10;
    }
    i = i * accuracy_coefficient;
    value.whole = (i/div) / accuracy_coefficient;
    value.decimal = (i/div) % accuracy_coefficient;

    return value;
}


