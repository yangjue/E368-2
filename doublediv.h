#ifndef DOUBLDEDIV_HEADER_H
#define DOUBLDEDIV_HEADER_H
/// \file doublediv.h doublediv.c header

///Struct for doublediv
/**
	Struct to handle doing fixed point division
 */
typedef struct double_int_t {
	//whole part of the number
    int whole;
    //decimal part of the number
    int decimal;
} double_int;

//do a division
/**
	division on the double_int type, if needed. Currently unused (avoided division)
 */
double_int doubleDiv(int i, int div, int digits);

#endif

