//------------------------------------------------------------------------------ 
//  _NVRM_COPYRIGHT_BEGIN_
//  
//   Copyright 1993-2008 by NVIDIA Corporation.  All rights reserved.  All
//   information contained herein is proprietary and confidential to NVIDIA
//   Corporation.  Any use, reproduction, or disclosure without the written
//   permission of NVIDIA Corporation is prohibited.
//  
//   _NVRM_COPYRIGHT_END_
//------------------------------------------------------------------------------
/// \file itoa.c contains the itoa function

#include <string.h>

///reversees a string in place
static void reverse (char *);

//------------------------------------------------------------------------------
/// itoa: convert n to characters in s
///
//------------------------------------------------------------------------------
char* itoa(int n, char *s)
{
  int sign;
  char *t = s;
  
  if ((sign = n) < 0)
    n = -n;
  
  do
  {
    *s++ = n % 10 + '0';
  } 
  while ((n /= 10) >0);
  
  if (sign < 0)
    *s++ = '-';
  
  *s = '\0';
  
  reverse(t);
  
  return t;
}


//------------------------------------------------------------------------------
/// reverse: reverse sting s in place
///
//------------------------------------------------------------------------------
void reverse(char *s)
{
  int c;
  char *t;
  
  for (t = s + (strlen(s) - 1); s < t; s++, t-- )
  {
    c = *s;
    *s = *t;
    *t = c;
  }
}

