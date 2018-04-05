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
/// \file lcdInterface.h Header for lcdInterface.c

#ifndef UILCD_H
#define UILCD_H

void vLCDInterface( void *pvParameters );
void LCD_info_changed(void);
void displayerror( void );
void home(void);

#endif
