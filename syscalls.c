/****************************************************************************
*  Copyright (c) 2009 by Michael Fischer. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************
*  History:
*
*  28.03.09  mifi   First Version, based on the original syscall.c from
*                   newlib version 1.17.0
****************************************************************************/

/// \file syscalls.c system calls, includes printf/readline functionality for serial
/** \file
	Has a number of system calls that are implemented for the lower level system

	Includes printf/readline functionality for the serial interfaces
*/

#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdarg.h>
#include <stdio.h>

#include "portmacro.h"

#include "Drivers/serial.h"
#include "FreeRTOS.h"
#include "task.h"

int _read_r (struct _reent *r, int file, char * ptr, int len);
int _lseek_r (struct _reent *r, int file, int ptr, int dir);
int _write_r (struct _reent *r, int file, char * ptr, int len);
int _close_r (struct _reent *r, int file);
caddr_t _sbrk_r (struct _reent *r, int incr);
int _fstat_r (struct _reent *r, int file, struct stat * st);
int _isatty_r(struct _reent *r, int fd);
int isatty(struct _reent *r, int fd);

int em_printf(const char *fmt, ...);

int em_printf(const char *fmt, ...) {
    int r,i,j;
    char buffer[128];
    va_list argp;
    va_start(argp, fmt);
    r = vsprintf(buffer, fmt, argp);
    va_end(argp);

    //foreach serial port
    for (j = 0; j < NUMBER_SERIAL_PORTS; j++) {
        //wait for buffer to clear, or for disconnect (then we don't care)
        while ((serial_ports[j].outWaiting() + r > serial_ports[j].buffer_length)
                &&((serial_ports[j].connected == 0)||(serial_ports[j].connected())))
            vTaskDelay( portTICK_RATE_MS * 10 );
        //send out the buffer
        for (i = 0; i < r; i++) {
            serial_ports[j].send(buffer[i]);
        }
    }

    return r;

}

int em_readline(char *line) {
    int j;
    char buffers[3][64];
    char in;
    unsigned int pos[3] = {0,0,0};
    while (1) {
        for (j = 0; j < NUMBER_SERIAL_PORTS; j++) {
            while (serial_ports[j].inWaiting()) {
            	in = serial_ports[j].recv();
            	if (in == 0x08) {//backspace
            		if (pos[j] > 0) {
						pos[j]--; //decrement our position
						//transmit a backspace, space and another backspace (clear the char)
						serial_ports[j].send(0x08);
						serial_ports[j].send(' ');
						serial_ports[j].send(0x08);
					}
					continue;
            	}
            	else 
	                buffers[j][pos[j]] = in;
                if ((buffers[j][pos[j]] == '\r')||(buffers[j][pos[j]] == '\n')) {
                    buffers[j][pos[j]] = 0;
                    strcpy(line, buffers[j]);
                    return pos[j];
                }
                pos[j]++;
                if (pos[j] > 64)
                    return 0;
            }
        }
        vTaskDelay( portTICK_RATE_MS * 50 );
    }
}

/***************************************************************************/

int _read_r (struct _reent *r, int file, char * ptr, int len) {
    r = r;
    file = file;
    ptr = ptr;
    len = len;

    errno = EINVAL;
    return -1;
}

/***************************************************************************/

int _lseek_r (struct _reent *r, int file, int ptr, int dir) {
    r = r;
    file = file;
    ptr = ptr;
    dir = dir;

    return 0;
}

/***************************************************************************/

int _write_r (struct _reent *r, int file, char * ptr, int len) {
    r = r;
    file = file;
    ptr = ptr;

#if 0
    int index;

    /* For example, output string by UART */
    for(index=0; index<len; index++) {
        if (ptr[index] == '\n') {
            uart_putc('\r');
        }

        uart_putc(ptr[index]);
    }
#endif

    return len;
}

/***************************************************************************/

int _close_r (struct _reent *r, int file) {
    return 0;
}

/***************************************************************************/

/* Register name faking - works in collusion with the linker.  */
register char * stack_ptr asm ("sp");

caddr_t _sbrk_r (struct _reent *r, int incr) {
    extern char   end asm ("end"); /* Defined by the linker.  */
    static char * heap_end;
    char *        prev_heap_end;

    if (heap_end == NULL)
        heap_end = & end;

    prev_heap_end = heap_end;

    if (heap_end + incr > stack_ptr) {
        /* Some of the libstdc++-v3 tests rely upon detecting
          out of memory errors, so do not abort here.  */
#if 0
        extern void abort (void);

        _write (1, "_sbrk: Heap and stack collision\n", 32);

        abort ();
#else
        errno = ENOMEM;
        return (caddr_t) -1;
#endif
    }

    heap_end += incr;

    return (caddr_t) prev_heap_end;
}

/***************************************************************************/

int _fstat_r (struct _reent *r, int file, struct stat * st) {
    r = r;
    file = file;

    memset (st, 0, sizeof (* st));
    st->st_mode = S_IFCHR;
    return 0;
}

/***************************************************************************/

int _isatty_r(struct _reent *r, int fd) {
    r = r;
    fd = fd;

    return 1;
}

int isatty(struct _reent *r, int fd) {
    r = r;
    fd = fd;

    return 1;
}

/*** EOF ***/


