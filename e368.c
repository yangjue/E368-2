#include "e368.h"

short ntohs(short s )
{
	union {
		short s;
		char  c[2];
	}u;
	char t = 0;
	u.s = s ;
	t = u.c[0];
	u.c[0] = u.c[1];
	u.c[1] = t;
	return u.s;
}
int ntohl(int i) 
{
	union {
		int i;
		char  c[4];
	}u;
	char t = 0;
	u.i = i;
	t = u.c[0];
	u.c[3] = t;
	u.c[0] = u.c[3];
	t = u.c[1];
	u.c[1] = u.c[2];
	u.c[2] = t;
	return u.i;
}
