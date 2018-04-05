#ifndef TERM_H
#define TERM_H
/// \file terminal.h terminal.c header file
#define BANNER_N 9
#include "sensors.h"
#include "Drivers\eeprom.h"

///terminal command structure
typedef struct {
	///text of command (what the user types in)
    const char* command;
    ///function to be called
    void (*function)( char * args );
    ///help string
    const char* help;
} t_command;

extern short lasterror1 ;
extern short lasterror2 ;
extern char errorLog1[4][30];
extern char errorLog2[4][30];
//define data type
typedef char bool;
typedef	char BOOL;

void cmd_watch12(char *);
void cmd_watch1(char *);
void cmd_watch2(char *);

void cmd_settemp1( char * );
void cmd_settemp2( char * );

void cmd_gettemp1( void );
void cmd_gettemp2( void );


void checkOverrun( void );
void uiusart_isr( void );
void writeUSART( char c );
void debugpgm( const char*);
void debug( char* rs );
void cmd_debug( char * );
void cmd_vout( char * );
void cmd_setmaxvout( char * );
void cmd_setminvout( char * );
void cmd_forcesen1(char *);
void cmd_forcesen2(char *);
void cmd_scan(char *);
void cmd_safety1(char *);
void cmd_safety2(char *);
void cmd_setsafetyval1(char *);
void cmd_setsafetyval2(char *);
void cmd_setsafetyband1(char *);
void cmd_setsafetyband2(char *);
void cmd_group1info(char *);
void cmd_group2info(char *);
void cmd_setph1( char * );
void cmd_setph2( char * );
void cmd_setthresh1( char * );
void cmd_setthresh2( char * );
void cmd_setmode1( char * );
void cmd_setmode2( char * );
void cmd_isstable1( char * );
void cmd_isstable2( char * );
void cmd_tinfo1( char * );
void cmd_tinfo2( char * );
void cmd_status1( char * );
void cmd_status2( char * );
void cmd_help( char * );
void cmd_chkerr1( char * );
void cmd_chkerr2( char * );
void cmd_chkerr12( char * );
void cmd_clrerr1(char *arg);
void cmd_clrerr2(char *arg);
void cmd_debug( char * );
void cmd_getver(char *);
void cmd_dispip( char *arg );
void cmd_monitorpid1(char *arg);
void cmd_monitorpid2(char *arg);
//void cmd_setheatpid(char *arg);
void vTerminalTask(void *pvParameters);
void cmd_flowmeter1opt(char *arg);
void cmd_flowmeter2opt(char *arg);
void cmd_pid1setopt(char *arg);
void cmd_pid2setopt(char *arg);
void cmd_reset(char *arg);
void cmd_endian(char *arg);
void cmd_logchan1(char *arg);
void cmd_logchan2(char *arg);
void cmd_loadsetting1(char *arg);
void cmd_loadsetting2(char *arg);
void cmd_setslope1(char *arg);
void cmd_setslope2(char *arg);
void setslope(group *g,float f );
void cmd_setoffset1(char *arg);
void cmd_setoffset2(char *arg);
void setoffset(group *g,float f );
void cmd_geterrlog1(void);
void cmd_geterrlog2(void);
void geterrorlog(unsigned short i);
//void seterrorlog(unsigned short i,char *s);
void seterrorlog(short i, char *r);
void cmd_input(void);
void cmd_getslope1();
void cmd_getslope2();
void formatfloatstr(float f, char *s );
void cmd_dstore(char *arg  );
void cmd_dfetch( char *arg );
void cmd_version( void );
void cmd_thermaltype( char *arg );
#endif

