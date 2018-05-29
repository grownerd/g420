
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H


void command_parser(char * command);
void host_cmd_get(char * item);
void host_cmd_set(char * item, char * val);
void host_cmd_del(char * item, char * val);

void set_rtc(char * timestring);
void set_timer(char * args);

void del_timer(char * index);

void print_rtc(void);
void print_timers(void);


#endif

