#ifndef INC_MY_LIB_H_
#define INC_MY_LIB_H_

#include "my_types.h"

char parse_char(const char* restrict string, char** restrict tailptr);
void print_string(const char* const str);

char read_char(const char** p_str, BOOL* const OK);
char try_char(const char* p_str, BOOL* const OK);
unsigned int read_uint(const char** p_str, BOOL* const OK);
double read_double(const char** p_str, BOOL* const OK);

//int parse_uint32(char* str, int n, uint32_t* value, char** endptr);
void test_lib();

#endif /* INC_MY_LIB_H_ */
