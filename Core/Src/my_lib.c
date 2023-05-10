#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <limits.h>

#include "my_lib.h"

// function skip spaces
char parse_char(const char* restrict string, char** restrict tailptr) {
	for (*tailptr = (char*)string; *string != '\0'; string++) {
		if (isalpha(*string) || string[0] == '%') {
			*tailptr = (char*)(string + 1);
			return *string;
		}
		else if (*string == ' ' || *string == '\t')
			continue;
		else
			return '\0';
	}

	*tailptr = (char*)(string + 1);
	return '\0';
}

void print_string(const char* const str) {
	str != NULL ? printf("%s\n", str) : printf("NULL\n");
}

// return ABC... or '\0'
char read_char(const char** p_str, BOOL* const OK) {
	char* new_pos;
	char res = parse_char(*p_str, &new_pos);

	if (new_pos != *p_str) {
		*p_str = new_pos;
		if (OK) *OK = TRUE;
	}
	else
		if (OK) *OK = FALSE;

	return res;
}

char try_char(const char* p_str, BOOL* const OK) {
	char* new_pos;
	char res = parse_char(p_str, &new_pos);

	if (OK)
		*OK = (new_pos != p_str) ? TRUE : FALSE;

	return res;
}

unsigned int read_uint(const char** p_str, BOOL* const OK) {
	char* new_pos;
	unsigned int res = (unsigned int)strtoul(*p_str, &new_pos, 10);

	if (new_pos != *p_str) {
		*p_str = new_pos;
		if (OK) *OK = TRUE;
	}
	else
		if (OK) *OK = FALSE;

	return res;
}

double read_double(const char** p_str, BOOL* const OK) {
	char* new_pos;
	double res = strtod(*p_str, &new_pos);

	if (new_pos != *p_str) {
		*p_str = new_pos;
		if (OK) *OK = TRUE;
	}
	else
		if (OK) *OK = FALSE;

	return res;
}

//int parse_uint32(char* str, int n, uint32_t* value, char** endptr) {
//	const int DIGITS = 10;
//	register int i = 0;
//
//	if (n > 0) {
//		*value = 0;
//
//		while (i < DIGITS && i < n)
//			if (isdigit(*str)) {
//				unsigned int x = *str++ - '0';
//				i++;
//
//				if (*value <= UINT_MAX/10)
//					*value *= 10;
//				else
//					return 0;
//
//				if (*value <= UINT_MAX - x)
//					*value += x;
//				else
//					return 0;
//			}
//			else
//				break;
//
//		if (i == DIGITS && i < n && isdigit(*str))
//			return 0;
//
//		*endptr = str;
//
//		return 1;
//	}
//	else
//		return 0;
//}

// TEST
void test_lib() {
	const char* str_ar[] = {
			"G00       X10 Y11.1",
			"G01 X1.0",
			"G04 P1000",
			"G92 X0.Y0.0",
			"M2",
			"M30"
	};
	char* str = NULL;
	uint32_t value;

	for (int i = 0; i < sizeof(str_ar) / sizeof(char*); i++) {
		printf("Test parse %s:\n", str_ar[i]);

//		int OK = parse_uint32(str_ar[i] + 1, 10, &value, &str);

		value = strtoul(str_ar[i] + 1, &str, 10);

		if (str > str_ar[i] + 1)
			printf("%d endpos: %d\n", (int)value, str - str_ar[i]);
		else
			printf("Parse error\n");
	}

	printf("Test parse %s:\n", str_ar[2]);

//	int OK = parse_uint32(str_ar[2] + 5, 10, &value, &str);
	value = strtoul(str_ar[2] + 5, &str, 10);

//	if (OK)
	if (str > str_ar[2] + 5)
		printf("%d endpos: %d\n", (int)value, str - str_ar[2]);
	else
		printf("Parse error\n");

	printf("Test parse double %s:\n", str_ar[0]);
	double fvalue;

	fvalue = strtod(str_ar[0] + 9, &str);

	if (str > str_ar[0] + 9)
		printf("%f endpos: %d\n", fvalue, str - str_ar[0]);
	else
		printf("Parse error\n");

	fvalue = strtod(str_ar[0] + 7, &str);

	if (str > str_ar[0] + 7)
		printf("%f endpos: %d\n", fvalue, str - str_ar[0]);
	else
		printf("Parse error\n");

	char ch = parse_char(str_ar[0] + 3, &str);
	printf("Char %c pos: %d\n", ch, str - str_ar[0]);
}
