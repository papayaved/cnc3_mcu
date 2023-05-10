#ifndef INC_AUX_FUNC_H_
#define INC_AUX_FUNC_H_

#include "my_types.h"

//#define MIN(A, B) ((A) < (B) ? (A) : (B))

void printBytes(const uint8_t buf[], int N, int start_index, int len);

uint8_t read_u8(const uint8_t buf[], const size_t N, size_t* const pos);
uint8_t read_u8_ring(const uint8_t buf[], const size_t N, size_t* const pos);

uint16_t read_u16(const uint8_t buf[], size_t N, size_t pos);
uint32_t read_u32(const uint8_t buf[], size_t N, size_t pos);
uint32_t read_u32_ring(const uint8_t buf[], const size_t N, size_t* const pos);
uint32_t read_u32_rev(const uint8_t buf[], size_t N, size_t pos);
uint32_t read_u32_rev_ring(const uint8_t buf[], const size_t N, size_t* const pos);

int write_u16(uint8_t buf[], int N, size_t pos, uint16_t data);
int write_u32(uint8_t buf[], int N, size_t pos, uint32_t data);
int write_u32_rev(uint8_t buf[], int N, int pos, uint32_t data);

uint32_t crc32(const uint8_t buf[], size_t len);
uint32_t crc32_ring(const uint8_t buf[], const size_t N, const size_t start, size_t len);

uint8_t double_to_uint8(double value);
uint16_t double_to_uint16(double value);
uint32_t double_to_uint32(double value);

void point_swap(point_t* const a, point_t* const b);
void fpoint_swap(fpoint_t* const a, fpoint_t* const b);
void iswap(int32_t* const a, int32_t* const b);
void fswap(double* const a, double* const b);

BOOL point_cmp(const point_t* const A, const point_t* const B);
BOOL fpoint_cmp(const fpoint_t* const A, const fpoint_t* const B, const scale_t* const scale);
BOOL ifpoint_cmp(const point_t* const iA, const fpoint_t* const fB, const scale_t* const scale);

double length(int Ax, int Ay, int Bx, int By);
double ilength(int32_t nx, int32_t ny);
double length_fpoint(const fpoint_t* const A, const fpoint_t* const B);

BOOL sign(int value);
BOOL fsign(double value);

double rad2degree(double value);

typedef struct {
	unsigned char sign;
	int value;
	int rem;
} decimal_t;

void decimal_print(const decimal_t* const value);
void double_print(double x);
void mm_print(double x);

//void fpoint_print(const fpoint_t* const pt);

decimal_t float2fix(double value);
fpoint_t polar_cart(double R, double rad);

double sq(double x);

fpoint_t circle_3pt_n(const fpoint_t* restrict A, const fpoint_t* restrict B, const fpoint_t* restrict C, double* const D);

#endif /* INC_AUX_FUNC_H_ */
