#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include "aux_func.h"
#include "cnc_func.h"

void printBytes(const uint8_t buf[], int N, int start_index, int len) {
    int i, a;

    for (i = 0, a = start_index; i < len && a < N; i++, a++)
        if ((i & 7) == 7)
            printf("%02x\n", buf[a]);
        else
            printf("%02x ", buf[a]);

    if ((i & 7) != 7)
        printf("\n");
}

uint8_t read_u8(const uint8_t buf[], const size_t N, size_t* const pos) {
	return *pos >= N ? 0 : buf[(*pos)++];
}

uint8_t read_u8_ring(const uint8_t buf[], const size_t N, size_t* const pos) {
	if (*pos >= N) return 0;

	uint8_t res = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	return res;
}

uint16_t read_u16(const uint8_t buf[], size_t N, size_t pos) {
	uint16_t data;

	if (pos + sizeof(data) <= N)
		memcpy(&data, buf + pos, sizeof(uint16_t));
	else
		data = 0;

	return data;
}

uint32_t read_u32(const uint8_t buf[], size_t N, size_t pos) {
	uint32_t data;

	if (pos + sizeof(data) <= N)
		memcpy(&data, buf + pos, sizeof(uint32_t));
	else
		data = 0;

	return data;
}

uint32_t read_u32_ring(const uint8_t buf[], const size_t N, size_t* const pos) {
	uint32_t res;
	uint8_t* ptr = (uint8_t*)&res;

	if (*pos >= N) return 0;

	*ptr++ = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	*ptr++ = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	*ptr++ = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	*ptr++ = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	return res;
}

uint32_t read_u32_rev(const uint8_t buf[], size_t N, size_t pos) {
	uint32_t data;
	uint8_t* const p1 = (uint8_t*)&data;
	const uint8_t* const p2 = buf + pos;

	if (pos + sizeof(data) <= N) {
		p1[0] = p2[3];
		p1[1] = p2[2];
		p1[2] = p2[1];
		p1[3] = p2[0];
	}
	else
		data = 0;

	return data;
}

uint32_t read_u32_rev_ring(const uint8_t buf[], const size_t N, size_t* const pos) {
	uint32_t res;
	uint8_t* ptr = (uint8_t*)&res + 3;

	if (*pos >= N) return 0;

	*ptr-- = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	*ptr-- = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	*ptr-- = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	*ptr = buf[(*pos)++];
	if (*pos >= N) *pos = 0;

	return res;
}

int write_u16(uint8_t buf[], int N, size_t pos, uint16_t data) {
	if (pos + sizeof(data) <= N) {
		memcpy(buf + pos, &data, sizeof(data));
		return 0;
	}
	else
		return -1;
}

int write_u32(uint8_t buf[], int N, size_t pos, uint32_t data) {
	if (pos + sizeof(data) <= N) {
		memcpy(buf + pos, &data, sizeof(data));
		return 0;
	}
	else
		return -1;
}

int write_u32_rev(uint8_t buf[], int N, int pos, uint32_t data) {
	const uint8_t* const p1 = (uint8_t*)&data;
	uint8_t* const p2 = buf + pos;

	if (pos + sizeof(data) <= N) {
		p2[0] = p1[3];
		p2[1] = p1[2];
		p2[2] = p1[1];
		p2[3] = p1[0];

		return 0;
	}
	else
		return -1;
}

uint16_t crc16_ModRTU(const uint8_t buf[], int len) {
	uint16_t crc = 0xFFFF;

	for (int pos = 0; pos < len; pos++) {
		crc ^= (uint16_t)buf[pos];

		for (int i = 8; i != 0; i--) {
			if ((crc & 0x0001) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}

	return crc;
}

uint32_t crc32(const uint8_t buf[], size_t len) {
	uint32_t crc = ~0U;

	for (size_t pos = 0; pos < len; pos++) {
		crc ^= (uint32_t)buf[pos];

		for (int i = 8; i != 0; i--) {
			if ((crc & 1) != 0) {
				crc >>= 1;
				crc ^= 0xEDB88320;
			}
			else
				crc >>= 1;
		}
	}

	return crc;
}

uint32_t crc32_ring(const uint8_t buf[], const size_t N, const size_t start, size_t len) {
	uint32_t crc = ~0U;

	if (len > N) return 0;

	for (size_t k = 0, pos = start; k < len; k++, pos++) {
		if (pos >= N) pos = 0;
		crc ^= (uint32_t)buf[pos];

		for (int i = 8; i != 0; i--) {
			if ((crc & 1) != 0) {
				crc >>= 1;
				crc ^= 0xEDB88320;
			}
			else
				crc >>= 1;
		}
	}

	return crc;
}

uint8_t double_to_uint8(double value) {
	if (value < 0)
		return 0;
	else if (value > UINT8_MAX)
		return UINT8_MAX;
	else
		return (uint8_t)round(value);
}

uint16_t double_to_uint16(double value) {
	if (value < 0)
		return 0;
	else if (value > UINT16_MAX)
		return UINT16_MAX;
	else
		return (uint16_t)round(value);
}

uint32_t double_to_uint32(double value) {
	value = round(value);

	if (value < 0)
		return 0;

	if (value > (uint32_t)UINT32_MAX)
		return (uint32_t)UINT32_MAX;

	return (uint32_t)value;
}

void point_swap(point_t* const a, point_t* const b) {
	iswap(&a->x, &b->x);
	iswap(&a->y, &b->y);
}

void fpoint_swap(fpoint_t* const a, fpoint_t* const b) {
	fswap(&a->x, &b->x);
	fswap(&a->y, &b->y);
}

void iswap(int32_t* const a, int32_t* const b) {
	*a = *a ^ *b;
	*b = *a ^ *b;
	*a = *b ^ *a;
}

void fswap(double* const pa, double* const pb) {
	double c = *pa;
	*pa = *pb;
	*pb = c;
}

BOOL point_cmp(const point_t* const A, const point_t* const B) {
	return A->x == B->x && A->y == B->y;
}

BOOL fpoint_cmp(const fpoint_t* const A, const fpoint_t* const B, const scale_t* const scale) {
	point_t a = fpoint_mm_to_steps(A, scale);
	point_t b = fpoint_mm_to_steps(B, scale);
	return point_cmp(&a, &b);
}

BOOL ifpoint_cmp(const point_t* const iA, const fpoint_t* const fB, const scale_t* const scale) {
	point_t iB = fpoint_mm_to_steps(fB, scale);
	return point_cmp(iA, &iB);
}

double length(int Ax, int Ay, int Bx, int By) {
	double dx = Bx - Ax;
	double dy = By - Ay;
	return sqrt(dx * dx + dy * dy);
}

double ilength(int32_t nx, int32_t ny) {
	int64_t nx_64 = nx;
	int64_t ny_64 = ny;
	return sqrt( (double)( (uint64_t)(nx_64 * nx_64) + (uint64_t)(ny_64 * ny_64) ) );
}

double length_fpoint(const fpoint_t* const A, const fpoint_t* const B) {
	double dx = B->x - A->x;
	double dy = B->y - A->y;
	return sqrt(dx * dx + dy * dy);
}

BOOL sign(int value) { return value < 0; }
BOOL fsign(double value) { return value < 0; }

double rad2degree(double value) { return 180 * value / M_PI; }

decimal_t float2fix(double value) {
	decimal_t res;
	res.sign = value < 0;
	value = fabs(value);
	res.value = (int)floor(value);

//	res.rem = (int)round((value - res.value) * UINT_MAX);
	res.rem = (int)round((value - res.value) * 1000);

	return res;
}

void decimal_print(const decimal_t* const x) {
	if (x)
		printf("%s%d.%03d", x->sign ? "-" : "", x->value, x->rem);
	else
		printf("NULL");
}

void double_print(double x) {
	decimal_t d = float2fix(x);
	decimal_print(&d);
}

// print in um
void mm_print(double x) {
	double_print(x * 1000);
}

//void fpoint_print(const fpoint_t* const pt) {
//	if (pt) {
//		printf("(");
//		decimal_print(pt->x);
//		printf(", ");
//		decimal_print(pt->y);
//		printf(")");
//	}
//	else
//		printf("NULL");
//}

//BOOL angle_cmp2(fpoint_t* const C, point_t* const base, point_t* const pt) {
//	double base_x = (double)base->x - C->x;
//	double base_y = (double)base->y - C->y;
//	double pt_x = (double)pt->x - C->x;
//	double pt_y = (double)pt->y - C->y;
//
//
//}

fpoint_t polar_cart(double R, double rad) {
	fpoint_t pt;
	pt.x = R * cos(rad);
	pt.y = R * sin(rad);
	return pt;
}

double sq(double x) { return x*x; }

/* Find circle center by 3 points
 * A, B, C - points
 * return: center and diameter (D)
 * D < 0 - points error
*/
fpoint_t circle_3pt_n(const fpoint_t* restrict An, const fpoint_t* restrict Bn, const fpoint_t* restrict Cn, double* const Dn) {
	static fpoint_t res = { 0,0 };
	static fpoint_t AB = { 0,0 }, BC = { 0,0 }, CA = {0, 0};
	static double a1 = 0, b1 = 0, a2 = 0, b2 = 0;

	res.x = res.y = 0;

	AB.x = An->x - Bn->x;
	AB.y = An->y - Bn->y;

	BC.x = Bn->x - Cn->x;
	BC.y = Bn->y - Cn->y;

	CA.x = Cn->x - An->x;
	CA.y = Cn->y - An->y;

	if (BC.x > AB.x && CA.x > AB.x) { // AB is the most vertical line
		const fpoint_t* t = Cn;
		Cn = Bn;
		Bn = An;
		An = t;

		AB.x = An->x - Bn->x;
		AB.y = An->y - Bn->y;

		BC.x = Bn->x - Cn->x;
		BC.y = Bn->y - Cn->y;
	}
	else if (AB.x > BC.x && CA.x > BC.x) { // BC is the most vertial line
		const fpoint_t* t = An;
		An = Bn;
		Bn = Cn;
		Cn = t;

		AB.x = An->x - Bn->x;
		AB.y = An->y - Bn->y;

		BC.x = Bn->x - Cn->x;
		BC.y = Bn->y - Cn->y;
	}

	if (fabs(AB.x) >= 1 && fabs(BC.x) >= 1) { // There are not vertical lines
		a1 = (sq(An->x) - sq(Bn->x) + sq(An->y) - sq(Bn->y)) / (2.0 * AB.x);
		b1 = -AB.y / AB.x;

		a2 = (sq(Bn->x) - sq(Cn->x) + sq(Bn->y) - sq(Cn->y)) / (2.0 * BC.x);
		b2 = -BC.y / BC.x;

		if (fabs(b1 - b2) > 0.001) { // it's not one line
			res.y = (a2 - a1) / (b1 - b2);
			res.x = a1 + b1 * res.y;

			if (Dn) *Dn = (length_fpoint(&res, An) + length_fpoint(&res, Bn) + length_fpoint(&res, Cn)) * (2.0 / 3.0);
			return res;
		}
	}

	if (Dn) *Dn = -1;
	return res;
}
