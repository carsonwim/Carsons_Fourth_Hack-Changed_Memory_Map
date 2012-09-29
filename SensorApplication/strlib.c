

#include "strlib.h"
#include "string.h"
#include "carsons_file.h"

//*****************************************************************************
//
//! \brief   Integer to ASCII
//!
//! \param  n is the number to be converted to ASCII
//! \param s is a pointer to an array where the ASCII string will be placed
//! \param b is the base (10 for decimal)
//!
//! \return none
//
//*****************************************************************************
char *itoa(int n, char *s, int b) {
	static char digits[] = "0123456789abcdefghijklmnopqrstuvwxyz";
	int i=0, sign;
    
	if ((sign = n) < 0)
		n = -n;

	do {
		s[i++] = digits[n % b];
	} while ((n /= b) > 0);

	if (sign < 0)
		s[i++] = '-';
	s[i] = '\0';

	return strrev(s);
}

//*****************************************************************************
//
//! \brief   Reverses a string
//!
//! \param  str is a pointer to the string to be reversed
//!
//! \return none
//
//*****************************************************************************
char *strrev(char *str) {
	char *p1, *p2;

	if (!str || !*str)
		return str;

	for (p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2) {
		*p1 ^= *p2;
		*p2 ^= *p1;
		*p1 ^= *p2;
	}

	return str;
}
