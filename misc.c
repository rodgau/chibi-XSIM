
#include "misc.h"
#include "string.h"


/* itoa:  convert n to characters in s
 *				returns pointer to s
 */
 char* itoa(int n, char s[])
 {
     int i, sign;
		  
     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
		 
		 return s;
 }


			 
/* reverse string in place 
 */
void reverse(char *s) {
char *j;
int c;
 
  j = s + strlen(s) - 1;
  while(s < j) {
    c = *s;
    *s++ = *j;
    *j-- = c;
  }
}
