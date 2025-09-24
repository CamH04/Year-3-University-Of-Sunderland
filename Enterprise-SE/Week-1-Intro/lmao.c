#include <stdio.h>

int main() {
    //add 1+1 :)
    int result;
    asm volatile (
        "movl $1, %%eax\n\t"
        "addl $1, %%eax\n\t"
        "movl %%eax, %0"
        : "=m" (result)
        :
        : "eax"
    );
    printf("Hello World\n");
    return 0;
}
