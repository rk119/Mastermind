#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

#define SEQL 3

int concat(int a, int b);

int matches(int *seq1, int *seq2)
{
    int correct = 0, approx = 0;
    int *check = (int *)malloc(SEQL * sizeof(int));

    asm(
        "\tMOV R0, #0 \n"      // correct
        "\tMOV R1, %[seq1]\n"  // A
        "\tMOV R2, %[seq2]\n"  // B
        "\tMOV R3, %[check]\n" // C
        "\tMOV R4, #0\n"       // counter

        "exactLoop:\n"
        "\tLDR R5, [R1], #4\n" // loading array value and incrementing index by 8 to access values at even positions
        "\tLDR R6, [R2], #4\n"
        "\tCMP R5, R6\n"
        "\tBNE check\n"
        "\tBL exactValue\n"  // loop again if they are not equal

        "increment:\n"
        "\tADD R4, #1\n" // counter + 1
        "\tCMP R4, #3\n" // counter > 3
        "\tBLT exactLoop\n" 
        "\tB exitE\n"

        "check:\n"
        "\tMOV R5, #0\n"
        "\tSTR R5, [R3]\n"
        "\tADD R3, #4\n"
        "\tB increment\n"

        "exactValue:\n"
        "\tADD R0, #1\n"
        "\tMOV R5, #1\n"
        "\tSTR R5, [R3]\n"
        "\tADD R3, #4\n"
        "\tBX LR\n"

        "exitE:\n"
        "\tMOV %[result], R0\n"
        : [result] "=r"(correct)
        : [seq1] "r"(seq1), [seq2] "r"(seq2), [check] "r"(check)
        : "r0", "r1", "r2", "r3", "r4","r5", "r6", "cc");


    if (correct != 3) {

        asm (
            "\tMOV R0, #0 \n"   //appox
            "\tMOV R1,%[seq1]\n" //A
            "\tMOV R2,%[seq2]\n" // B
            "\tMOV R5, #0\n" //counter

            "outerLoop:\n" 
            "\tADD R5, #1\n" // counter + 1
            "\tCMP R5, #4\n" // counter > 4
            "\tBGE exitA\n" 
            "\tLDR R6, [R1], #4\n" // A[i]
            "\tLDR R7, [R2], #4\n" // B[i]
            "\tCMP R6, R7\n" // A[i] == B[i] 
            "\tBEQ outerLoop\n" // repeat
            "\tMOV R3, %[check]\n"	 // C
            "\tMOV R4, %[seq2]\n" // B
            "\tMOV R7, #0\n" // j

            "innerLoop:\n" 
            "\tADD R7, #1\n"
            "\tCMP R7, #4\n"
            "\tBGE outerLoop\n"
            "\tLDR R8, [R3], #4\n" // check[j]
            "\tADD R4, #4\n" // B[j]
            "\tCMP R8, #0\n" 
            "\tBEQ condition2\n"
            "\tB innerLoop\n"

            "condition2:\n" 
            "\tCMP R5, R7\n"
            "\tBNE condition3\n"
            "\tB innerLoop\n"
            
            "condition3:\n" 
            "\tSUB R4, #4\n" 
            "\tLDR R8, [R4]\n"
            "\tADD R4, #4\n"
            "\tCMP R6, R8\n"
            "\tBEQ approxValue\n"
            "\tB innerLoop\n"

            "approxValue:\n" 
            "\tADD R0, #1\n"
            "\tMOV R8, #1\n"
            "\tSUB R3, #4\n"
            "\tSTR R8, [R3]\n"
            "\tADD R3, #4\n"
            "\tB outerLoop\n"

            "exitA:\n"
            "\tMOV %[result], R0\n"
            : [result] "=r"(approx)
            : [seq1] "r"(seq1), [seq2] "r"(seq2), [check] "r"(check)
            : "r0", "r1", "r2", "r3", "r4","r5", "r6", "r7", "r8", "cc");
    }

    free(check);
    int result  = concat(correct, approx);
    return result;
}