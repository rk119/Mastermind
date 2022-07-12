/*
 * CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
 * This repo: https://gitlab-student.macs.hw.ac.uk/f28hs-2021-22/f28hs-2021-22-staff/f28hs-2021-22-cwk2-sys

 * Compile:
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:
 sudo ./master-mind

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 ***********************************************************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * See:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

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

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define GREEN 13
// GPIO pin for red LED
#define RED 5
// GPIO pin for button
#define BUTTON 19

// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY 200
// in micro-seconds: 3s
#define TIMEOUT 3000000

// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

#ifndef TRUE
#define TRUE (1 == 1)
#define FALSE (1 == 2)
#endif

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN 25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar[8] =
    {
        0b11111,
        0b10001,
        0b10001,
        0b10101,
        0b11111,
        0b10001,
        0b10001,
        0b11111,
};

static unsigned char hawoNewChar[8] =
    {
        0b11111,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b11111,
};

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char *color_names[] = {"red", "green", "blue"};

static int *theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;

/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
    int bits, rows, cols;
    int rsPin, strbPin;
    int dataPins[8];
    int cx, cy;
};

static int lcdControl;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY 0x04
#define LCD_CTRL 0x08
#define LCD_CDSHIFT 0x10
#define LCD_FUNC 0x20
#define LCD_CGRAM 0x40
#define LCD_DGRAM 0x80

// Bits in the entry register

#define LCD_ENTRY_SH 0x01
#define LCD_ENTRY_ID 0x02

// Bits in the control register

#define LCD_BLINK_CTRL 0x01
#define LCD_CURSOR_CTRL 0x02
#define LCD_DISPLAY_CTRL 0x04

// Bits in the function register

#define LCD_FUNC_F 0x04
#define LCD_FUNC_N 0x08
#define LCD_FUNC_DL 0x10

#define LCD_CDSHIFT_RL 0x04

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define PI_GPIO_MASK (0xFFFFFFC0)

static unsigned int gpiobase;
static uint32_t *gpio;

static int timed_out = 0;

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure(int fatal, const char *message, ...)
{
    va_list argp;
    char buffer[1024];

    if (!fatal) 
        return -1;

    va_start(argp, message);
    vsnprintf(buffer, 1023, message, argp);
    va_end(argp);

    fprintf(stderr, "%s", buffer);
    exit(EXIT_FAILURE);

    return 0;
}

/*
 * waitForEnter:
 *********************************************************************************
 */

void waitForEnter(void)
{
    printf("Press ENTER to continue: ");
    (void)fgetc(stdin);
}

/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;


/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
uint64_t timeInMicroseconds()
{
    struct timeval tv;
    uint64_t now;
    gettimeofday(&tv, NULL);
    now = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec; // in us
    // now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ; // in ms

    return (uint64_t)now;
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
void timer_handler(int signum)
{
    static int count = 0;
    stopT = timeInMicroseconds();
    count++;
    fprintf(stderr, "timer expired %d times; (measured interval %f sec)\n", count, (stopT - startT) / 1000000.0);
    startT = timeInMicroseconds();
}

/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout)
{
    /* ***  COMPLETE the code here  ***  */
}

/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicroseconds(unsigned int howLong)
{
    struct timespec sleeper;
    unsigned int uSecs = howLong % 1000000;
    unsigned int wSecs = howLong / 1000000;

    /**/ if (howLong == 0)
        return;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
    else
    {
        sleeper.tv_sec = wSecs;
        sleeper.tv_nsec = (long)(uSecs * 1000L);
        nanosleep(&sleeper, NULL);
    }
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay(unsigned int howLong)
{
    struct timespec sleeper, dummy;

    sleeper.tv_sec = (time_t)(howLong / 1000);
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

    nanosleep(&sleeper, &dummy);
}

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* sends a @value (LOW or HIGH) on pin number @pin; @gpio@ is the mmaped GPIO base address */
void digitalWrite(uint32_t *gpio, int pin, int value)
{
    int off, res;
    off = (value == LOW) ? 10 : 7;

    asm volatile(
        "\tLDR R1, %[gpio]\n"
        "\tADD R0, R1, %[off]\n"
        "\tMOV R2, #1\n"
        "\tMOV R1, %[pin]\n"
        "\tAND R1, #31\n"
        "\tLSL R2, R1\n"
        "\tSTR R2, [R0, #0]\n"
        "\tMOV %[result], R2\n"
        : [result] "=r"(res)
        : [pin] "r"(pin), [gpio] "m"(gpio), [off] "r"(off * 4)
        : "r0", "r1", "r2", "cc");
}

/* sets the @mode of a GPIO @pin to INPUT or OUTPUT; @gpio is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode)
{
    int fSel = pin / 10;
    int shift = (pin % 10) * 3;
    int res;

    // output
    if (mode == OUTPUT)
    {
        asm(/* inline assembler version of setting to ouput" */
            "\tLDR R1, %[gpio]\n"
            "\tADD R0, R1, %[fSel]\n"
            "\tLDR R1, [R0, #0]\n"
            "\tMOV R2, #0b111\n"
            "\tLSL R2, %[shift]\n"
            "\tBIC R1, R1, R2\n"
            "\tMOV R2, #1\n"
            "\tLSL R2, %[shift]\n"
            "\tORR R1, R2\n"
            "\tSTR R1, [R0, #0]\n"
            "\tMOV %[result], R1\n"
            : [result] "=r"(res)
            : [pin] "r"(pin), [gpio] "m"(gpio), [fSel] "r"(fSel * 4), [shift] "r"(shift)
            : "r0", "r1", "r2", "cc");
    }

    // input
    else if (mode == INPUT)
    {
        asm(/* inline assembler version of setting to input" */
            "\tLDR R1, %[gpio]\n"
            "\tADD R0, R1, %[fSel]\n"
            "\tLDR R1, [R0, #0]\n"
            "\tMOV R2, #0b111\n"
            "\tLSL R2, %[shift]\n"
            "\tBIC R1, R1, R2\n"
            "\tSTR R1, [R0, #0]\n"
            "\tMOV %[result], R1\n"
            : [result] "=r"(res)
            : [pin] "r"(pin), [gpio] "m"(gpio), [fSel] "r"(fSel * 4), [shift] "r"(shift)
            : "r0", "r1", "r2", "cc");
    }

    else
    {
        fprintf(stderr, "Invalid mode");
    }
}

/* sends a @value (LOW or HIGH) on pin number @pin; @gpio is the mmaped GPIO base address */
void writeLED(uint32_t *gpio, int led, int value)
{
    int off, res;
    if (value == LOW)
    {
        off = 10;
        asm volatile(
            /*inline assembler version of clearing LED*/
            "\tLDR R1, %[gpio]\n"
            "\tADD R0, R1, %[off]\n"
            "\tMOV R2, #1\n"
            "\tMOV R1, %[pin]\n"
            "\tAND R1, #31\n"
            "\tLSL R2, R1\n"
            "\tSTR R2, [R0,#0]\n"
            "\tMOV %[result], R2\n"
            : [result] "=r"(res)
            : [pin] "r"(led), [gpio] "m"(gpio), [off] "r"(off * 4)
            : "r0", "r1", "r2", "cc");
    }
    else
    {
        off = 7;
        asm volatile(
            /*inline assembler version of setting LED*/
            "\tLDR R1, %[gpio]\n"
            "\tADD R0, R1, %[off]\n"
            "\tMOV R2, #1\n"
            "\tMOV R1, %[pin]\n"
            "\tAND R1, #31\n"
            "\tLSL R2, R1\n"
            "\tSTR R2, [R0,#0]\n"
            "\tMOV %[result], R2\n"
            : [result] "=r"(res)
            : [pin] "r"(led), [gpio] "m"(gpio), [off] "r"(off * 4)
            : "r0", "r1", "r2", "cc");
    }
}

/* reads a @value (LOW or HIGH) from pin number @pin (a button device); @gpio is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button)
{
    int state = 0;
    asm volatile(
        "MOV R1, %[gpio]\n"
        "LDR R2, [R1, #0x34]\n"
        "MOV R3, %[pin]\n"
        "MOV R4, #1\n"
        "LSL R4, R3\n"
        "AND %[state], R2, R4\n"
        : [state] "=r"(state)
        : [pin] "r"(button), [gpio] "r"(gpio)
        : "r0", "r1", "r2", "r3", "r4", "cc");

    return state > 0;
}

/* waits for a button input on pin number @button; @gpio@ is the mmaped GPIO base address */
/* uses readButton() */
void waitForButton(uint32_t *gpio, int button)
{
    for (int j = 0; j < 13; j++)
    {
        if (readButton(gpio, button))
            break;
        delay(DELAY); 
    }
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led, @c times */
void blinkN(uint32_t *gpio, int led, int c)
{
    for (int i = 0; i < c; i++)
    {
        writeLED(gpio, led, HIGH);
        delay(700);
        writeLED(gpio, led, LOW);
        delay(700);
    }
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */

void strobe(const struct lcdDataStruct *lcd)
{
    // timing changes for new version of delayMicroseconds ()
    digitalWrite(gpio, lcd->strbPin, 1);
    delayMicroseconds(50);
    digitalWrite(gpio, lcd->strbPin, 0);
    delayMicroseconds(50);
}

/*
 * sentDataCmd:
 *	Send some data or command byte to the display.
 *********************************************************************************
 */

void sendDataCmd(const struct lcdDataStruct *lcd, unsigned char data)
{
    register unsigned char myData = data;
    unsigned char i, d4;

    if (lcd->bits == 4)
    {
        d4 = (myData >> 4) & 0x0F;
        for (i = 0; i < 4; ++i)
        {
            digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
            d4 >>= 1;
        }
        strobe(lcd);

        d4 = myData & 0x0F;
        for (i = 0; i < 4; ++i)
        {
            digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
            d4 >>= 1;
        }
    }
    else
    {
        for (i = 0; i < 8; ++i)
        {
            digitalWrite(gpio, lcd->dataPins[i], (myData & 1));
            myData >>= 1;
        }
    }
    strobe(lcd);
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */

void lcdPutCommand(const struct lcdDataStruct *lcd, unsigned char command)
{
    // #ifdef DEBUG
    //     fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin, 0, lcd, command);
    // #endif
    digitalWrite(gpio, lcd->rsPin, 0);
    sendDataCmd(lcd, command);
    delay(2);
}

void lcdPut4Command(const struct lcdDataStruct *lcd, unsigned char command)
{
    register unsigned char myCommand = command;
    register unsigned char i;

    digitalWrite(gpio, lcd->rsPin, 0);

    for (i = 0; i < 4; ++i)
    {
        digitalWrite(gpio, lcd->dataPins[i], (myCommand & 1));
        myCommand >>= 1;
    }
    strobe(lcd);
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

void lcdHome(struct lcdDataStruct *lcd)
{
#ifdef DEBUG
    fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
    lcdPutCommand(lcd, LCD_HOME);
    lcd->cx = lcd->cy = 0;
    delay(5);
}

void lcdClear(struct lcdDataStruct *lcd)
{
    // #ifdef DEBUG
    //     fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
    // #endif
    lcdPutCommand(lcd, LCD_CLEAR);
    lcdPutCommand(lcd, LCD_HOME);
    lcd->cx = lcd->cy = 0;
    delay(5);
}

/*
 * lcdPosition:
 *	Updates the position of the cursor on the display.
 *	Ignores invalid locations.
 *********************************************************************************
 */

void lcdPosition(struct lcdDataStruct *lcd, int x, int y)
{

    if ((x > lcd->cols) || (x < 0))
        return;
    if ((y > lcd->rows) || (y < 0))
        return;

    lcdPutCommand(lcd, x + (LCD_DGRAM | (y > 0 ? 0x40 : 0x00) /* rowOff [y] */));

    lcd->cx = x;
    lcd->cy = y;
}

/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */

void lcdDisplay(struct lcdDataStruct *lcd, int state)
{
    if (state)
        lcdControl |= LCD_DISPLAY_CTRL;
    else
        lcdControl &= ~LCD_DISPLAY_CTRL;

    lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursor(struct lcdDataStruct *lcd, int state)
{
    if (state)
        lcdControl |= LCD_CURSOR_CTRL;
    else
        lcdControl &= ~LCD_CURSOR_CTRL;

    lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursorBlink(struct lcdDataStruct *lcd, int state)
{
    if (state)
        lcdControl |= LCD_BLINK_CTRL;
    else
        lcdControl &= ~LCD_BLINK_CTRL;

    lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */

void lcdPutchar(struct lcdDataStruct *lcd, unsigned char data)
{
    digitalWrite(gpio, lcd->rsPin, 1);
    sendDataCmd(lcd, data);

    if (++lcd->cx == lcd->cols)
    {
        lcd->cx = 0;
        if (++lcd->cy == lcd->rows)
            lcd->cy = 0;

        // inline computation of address 
        lcdPutCommand(lcd, lcd->cx + (LCD_DGRAM | (lcd->cy > 0 ? 0x40 : 0x00) /* rowOff [lcd->cy] */));
    }
}

/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */

void lcdPuts(struct lcdDataStruct *lcd, const char *string)
{
    while (*string)
        lcdPutchar(lcd, *string++);
}

/* ======================================================= */
/* SECTION: helper functions                               */
/* ------------------------------------------------------- */
/* Helper functions for help with game logic */

/*  Function to concat two individual digits
    Reference: https://stackoverflow.com/questions/12700497/how-to-concatenate-two-integers-in-c*/
int concat(int x, int y)
{
    int temp = y;
    do
    {
        x *= 10;
        y /= 10;
    } while (y != 0);
    return x + temp;
}

/*  Function to reverse arr[] from start to end*/
void reverse(int arr[], int start, int end)
{
    int temp;
    while (start < end)
    {
        temp = arr[start];
        arr[start] = arr[end];
        arr[end] = temp;
        start++;
        end--;
    }
}

/* Helper function to show user guess on terminal */
void showGuess(int colorNum, struct lcdDataStruct *lcd)
{
    switch (colorNum)
    {
    case 1:
        lcdPuts(lcd, " R");
        fprintf(stderr, " R");
        break;
    case 2:
        lcdPuts(lcd, " G");
        fprintf(stderr, " G");
        break;
    case 3:
        lcdPuts(lcd, " B");
        fprintf(stderr, " B");
        break;
    }
}

void showMatchesLCD(int code, struct lcdDataStruct *lcd)
{
    // Variable as index value
    int index = 0;

    // Temporary array to store encoded values
    int *temp = (int *)malloc(2 * sizeof(int));

    char *text = (char *)malloc(3 * sizeof(char)); // rifrof

    // While loop to split code digits into array
    // If passed argument 'code' is not 0, and the current index is less than the length of secret sequence
    while (code != 0 && index < seqlen)
    {
        temp[index] = code % 10;
        ++index;
        code /= 10;
    }

    // Store digits in respective int values
    int approx = temp[0];
    int correct = temp[1];

    // Print out correct and approximate values to terminal
    // printf("Exact: %d    Approximate: %d\n\n", correct, approx);
    printf("%d exact\n", correct);
    printf("%d approximate\n", approx);

    // Free temp array
    free(temp);

    lcdPosition(lcd, 0, 1);
    lcdPuts(lcd, "Exact: ");
    sprintf(text, "%d", correct);
    lcdPosition(lcd, 6, 1);
    lcdPuts(lcd, text);
    blinkN(gpio, GREEN, correct);
    blinkN(gpio, RED, 1);
    lcdPosition(lcd, 8, 1);
    lcdPuts(lcd, "Approx: ");
    sprintf(text, "%d", approx);
    lcdPosition(lcd, 15, 1);
    lcdPuts(lcd, text);
    blinkN(gpio, GREEN, approx);
}

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */


/* initialise the secret sequence; by default it should be a random sequence */
void initSeq()
{
    // Allocating memory for array
    theSeq = (int *)malloc(seqlen * sizeof(int));

    // Exit program if array is null
    if (theSeq == NULL)
    {
        printf("Array is null i.e., memory not allocated!");
        exit(0);
    }
    // If array is not null
    else
    {
        // Loop through sequence length, and add random values between 1 to 3
        for (int i = 0; i < seqlen; ++i)
            theSeq[i] = rand() % 3 + 1;
    }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq)
{
    printf("Secret : ");
    for (int i = 0; i < seqlen; ++i)
    {
        // printf(" %d ", theSeq[i]);
        switch (theSeq[i])
        {
        case 1:
            printf("R ");
            break;
        case 2:
            printf("G ");
            break;
        case 3:
            printf("B ");
            break;
        }
    }
    printf("\n");
}

#define NAN1 8
#define NAN2 9

// Count matches in ARM Assembly
/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, either both encoded in one value, */
/* or as a pointer to a pair of values */
// int countMatches(int *seq1, int *seq2)
// {
//     int correct = 0, approx = 0;
//     int *check = (int *)malloc(SEQL * sizeof(int));

//     asm(
//         "\tMOV R0, #0 \n"      // correct
//         "\tMOV R1, %[seq1]\n"  // A
//         "\tMOV R2, %[seq2]\n"  // B
//         "\tMOV R3, %[check]\n" // C
//         "\tMOV R4, #0\n"       // counter

//         "exactLoop:\n"
//         "\tLDR R5, [R1], #4\n" // loading array value and incrementing index by 8 to access values at even positions
//         "\tLDR R6, [R2], #4\n"
//         "\tCMP R5, R6\n"
//         "\tBNE check\n"
//         "\tBL exactValue\n"  // loop again if they are not equal

//         "increment:\n"
//         "\tADD R4, #1\n" // counter + 1
//         "\tCMP R4, #3\n" // counter > 3
//         "\tBLT exactLoop\n" 
//         "\tB exitE\n"

//         "check:\n"
//         "\tMOV R5, #0\n"
//         "\tSTR R5, [R3]\n"
//         "\tADD R3, #4\n"
//         "\tB increment\n"

//         "exactValue:\n"
//         "\tADD R0, #1\n"
//         "\tMOV R5, #1\n"
//         "\tSTR R5, [R3]\n"
//         "\tADD R3, #4\n"
//         "\tBX LR\n"

//         "exitE:\n"
//         "\tMOV %[result], R0\n"
//         : [result] "=r"(correct)
//         : [seq1] "r"(seq1), [seq2] "r"(seq2), [check] "r"(check)
//         : "r0", "r1", "r2", "r3", "r4","r5", "r6", "cc");


//     if (correct != 3) {

//         asm (
//             "\tMOV R0, #0 \n"   //appox
//             "\tMOV R1,%[seq1]\n" //A
//             "\tMOV R2,%[seq2]\n" // B
//             "\tMOV R5, #0\n" //counter

//             "outerLoop:\n" 
//             "\tADD R5, #1\n" // counter + 1
//             "\tCMP R5, #4\n" // counter > 4
//             "\tBGE exitA\n" 
//             "\tLDR R6, [R1], #4\n" // A[i]
//             "\tLDR R7, [R2], #4\n" // B[i]
//             "\tCMP R6, R7\n" // A[i] == B[i] 
//             "\tBEQ outerLoop\n" // repeat
//             "\tMOV R3, %[check]\n"	 // C
//             "\tMOV R4, %[seq2]\n" // B
//             "\tMOV R7, #0\n" // j

//             "innerLoop:\n" 
//             "\tADD R7, #1\n"
//             "\tCMP R7, #4\n"
//             "\tBGE innerLoop\n"
//             "\tLDR R8, [R3], #4\n" // check[j]
//             "\tADD R4, #4\n" // B[j]
//             "\tCMP R8, #0\n" 
//             "\tBEQ condition2\n"
//             "\tB innerLoop\n"

//             "condition2:\n" 
//             "\tCMP R5, R7\n"
//             "\tBNE condition3\n"
//             "\tB innerLoop\n"
            
//             "condition3:\n" 
//             "\tSUB R4, #4\n" 
//             "\tLDR R8, [R4]\n"
//             "\tADD R4, #4\n"
//             "\tCMP R6, R8\n"
//             "\tBEQ approxValue\n"
//             "\tB innerLoop\n"

//             "approxValue:\n" 
//             "\tADD R0, #1\n"
//             "\tMOV R8, #1\n"
//             "\tSUB R3, #4\n"
//             "\tSTR R8, [R3]\n"
//             "\tADD R3, #4\n"
//             "\tB outerLoop\n"

//             "exitA:\n"
//             "\tMOV %[result], R0\n"
//             : [result] "=r"(approx)
//             : [seq1] "r"(seq1), [seq2] "r"(seq2), [check] "r"(check)
//             : "r0", "r1", "r2", "r3", "r4","r5", "r6", "r7", "r8", "cc");
//     }

//     free(check);
//     int result  = concat(correct, approx);
//     return result;
// }

// Count matches in C
int /* or int* */ countMatches(int *seq1, int *seq2)
{
    /* ***  COMPLETE the code here  ***  */

    // Loop index variables
    int i, j, k, m;

    // Temporary array for flagging seen colours
    int *check = (int *)malloc(seqlen * sizeof(int));

    // Variables for holding correct and approximate guesses
    int correct = 0, approx = 0;

    // Fill initial check flag array with 0's
    for (i = 0; i < seqlen; i++)
        check[i] = 0;

    // Loop through secret sequence and guess sequence to find out correct positions i.e., correct colours in correct positions
    for (j = 0; j < seqlen; j++)
    {
        // If element in sequence 1 matches element in sequence 2
        if (seq2[j] == seq1[j])
        {
            // Modify flag to 1 i.e., seen
            check[j] = 1;
            // Increment correct entry
            correct++;
        }
    }

    // Loop through secret sequence and guess sequence to find out approximate positions i.e., correct colours in wrong positions
    for (k = 0; k < seqlen; k++)
    {
        // If element in sequence 1 matches element in sequence 2
        if (seq2[k] == seq1[k])
        {
            // Don't do anything, continue further
            continue;
        }
        // If elements don't match
        else
        {
            // Iterate through sequence length
            for (m = 0; m < seqlen; m++)
            {
                // Check whether colour has been flagged and whether elements are equal i.e., correct element wrong position
                if (!check[m] && m != k && seq2[k] == seq1[m])
                {
                    // Increment approximate entry
                    approx++;
                    // Modify flag to 1 i.e., seen
                    check[m] = 1;
                    // Break out of current loop
                    break;
                }
            }
        }
    }

    // Free check array
    free(check);

    // Store concatenated correct and approx values (2 numbers encoded into 1)
    int ret = concat(correct, approx);

    // Return encoded number
    return ret;
}

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int code, int *seq1, int *seq2, int lcd_format)
{
    // Variable as index value
    int index = 0;

    // Temporary array to store encoded values
    int *temp = (int *)malloc(2 * sizeof(int));

    // While loop to split code digits into array
    // If passed argument 'code' is not 0, and the current index is less than the length of secret sequence
    while (code != 0 && index < seqlen)
    {
        temp[index] = code % 10;
        ++index;
        code /= 10;
    }

    // Store digits in respective int values
    int approx = temp[0];
    int correct = temp[1];

    // Print out correct and approximate values to terminal
    printf("%d exact\n", correct);
    printf("%d approximate\n", approx);

    // Free temp array
    free(temp);
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val)
{
    int i = 0;

    // While loop to add integer digit to array passed as argument
    while (val != 0 && i < seqlen)
    {
        seq[i] = val % 10;
        ++i;
        val /= 10;
    }

    // Since added elements to array will be in reverse order, we reverse the array with a helper function
    reverse(seq, 0, seqlen - 1);

    // Print out entered sequence to terminal
    printf("Your entered sequence is : \n");

    for (int i = 0; i < seqlen; i++)
    {
        printf("%d ", seq[i]);
    }
    printf("\n");
}

/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */
int *readNum(int max)
{
    int index = 0;

    // Array to store digits of passed argument
    int *arr = (int *)malloc(seqlen * sizeof(int));
    if (!arr)
        return NULL;

    // Split passed argument into digits and store in array
    while (max != 0 && index < SEQL)
    {
        arr[index] = max % 10;
        ++index;
        max /= 10;
    }

    // Since digits stored in array are in reverse order, reverse array through helper function
    reverse(arr, 0, SEQL - 1);

    // Return passed argument as array of digits
    return arr;
}
