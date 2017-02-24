/*
 * type.h
 *
 *  Created on: Nov 19, 2016
 *      Author: jalex
 */

#ifndef _TYPE_H_
#define _TYPE_H_


typedef struct Detection
{
	unsigned char Segment;
	unsigned int Distance;
	unsigned int Amplitude;



}Detection;

// These definitions assumes 8-bit chars, 16-bit shorts and 32-bit ints.
// If your platform uses different sizes you will have to modify them.
typedef unsigned char  LtBool;
typedef unsigned char  LtByte;
typedef int            LtResult;
typedef unsigned short LtU16;
typedef short          Lt16;
typedef unsigned int   LtU32;

typedef void* LtHandle;

#define LT_INVALID_HANDLE     ((void *)-1)
#define LT_MAX_PORT_NAME_LEN  48

unsigned char NbDet;
unsigned long TimeStamp;
float Temperature;
Detection Detections[3];

#endif /* TYPE_H_ */
