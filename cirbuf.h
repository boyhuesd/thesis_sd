/*
 * cirbuf.h
 * Header for implementing Circular buffers
 *
 *  Created on: 13-04-2014
 *      Author: boyhuesd
 */

#ifndef CIRBUF_H_
#define CIRBUF_H_

#include <stdint.h>
#include <stdbool.h>

enum {
  bufSize = 12,
  elementSize = 512
};

typedef enum elementStatus {
  FREE,
  READ,
  WRITE
} elementStatusT;

typedef int8_t elementIndexT;
typedef int16_t bufDataT;

typedef struct bufElement {
  bufDataT data[elementSize];

  elementIndexT index;
  elementStatusT status; // Status of the element
} elementT;

typedef struct {
  elementT item[bufSize]; // Sadly it's fixed size.

  elementIndexT front;
  elementIndexT rear;

  int8_t count;
} bufT;



void bufInit(volatile bufT * buf);
bool bufIsFull(volatile bufT * buf);
bool bufIsEmpty(volatile bufT * buf);
uint8_t bufIS(volatile elementIndexT index);
void bufItemSetRead(volatile bufT * buf, elementIndexT itemIndex);
void bufItemSetWrite(volatile bufT * buf, elementIndexT itemIndex);
void bufItemSetFree(volatile bufT *buf, elementIndexT itemIndex);
elementStatusT bufItemIsBusy(volatile bufT * buf, elementIndexT itemIndex);
elementStatusT bufItemIsBusy(volatile bufT * buf, elementIndexT itemIndex);

elementT * bufGet(volatile bufT * buf);
elementT * bufGetFree(volatile bufT * buf);

#endif /* CIRBUF_H_ */
