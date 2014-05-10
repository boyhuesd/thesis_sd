/*
 * cirbuf.c
 *
 *  Created on: 24-04-2014
 *      Author: boyhuesd
 */
#include "cirbuf.h"
#include "stdbool.h"

// Init the data buffer
void bufInit(volatile bufT * buf) {
  int8_t i;

  buf->count = 0;
  buf->front = -1;
  buf->rear = 0;

  // Set all element status to FREE
  for (i = 0; i < bufSize; i++) {
    buf->item[i].index = i;
    buf->item[i].status = FREE;
  }
}

bool bufIsFull(volatile bufT * buf) {
  uint8_t i, t = 0;

  for (i = 0; i < bufSize; i++) {
    if (buf->item[i].status != FREE) {
      t++;
    }
  }
  return ((buf->count + t) >= bufSize);
}

bool bufIsEmpty(volatile bufT * buf) {
  if (buf->count == 0) {
    return true;
  }
  else {
    return false;
  }
}

// Buffer Index Helper. Make sure return index is valid
uint8_t bufIS(elementIndexT index) {
  if (index >= bufSize) {
    return 0;
  }

  else if (index < 0) {
    return (bufSize - 1);
  }

  else {
    return index;
  }
}


// Set the buffer element in READing status
void bufItemSetRead(volatile bufT * buf, elementIndexT itemIndex) {
  buf->item[itemIndex].status = READ;
}

// Set the buffer element in WRITING status
void bufItemSetWrite(volatile bufT * buf, elementIndexT itemIndex) {
  buf->item[itemIndex].status = WRITE;
}

// Set the buffer element free for accessing
void bufItemSetFree(volatile bufT *buf, elementIndexT itemIndex) {
  buf->item[itemIndex].status = FREE;
}

// Check if buffer element is currently in use
elementStatusT bufItemIsBusy(volatile bufT * buf, elementIndexT itemIndex) {
  return (buf->item[itemIndex].status);
}

//
// Return a pointer to a free element in the buffer
// If there's no space for new data, return zero (0) pointer
//
elementT * bufGetFree(volatile bufT * buf) {
  elementIndexT i;
  elementT * result;

  if (bufIsFull(buf)) {
    return 0;
  }
  else {
    // Increase element counter
    buf->count++;

    // Element index for return
    buf->front = bufIS(++buf->front);
    i = buf->front;

    // Check if buffer element is avaiable for accessing
    if (bufItemIsBusy(buf, i)) {
      return 0;
    }
    else {
      // Set element in WRITING status
      buf->item[i].status = WRITE;

      // Return the buffer element
      result = (elementT *) &(buf->item[i]);

      return result;
    }
  }
}

// Return a pointer to the front element of the queue (delete an element)
// If the buffer is empty, return a zero pointer
elementT * bufGet(volatile bufT * buf) {
  elementIndexT i;

  if (bufIsEmpty(buf)) {
    return 0;
  }
  else {
    // Decrease element counter
    buf->count--;

    i = bufIS(buf->rear);

    // Check for availability before returning
    if (bufItemIsBusy(buf, i)) {
      return 0;
    }
    else {
      // Set element status in READING
      buf->item[i].status = READ;

      // Increase rear by 1
      buf->rear = bufIS(++buf->rear);

      return (elementT *) &(buf->item[i]);
    }
  }
}


