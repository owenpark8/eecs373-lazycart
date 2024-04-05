#ifndef avgqueue_h
#define avgqueue_h

#include <stdio.h>
#include <stdlib.h>

typedef struct Node {
	struct Node *next;
	uint32_t data;
} Node;

typedef struct AvgQueue {
	Node *front;
	Node *last;
	uint32_t sum;
	uint8_t size;
} AvgQueue;

void init(AvgQueue *q) {
	q->front = NULL;
	q->last = NULL;
	q->sum = 0;
	q->size = 0;
}

int front(AvgQueue *q) {
	if (q->front == NULL) {
		return 0;
	}
	return q->front->data;
}

void pop(AvgQueue *q) {
	if (q->size == 0) return;

	q->size--;
	q->sum -= q->front->data;

	Node *tmp = q->front;
	q->front = q->front->next;
	free(tmp);
}

void push(AvgQueue *q, uint32_t data) {
	q->size++;
	q->sum += data;

	if (q->front == NULL) {
		q->front = (Node *) malloc(sizeof(Node));
		q->front->data = data;
		q->front->next = NULL;
		q->last = q->front;
	} else {
		q->last->next = (Node *) malloc(sizeof(Node));
		q->last->next->data = data;
		q->last->next->next = NULL;
		q->last = q->last->next;
	}
}

uint32_t get_average(AvgQueue *q) {
	return q->sum / q->size;
}

#endif
