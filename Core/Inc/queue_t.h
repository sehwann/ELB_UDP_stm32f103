#ifndef __QUEUE_T_H__
#define __QUEUE_T_H__


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "node.h"


typedef struct _Queue_t
{
	int32_t		qsize;
	Node*		head;
	Node*		tail;
	int32_t		max_queue;
}Queue_t;

void queue_new		(Queue_t* self, int32_t _max_queue);
void queue_enqueue	(Queue_t* self, Node* new_node);
Node queue_dequeue	(Queue_t* self);

#endif
