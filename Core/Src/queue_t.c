#define _CRT_SECURE_NO_WARNINGS
#include "queue_t.h"


void queue_new(Queue_t* self, int32_t _max_queue)
{
	memset(self, 0, sizeof(Queue_t));
	self->max_queue = _max_queue;
	self->head = NULL;
}


void queue_enqueue(Queue_t* self, Node* new_node)
{
	if (new_node == NULL)
		return;

	if (self->qsize >= self->max_queue)
		return;

	if (self->head == NULL) {
		self->head = new_node;
	}
	else {
		if (self->head == self->tail) {
			self->head->next = new_node;
		}
		else {
			self->tail->next = new_node;
		}
	}
	self->tail = new_node;
	self->qsize++;
}


Node queue_dequeue(Queue_t* self)
{
	if (self->qsize == 0)
		return;

	Node node;
	memcpy(&node, self->head, sizeof(Node));

	free(self->head);

	self->qsize--;
	self->head = node.next;

	return node;
}

