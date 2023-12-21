#ifndef __NODE_H__
#define __NODE_H__


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct _Node
{
	int32_t			len;
	uint8_t			str[20];
	struct _Node*	next;
}Node;

Node* new_node(uint8_t* str, int32_t len);

#endif

