#define _CRT_SECURE_NO_WARNINGS
#include "node.h"


Node* node_new(uint8_t* str, int32_t len)
{
	Node* new_node = (Node*)malloc(sizeof(Node));

	new_node->len = len;
	strcpy(new_node->str, str);
	new_node->next = NULL;

	return new_node;
}

