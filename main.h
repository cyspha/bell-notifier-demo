#include <stdlib.h>

typedef struct command_queue_node {
    char *command;
    struct command_queue_node *next_command_node;
} command_queue_node;


typedef struct command_queue {
    command_queue_node *head;
    command_queue_node *tail;
} command_queue;


void add_command_to_queue(char *cmd, command_queue *queue) {
    command_queue_node *node = malloc(sizeof(command_queue_node));
    
    node->command = cmd;
    node->next_command_node = NULL;
    
    if (queue->tail != NULL) {
        queue->tail->next_command_node = node;
    }
    queue->tail = node;
    if (queue->head == NULL) queue->head = queue->tail;
}


char* get_next_command_from_queue(command_queue *queue) {
    command_queue_node *node = queue->head;
    if (node == NULL) return NULL;

    char *cmd = node->command;

    queue->head = node->next_command_node;
    if (node == queue->tail) queue->tail = NULL;
    free(node);
    return cmd;
}