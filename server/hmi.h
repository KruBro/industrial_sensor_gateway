#ifndef HMI_H
#define HMI_H

#include "db.h"
#include "poller.h"


int server_init(void);
int server_handle_request(int client_fd, ProcessDB *db, struct pending_request *queue);
void hmi_reset_client_buffer(void);
void hmi_shutdown(int server_fd, int client_fd);

#endif