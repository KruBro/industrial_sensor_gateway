#ifndef _XOPEN_SOURCE 
#define _XOPEN_SOURCE 700 
#endif
#include <stdio.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include "hmi.h"
#include "db.h"
#include "poller.h"
#include "modbus.h"

typedef  void (*HmiCmdFunc)(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len);

typedef struct {
    const char *cmd_name;
    HmiCmdFunc action;
} HmiCommandMapping;

static void cmd_press(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    (void)queue;
    if(db->device_online ==  0)
    {
        snprintf(reply_buf, buf_len, "OFFLINE\n");
        return;
    }

    uint16_t p = db->registers[0];
    snprintf(reply_buf, buf_len, "PRESSURE %u.%u bar\n", (p / 10), (p % 10));
}

static void cmd_flow(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    (void)queue;
    if(db->device_online ==  0)
    {
        snprintf(reply_buf, buf_len, "OFFLINE\n");
        return;
    }

    uint16_t f = db->registers[1];
    snprintf(reply_buf, buf_len, "FLOW %u.%u L/min\n", (f / 10), (f % 10));
}

static void cmd_temp(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    (void)queue;
    if(db->device_online ==  0)
    {
        snprintf(reply_buf, buf_len, "OFFLINE\n");
        return;
    }

    uint16_t t = db->registers[2];
    snprintf(reply_buf, buf_len, "TEMP %u.%u C\n", (t / 10), (t % 10));
}

static void cmd_valve_query(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    (void)queue;
    if(db->device_online ==  0)
    {
        snprintf(reply_buf, buf_len, "OFFLINE\n");
        return;
    }

    uint16_t valve = db->registers[3];
    snprintf(reply_buf, buf_len, "VALVE %s\n", (valve == 1) ? "OPEN" : "CLOSED");
}

static void cmd_all(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    (void)queue;
    uint16_t p = db->registers[0];
    uint16_t f = db->registers[1];
    uint16_t t = db->registers[2];
    uint16_t valve = db->registers[3];

    int online = db->device_online;

    snprintf(reply_buf, buf_len,
             "{\"p\":%u.%u,\"f\":%u.%u,\"t\":%u.%u,\"valve\":\"%s\",\"online\":%s}\n",
             p / 10, p % 10,
             f / 10, f % 10,
             t / 10, t % 10,
             (valve == 1) ? "OPEN" : "CLOSED",
             online ? "true" : "false");
}

static void cmd_valve_open(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    (void)db;
    queue->has_request = 1;
    queue->addr = 0x01;
    queue->count = 0;
    queue->is_write_coil = 1;
    queue->write_value = 0xFF00;
    snprintf(reply_buf, buf_len, "OK\n");
}

static void cmd_valve_close(ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    (void)db;
    queue->has_request = 1;
    queue->addr = 0x01;
    queue->count = 0;
    queue->is_write_coil = 1;
    queue->write_value = 0x0000;
    snprintf(reply_buf, buf_len, "OK\n");
}

static HmiCommandMapping hmi_cmd_table[] = {
    {"PRESSURE",    cmd_press},
    {"FLOW",        cmd_flow},
    {"TEMP",        cmd_temp},
    {"VALVE",       cmd_valve_query},
    {"ALL",         cmd_all},
    {"VALVE:OPEN",  cmd_valve_open},
    {"VALVE:CLOSE", cmd_valve_close}
};

#define HMI_TABLE_SIZE (sizeof(hmi_cmd_table) / sizeof(hmi_cmd_table[0]))

static void get_server_reply(const char *cmd, ProcessDB *db, struct pending_request *queue, char *reply_buf, size_t buf_len)
{
    for(size_t i = 0; i < HMI_TABLE_SIZE; i++)
    {
        if(strcmp(cmd, hmi_cmd_table[i].cmd_name) == 0)
        {
            hmi_cmd_table[i].action(db, queue, reply_buf, buf_len);
            return;
        }
    }

    snprintf(reply_buf, buf_len, "ERR Unknown tag\n");
}

static char   rx_line_buf[128];
static size_t rx_line_len = 0;

// Call this whenever a brand new client connects (e.g., inside accept() in poller.c)
void hmi_reset_client_buffer(void) {
    rx_line_len = 0;
    memset(rx_line_buf, 0, sizeof(rx_line_buf));
}

#include <sys/file.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

static int g_lock_fd = -1;

int server_init(void)
{
    // 1. Acquire singleton lock so only one gateway runs at a time
    g_lock_fd = open("/tmp/scada.lock", O_CREAT | O_RDWR, 0644);
    if (g_lock_fd < 0) {
        perror("Failed to open lock file");
        return -1;
    }
    
    if (flock(g_lock_fd, LOCK_EX | LOCK_NB) < 0) {
        if (errno == EWOULDBLOCK || errno == EACCES) {
            fprintf(stderr, "Error: Another instance of the SCADA master gateway is already running.\n");
        } else {
            perror("Error locking singleton instance");
        }
        close(g_lock_fd);
        return -1;
    }

    // 2. Unlink stale socket and bind server
    unlink("/tmp/scada.sock");
    
    int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        // Release lock if socket creation fails
        flock(g_lock_fd, LOCK_UN);
        close(g_lock_fd);
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/scada.sock", sizeof(addr.sun_path) - 1);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(server_fd);
        flock(g_lock_fd, LOCK_UN);
        close(g_lock_fd);
        return -1;
    }

    if (listen(server_fd, 5) < 0) {
        perror("listen");
        close(server_fd);
        unlink("/tmp/scada.sock");
        flock(g_lock_fd, LOCK_UN);
        close(g_lock_fd);
        return -1;
    }

    return server_fd;
}

int server_handle_request(int client_fd, ProcessDB *db, struct pending_request *queue)
{
    // 1. Ensure we don't overflow our buffer (leave 1 byte for null terminator)
    if (rx_line_len >= sizeof(rx_line_buf) - 1) {
        // Line too long without a newline! Send error and disconnect.
        char *err_msg = "ERR line too long\n";
        send(client_fd, err_msg, strlen(err_msg), MSG_NOSIGNAL);
        close(client_fd);
        hmi_reset_client_buffer();
        return -1;
    }

    // 2. Read into the buffer right after our existing accumulated bytes
    ssize_t n = read(client_fd, rx_line_buf + rx_line_len, sizeof(rx_line_buf) - 1 - rx_line_len);
    if (n < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0; // No data right now, keep waiting
        }
        perror("read failed");
        close(client_fd);
        hmi_reset_client_buffer();
        return -1;
    }
    else if (n == 0)
    {
        // Client disconnected (EOF)
        close(client_fd);
        hmi_reset_client_buffer();
        return -1;
    }

    // 3. Update length and null-terminate safely
    rx_line_len += n;
    rx_line_buf[rx_line_len] = '\0';

    // 4. Look for a newline character ('\n') in our accumulated buffer
    char *newline_ptr = strchr(rx_line_buf, '\n');
    if (newline_ptr == NULL)
    {
        // Packet was fragmented! We got some bytes ("PRES"), but no '\n' yet.
        // Return 0 and wait for the next poll() wake-up to read the rest.
        return 0;
    }

    // 5. We found a complete line! Terminate the string at the newline
    *newline_ptr = '\0';
    
    // Also strip any carriage return '\r' if sent by telnet/nc
    char *cr_ptr = strrchr(rx_line_buf, '\r');
    if (cr_ptr != NULL) {
        *cr_ptr = '\0';
    }

    // 6. Process command via lookup table and generate reply
    char tx_buf[256];
    get_server_reply(rx_line_buf, db, queue, tx_buf, sizeof(tx_buf));

    // 7. Send reply back to client
    ssize_t sent = send(client_fd, tx_buf, strlen(tx_buf), MSG_NOSIGNAL);
    if (sent < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            perror("send failed");
            close(client_fd);
            hmi_reset_client_buffer();
            return -1;
        }
    }

    // 8. SHIFT LEFTOVER BYTES: 
    // If the client sent a second command bundled in the same read (e.g., "TEMP\nVALVE\n"),
    // we must preserve everything after the first newline for the next cycle!
    size_t processed_len = (size_t)(newline_ptr - rx_line_buf) + 1;
    size_t leftover_len = rx_line_len - processed_len;
    
    if (leftover_len > 0) {
        memmove(rx_line_buf, newline_ptr + 1, leftover_len);
    }
    rx_line_len = leftover_len;
    rx_line_buf[rx_line_len] = '\0';

    return 0; // Keep connection open for next query
}

void hmi_shutdown(int server_fd, int client_fd)
{
    if (server_fd >= 0)
    {
        close(server_fd);
    }

    if(client_fd >= 0)
    {
        close(client_fd);
    }
    
    // Clean up the socket file from the filesystem
    unlink("/tmp/scada.sock");
    printf("[Info] : HMI server shut down and socket file unlinked.\n");
}

