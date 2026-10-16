#ifndef _XOPEN_SOURCE 
#define _XOPEN_SOURCE 700 
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

static void print_menu(void) {
    printf("\n========================================\n");
    printf("       SCADA HMI CLIENT CONSOLE         \n");
    printf("========================================\n");
    printf("  [1] Get Pressure    (PRESSURE)\n");
    printf("  [2] Get Flow        (FLOW)\n");
    printf("  [3] Get Temperature (TEMP)\n");
    printf("  [4] Get Valve State (VALVE)\n");
    printf("  [5] Get All Data    (ALL)\n");
    printf("  [6] Open Valve      (VALVE:OPEN)\n");
    printf("  [7] Close Valve     (VALVE:CLOSE)\n");
    printf("  [0] Exit\n");
    printf("========================================\n");
    printf("Enter your choice [0-7]: ");
}

static int send_command_to_server(const char *cmd) {
    int client_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (client_fd < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_un cli = {0};
    cli.sun_family = AF_UNIX;
    strncpy(cli.sun_path, "/tmp/scada.sock", sizeof(cli.sun_path) - 1);

    if (connect(client_fd, (const struct sockaddr *)&cli, sizeof(cli)) < 0) {
        perror("connect (Is the SCADA server running?)");
        close(client_fd);
        return -1;
    }

    char req_buf[128];
    int req_len = snprintf(req_buf, sizeof(req_buf), "%s\n", cmd);
    if (send(client_fd, req_buf, (size_t)req_len, MSG_NOSIGNAL) < 0) {
        perror("send");
        close(client_fd);
        return -1;
    }

    char rx_buf[256];
    ssize_t n;
    while ((n = read(client_fd, rx_buf, sizeof(rx_buf) - 1)) > 0) {
        rx_buf[n] = '\0';
        printf("Reply: %s", rx_buf);
        if (strchr(rx_buf, '\n') != NULL) {
            break;
        }
    }

    close(client_fd);
    return 0;
}

int main(int argc, char *argv[]) {
    // Mode A: Direct argument passed (e.g. ./hmi_client ALL)
    if (argc >= 2) {
        return send_command_to_server(argv[1]) == 0 ? 0 : 1;
    }

    // Mode B: Interactive UI Menu (no arguments passed)
    int choice = -1;
    while (choice != 0) {
        print_menu();
        if (scanf("%d", &choice) != 1) {
            printf("Invalid input. Please enter a number between 0 and 7.\n");
            while (getchar() != '\n'); // Clear input buffer
            continue;
        }

        const char *cmd = NULL;
        switch (choice) {
            case 1: cmd = "PRESSURE"; break;
            case 2: cmd = "FLOW"; break;
            case 3: cmd = "TEMP"; break;
            case 4: cmd = "VALVE"; break;
            case 5: cmd = "ALL"; break;
            case 6: cmd = "VALVE:OPEN"; break;
            case 7: cmd = "VALVE:CLOSE"; break;
            case 0: printf("Exiting HMI Client.\n"); break;
            default:
                printf("Unknown option. Try again.\n");
                continue;
        }

        if (cmd != NULL) {
            send_command_to_server(cmd);
            // Pause briefly so user can read the reply before menu redraws
            sleep(1);
        }
    }

    return 0;
}