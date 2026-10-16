/*
 * test_m11.c -- M11 Unit Tests for:
 *   1. Database layout, seqlock, and state updates (gateway/db.c, gateway/db.h)
 *   2. Alarm edge logic, hysteresis, and severity levels (gateway/alarm.c, gateway/alarm.h)
 *   3. HMI line parser, fragmented packets, bundled commands, and responses (server/hmi.c, server/hmi.h)
 */

#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE 700
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <time.h>
#include "db.h"
#include "alarm.h"
#include "hmi.h"
#include "poller.h"

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *name)
{
    if (cond) {
        g_pass++;
        printf("  PASS  %s\n", name);
    } else {
        g_fail++;
        printf("  FAIL  %s\n", name);
    }
}

static void check_int(long got, long expected, const char *name)
{
    if (got == expected) {
        g_pass++;
        printf("  PASS  %s\n", name);
    } else {
        g_fail++;
        printf("  FAIL  %s (got %ld, expected %ld)\n", name, got, expected);
    }
}

static void check_str(const char *got, const char *expected, const char *name)
{
    if (strcmp(got, expected) == 0) {
        g_pass++;
        printf("  PASS  %s\n", name);
    } else {
        g_fail++;
        printf("  FAIL  %s (got \"%s\", expected \"%s\")\n", name, got, expected);
    }
}

/* --------------------------------------------------------------------------
 * 1. Database Layout & State Transitions (gateway/db.c & gateway/db.h)
 * -------------------------------------------------------------------------- */
static void test_db_layout_and_ops(void)
{
    printf("\n[1] Database Layout & Operations (db.c / db.h)\n");

    check_int(sizeof(ProcessDB), 40, "sizeof(ProcessDB) is exactly 40 bytes");
    check_int(offsetof(ProcessDB, last_poll_time), 0, "offsetof(last_poll_time) == 0");
    check_int(offsetof(ProcessDB, seq_count), 8, "offsetof(seq_count) == 8");
    check_int(offsetof(ProcessDB, poll_count), 12, "offsetof(poll_count) == 12");
    check_int(offsetof(ProcessDB, crc_errors), 16, "offsetof(crc_errors) == 16");
    check_int(offsetof(ProcessDB, failed_polls), 20, "offsetof(failed_polls) == 20");
    check_int(offsetof(ProcessDB, registers), 24, "offsetof(registers) == 24");
    check_int(offsetof(ProcessDB, coil_state), 32, "offsetof(coil_state) == 32");
    check_int(offsetof(ProcessDB, device_online), 33, "offsetof(device_online) == 33");

    const char *test_db_path = "/tmp/test_process_m11.db";
    unlink(test_db_path);

    ProcessDB *db = db_init(test_db_path);
    check(db != NULL, "db_init creates and maps database");
    if (!db) return;

    check_int(db->device_online, 1, "initial device_online == 1");
    check_int(db->seq_count, 0, "initial seq_count == 0 (even)");

    uint16_t vals[4] = { 3260, 3550, 450, 1 };
    db_update_success(db, vals, 4);
    check_int(db->seq_count, 2, "seq_count incremented by 2 after success (remains even)");
    check_int(db->poll_count, 1, "poll_count == 1");
    check_int(db->registers[0], 3260, "registers[0] (Pressure) == 3260");
    check_int(db->registers[1], 3550, "registers[1] (Flow) == 3550");
    check_int(db->registers[2], 450, "registers[2] (Temp) == 450");
    check_int(db->registers[3], 1, "registers[3] (Valve) == 1");
    check_int(db->coil_state, 1, "coil_state synced to registers[3]");
    check_int(db->device_online, 1, "device_online stays 1");

    db_update_failure(db, 1);
    check_int(db->crc_errors, 1, "CRC failure increments crc_errors");
    check_int(db->failed_polls, 0, "CRC failure does not increment failed_polls");
    check_int(db->device_online, 1, "device remains online after single CRC error");

    db_update_failure(db, 0);
    check_int(db->failed_polls, 1, "failed_polls == 1");
    check_int(db->device_online, 1, "still online after 1 timeout");

    db_update_failure(db, 0);
    check_int(db->failed_polls, 2, "failed_polls == 2");
    check_int(db->device_online, 1, "still online after 2 timeouts");

    db_update_failure(db, 0);
    check_int(db->failed_polls, 3, "failed_polls == 3");
    check_int(db->device_online, 0, "device_online becomes 0 on 3rd consecutive failure");

    db_update_success(db, vals, 4);
    check_int(db->device_online, 1, "device_online recovers to 1 on success");
    check_int(db->failed_polls, 0, "failed_polls reset to 0 on success");

    db_close(db);
    unlink(test_db_path);
}

/* --------------------------------------------------------------------------
 * 2. Alarm Edge Logic, Hysteresis & Severity (gateway/alarm.c & gateway/alarm.h)
 * -------------------------------------------------------------------------- */
static void sleep_30ms(void)
{
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 30000000L };
    nanosleep(&ts, NULL);
}

static int count_lines_in_file(const char *path, char *last_line, size_t max_len)
{
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
    int count = 0;
    char buf[256];
    while (fgets(buf, sizeof(buf), fp)) {
        count++;
        if (last_line && max_len > 0) {
            strncpy(last_line, buf, max_len - 1);
            last_line[max_len - 1] = '\0';
        }
    }
    fclose(fp);
    return count;
}

static void test_alarm_edge_and_hysteresis(void)
{
    printf("\n[2] Alarm Edge Logic & Hysteresis (alarm.c / alarm.h)\n");

    check_int(sizeof(AlarmEvent), 28, "sizeof(AlarmEvent) is exactly 28 bytes");

    const char *test_log = "/tmp/test_alarms_m11.log";
    unlink(test_log);

    int rc = alarm_init(test_log);
    check_int(rc, 0, "alarm_init forks logger child and opens pipe");

    char last_line[256] = {0};
    uint16_t regs[4] = { 2000, 4000, 1100, 0 };

    alarm_check(regs);
    sleep_30ms();
    check_int(count_lines_in_file(test_log, last_line, sizeof(last_line)), 0,
              "below limits (T=110.0 C) -> 0 alarm entries");

    regs[2] = 1200;
    alarm_check(regs);
    sleep_30ms();
    check_int(count_lines_in_file(test_log, last_line, sizeof(last_line)), 0,
              "at exact limit (T=120.0 C / 1200) -> 0 alarm entries");

    regs[2] = 1205;
    alarm_check(regs);
    sleep_30ms();
    check_int(count_lines_in_file(test_log, last_line, sizeof(last_line)), 1,
              "rising edge above limit (T=1205) -> logs 1 entry");
    check(strstr(last_line, "WARN TEMP 1205 1200") != NULL,
          "log entry contains 'WARN TEMP 1205 1200'");

    regs[2] = 1250;
    alarm_check(regs);
    regs[2] = 1300;
    alarm_check(regs);
    sleep_30ms();
    check_int(count_lines_in_file(test_log, last_line, sizeof(last_line)), 1,
              "staying above limit (1250, 1300) -> still exactly 1 entry (no spam)");

    regs[2] = 1180;
    alarm_check(regs);
    regs[2] = 1250;
    alarm_check(regs);
    sleep_30ms();
    check_int(count_lines_in_file(test_log, last_line, sizeof(last_line)), 1,
              "dip to 1180 (inside hysteresis >= 1170) then spike -> does NOT re-trigger");

    regs[2] = 1169;
    alarm_check(regs);

    regs[2] = 1350;
    alarm_check(regs);
    sleep_30ms();
    check_int(count_lines_in_file(test_log, last_line, sizeof(last_line)), 2,
              "drop below hysteresis (1169) then spike to 1350 -> logs 2nd entry");
    check(strstr(last_line, "HIGH TEMP 1350 1200") != NULL,
          "overshoot > 10% logs 'HIGH TEMP 1350 1200'");

    regs[2] = 1100;
    alarm_check(regs);
    regs[2] = 1550;
    alarm_check(regs);
    sleep_30ms();
    check_int(count_lines_in_file(test_log, last_line, sizeof(last_line)), 3,
              "reset then spike to 1550 -> logs 3rd entry");
    check(strstr(last_line, "CRIT TEMP 1550 1200") != NULL,
          "overshoot > 25% logs 'CRIT TEMP 1550 1200'");

    alarm_shutdown();
    sleep_30ms();
    unlink(test_log);
}

/* --------------------------------------------------------------------------
 * 3. HMI Line Parser & Socket Handler (server/hmi.c & server/hmi.h)
 * -------------------------------------------------------------------------- */
static void test_hmi_line_parser(void)
{
    printf("\n[3] HMI Line Parser & Protocol Handler (server/hmi.c)\n");

    ProcessDB db;
    memset(&db, 0, sizeof(db));
    db.device_online = 1;
    db.registers[0] = 3260;
    db.registers[1] = 3550;
    db.registers[2] = 450;
    db.registers[3] = 1;

    struct pending_request queue;
    memset(&queue, 0, sizeof(queue));

    int sv[2];
    check_int(socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0, "create AF_UNIX socketpair for testing");
    int flags = fcntl(sv[0], F_GETFL, 0);
    fcntl(sv[0], F_SETFL, flags | O_NONBLOCK);

    hmi_reset_client_buffer();
    char reply[256];
    ssize_t n;

    write(sv[1], "PRESSURE\n", 9);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "handle PRESSURE");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "PRESSURE 326.0 bar\n", "PRESSURE reply matches");

    write(sv[1], "FLOW\r\n", 6);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "handle FLOW with CR-LF (strips CR)");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "FLOW 355.0 L/min\n", "FLOW reply matches");

    write(sv[1], "TE", 2);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "partial 'TE' returns 0 (waits for newline)");
    write(sv[1], "MP\n", 3);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "remaining 'MP' + LF completes line");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "TEMP 45.0 C\n", "fragmented TEMP reply matches");

    write(sv[1], "VALVE\nALL\n", 10);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "first call processes VALVE from bundled buffer");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "VALVE OPEN\n", "VALVE reply matches");

    write(sv[1], "PRESSURE\n", 9);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "second call processes leftover ALL");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "{\"p\":326.0,\"f\":355.0,\"t\":45.0,\"valve\":\"OPEN\",\"online\":true}\n",
              "ALL JSON reply matches");

    hmi_reset_client_buffer();

    write(sv[1], "VALVE:OPEN\n", 11);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "handle VALVE:OPEN");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "OK\n", "VALVE:OPEN replies OK");
    check(queue.has_request == 1 && queue.is_write_coil == 1 && queue.write_value == 0xFF00,
          "VALVE:OPEN populates queue with 0xFF00 write coil request");

    write(sv[1], "VALVE:CLOSE\n", 12);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "handle VALVE:CLOSE");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "OK\n", "VALVE:CLOSE replies OK");
    check(queue.has_request == 1 && queue.is_write_coil == 1 && queue.write_value == 0x0000,
          "VALVE:CLOSE populates queue with 0x0000 write coil request");

    write(sv[1], "BOGUS\n", 6);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "handle BOGUS command");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "ERR Unknown tag\n", "unknown command replies 'ERR Unknown tag'");

    db.device_online = 0;
    write(sv[1], "PRESSURE\n", 9);
    check_int(server_handle_request(sv[0], &db, &queue), 0, "handle PRESSURE when offline");
    n = read(sv[1], reply, sizeof(reply) - 1);
    reply[n > 0 ? n : 0] = '\0';
    check_str(reply, "OFFLINE\n", "offline PRESSURE replies 'OFFLINE'");

    close(sv[0]);
    close(sv[1]);
}

int main(void)
{
    printf("=== M11 Unit Tests (Database, Alarm Edge, HMI Line Parser) ===\n");
    test_db_layout_and_ops();
    test_alarm_edge_and_hysteresis();
    test_hmi_line_parser();
    printf("\n==== %d passed, %d failed ====\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
