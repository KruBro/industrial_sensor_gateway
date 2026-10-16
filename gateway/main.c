#include <unistd.h>
#include <stdio.h>
#include "serial_io.h"
#include "main_helper.h"
#include "poller.h"

int main(int argc, char *argv[])
{
    cli_args_t args;

    if (parse_args(argc, argv, &args) < 0)
    {
        return 1;
    }

    int fd = open_port(args.port);
    if (fd < 0)
    {
        return 1;
    }

    int rc;
    
    if (args.runs == 1)
    {
        rc = run_single(fd, (uint8_t)args.addr, (uint16_t)args.count);
    }
    else if(args.runs > 1)
    {
        rc = run_counting(fd, (uint8_t)args.addr, (uint16_t)args.count, (unsigned)args.runs);
    }
    else if(args.runs == 0)
    {
        rc = poller_run(fd, (uint8_t)args.addr, (uint16_t)args.count, POLL_INTERVAL_MS, args.valve_value, args.log_path);
    }

    close(fd);
    return rc;
}
