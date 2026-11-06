/* fw_sim - run the firmware control loop on Linux against simulated servos.
 *
 * Host frames are read from stdin (a pipe, a FIFO or a file) and board frames are written to
 * stdout as raw protocol bytes, so a host driver can talk to it exactly as to the board's serial
 * port. Example (PTY bridge for a serial-port based driver):
 *     socat PTY,link=/tmp/armfw,raw,echo=0 EXEC:"build/fw_sim"
 *
 * Options:
 *   --fast            run virtual time as fast as possible (default: real time, 1 kHz)
 *   --steps N         stop after N control periods (default: until stdin EOF, plus --linger)
 *   --linger MS       keep running MS milliseconds after stdin EOF (default 0)
 *   --echo            simulate half-duplex echo of the board's own servo TX
 *   --estop-open-at MS  open the hardware e-stop loop at MS
 *   --log             print a summary to stderr at exit
 */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "sim.h"

static sim_t g_sim;

static void usage(void) {
    fprintf(stderr,
            "usage: fw_sim [--fast] [--steps N] [--linger MS] [--echo] [--estop-open-at MS] [--log]\n"
            "  reads host frames on stdin, writes board frames on stdout (raw protocol bytes)\n");
}

static int write_all(int fd, const uint8_t *b, size_t n) {
    while (n) {
        ssize_t w = write(fd, b, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) {
                struct pollfd p = {fd, POLLOUT, 0};
                poll(&p, 1, 100);
                continue;
            }
            return -1;
        }
        b += w;
        n -= (size_t)w;
    }
    return 0;
}

/* Move available stdin bytes into the sim (as much as fits). Returns -1 on EOF. */
static int pump_stdin(void) {
    uint8_t buf[4096];
    size_t room = SIM_HOST_IN_CAP - (g_sim.host_in.tail - g_sim.host_in.head);
    if (room == 0) return 0;
    if (room > sizeof buf) room = sizeof buf;
    ssize_t r = read(0, buf, room);
    if (r == 0) return -1;
    if (r < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    sim_host_send(&g_sim, buf, (size_t)r);
    return 1;
}

int main(int argc, char **argv) {
    int fast = 0, log = 0;
    long steps = -1, linger = 0, estop_at = -1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--fast")) fast = 1;
        else if (!strcmp(argv[i], "--log")) log = 1;
        else if (!strcmp(argv[i], "--echo")) g_sim.echo_tx = true;
        else if (!strcmp(argv[i], "--steps") && i + 1 < argc) steps = atol(argv[++i]);
        else if (!strcmp(argv[i], "--linger") && i + 1 < argc) linger = atol(argv[++i]);
        else if (!strcmp(argv[i], "--estop-open-at") && i + 1 < argc) estop_at = atol(argv[++i]);
        else {
            usage();
            return 2;
        }
    }
    bool echo = g_sim.echo_tx;
    sim_init(&g_sim, 0);
    g_sim.echo_tx = echo;
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

    bool eof = false;
    long eof_step = -1;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (long k = 0; steps < 0 || k < steps; k++) {
        if (!eof) {
            int r;
            while ((r = pump_stdin()) > 0) {
            }
            if (r < 0) {
                eof = true;
                eof_step = k;
            }
        }
        if (eof && steps < 0 && k - eof_step >= linger) break;
        if (estop_at >= 0 && k == estop_at) g_sim.estop_closed = false;
        sim_step(&g_sim);
        uint8_t out[8192];
        size_t n;
        while ((n = sim_host_recv(&g_sim, out, sizeof out)) > 0)
            if (write_all(1, out, n) < 0) return 0; /* reader went away */
        if (!fast) {
            next.tv_nsec += 1000000L;
            if (next.tv_nsec >= 1000000000L) {
                next.tv_nsec -= 1000000000L;
                next.tv_sec++;
            }
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR) {
            }
        }
    }
    if (log) {
        const control_t *c = &g_sim.ctl;
        fprintf(stderr,
                "fw_sim: %u steps, mode %d, faults 0x%04X, host frames %u (crc errors %u), acks %u, nacks %u, "
                "states %u, servo replies %u, misses %u\n",
                c->steps, (int)c->safety.mode, c->safety.faults, c->stats.host_frames,
                c->host_parser.stats.crc_errors, c->stats.acks, c->stats.nacks, c->stats.states,
                c->stats.servo_replies, c->stats.servo_misses);
    }
    return 0;
}
