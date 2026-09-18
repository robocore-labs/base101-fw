#define _POSIX_C_SOURCE 200809L
#include "calibration_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
static char reply[2048];
static bool command(calibration_protocol_t *s, const char *input, uint64_t now) {
    char line[512]; snprintf(line, sizeof(line), "%s", input);
    return calibration_command(s, line, now, reply, sizeof(reply));
}
static calibration_protocol_t make_state(void) {
    calibration_protocol_t s;
    calibration_init(&s, (calibration_parameters_t){.ax=.7, .jx=2, .aw=3, .jw=10, .k_icr=1.5});
    s.at_rest = true;
    return s;
}
int main(int argc, char **argv) {
    calibration_protocol_t s = make_state();
    if (argc == 2 && !strcmp(argv[1], "--malformed")) {
        char line[512];
        if (fgets(line, sizeof(line), stdin)) {
            puts("{\"id\":1,\"ok\":true,\"rpm\":000000000000}"); fflush(stdout);
            (void)fgets(line, sizeof(line), stdin); // Keep PTY master alive until cleanup stop.
        }
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "--emulate")) {
        char line[512];
        while (fgets(line, sizeof(line), stdin)) {
            struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
            uint64_t now = (uint64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
            line[strcspn(line, "\r\n")] = 0;
            calibration_expire(&s, now);
            if (s.releasing && now - s.operator_us > 200000) calibration_stop(&s,"released");
            s.ready = true; s.at_rest = !s.driving && !s.releasing;
            command(&s, line, now);
            fputs(reply, stdout); fflush(stdout);
        }
        return 0;
    }
    command(&s, "1 set 1 2 3 4 1.5", 0);
    assert(strstr(reply, "handshake_required"));
    assert(command(&s, "2 stop", 1));
    assert(command(&s, "3 hello 2", 100));
    assert(s.session);
    command(&s, "4 set 1 2 3 4 2", 101);
    assert(s.parameters.ax == 1 && s.parameters.k_icr == 2);
    const char *bad[] = {"5 set nan 2 3 4 2", "6 set 1 2 3 4 inf",
        "7 set 1 2 3 4 .9", "8 set 1 2 3 4 6", "9 set 1 2 3 4 2 extra",
        "10 set 1 2 3 4", "-1 status", "2147483648 status", "11 set 1x 2 3 4 2"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) {
        command(&s, bad[i], 102);
        assert(strstr(reply, "\"ok\":false"));
        assert(s.parameters.ax == 1 && s.parameters.k_icr == 2);
    }
    command(&s, "12 status", 499999);
    assert(s.heartbeat_us == 100);
    assert(!calibration_expire(&s, 500099));
    assert(calibration_expire(&s, 500100));
    assert(!s.session && !strcmp(s.stop_reason, "heartbeat_timeout"));
    command(&s, "13 heartbeat", 500101);
    assert(!s.session && strstr(reply, "handshake_required"));
    assert(command(&s, "14 stop", 500102));
    command(&s, "15 hello 3", 500103);
    assert(!s.session && strstr(reply, "unsupported_protocol"));
    command(&s, "16\thello\t2", 600000);
    command(&s, "17 heartbeat", 600100);
    assert(s.heartbeat_us == 600100);
    calibration_disconnect(&s);
    assert(!s.session && !strcmp(s.stop_reason, "disconnect"));
    s = make_state();
    command(&s, "20 hello 2", 0);
    command(&s, "21 drive .1 .3", 1);
    assert(strstr(reply, "not_ready") && !s.driving);
    s.ready = true;
    command(&s, "22 drive .1 .3", 10);
    assert(s.driving && s.action == CAL_DRIVE);
    command(&s, "23 set 2 3 4 5 1.6", 11);
    assert(strstr(reply, "not_at_rest") && s.parameters.ax == .7);
    command(&s, "24 heartbeat", 200000);
    assert(!calibration_expire(&s, 350009));
    assert(calibration_expire(&s, 350010));
    assert(s.session && !s.driving && !strcmp(s.stop_reason, "operator_timeout"));
    command(&s, "25 drive .1 .3", 350011);
    command(&s, "26 release", 350012);
    assert(s.releasing && !s.driving && s.action == CAL_RELEASE);
    assert(s.vx == 0 && s.wz == 0);
    command(&s, "27 heartbeat", 350013);
    // Keep server lease alive without renewing operator motion.
    for (uint64_t now = 700000; now < 2350012; now += 350000)
        command(&s, "28 heartbeat", now);
    assert(calibration_expire(&s, 2350012));
    assert(!s.releasing && !strcmp(s.stop_reason, "release_timeout"));
    command(&s, "29 drive .1 .3", 2350013);
    command(&s, "30 drive nan .3", 2350014);
    assert(!s.driving && strstr(reply, "invalid_velocity"));
    calibration_fault(&s, "motor_or_gyro_fault");
    command(&s, "31 drive .1 .3", 2350015);
    assert(strstr(reply, "not_ready") && s.fault);
    command(&s, "32 calibrate", 2350016);
    assert(!s.fault && !s.ready && s.action == CAL_GYRO);
    s.at_rest = false;
    command(&s, "33 set 2 3 4 5 1.6", 2350017);
    assert(strstr(reply, "not_at_rest") && s.parameters.ax == .7);
    s=make_state();command(&s,"40 hello 2",100);s.ready=true;
    command(&s,"41 bound 1000",101);assert(s.bound_until_us==1000101);
    command(&s,"42 drive .1 0",102);
    for(uint64_t now=100000;now<1000101;now+=100000) {
        command(&s,"43 heartbeat",now);command(&s,"44 drive .1 0",now);
    }
    assert(calibration_expire(&s,1000101));assert(!s.driving && !strcmp(s.stop_reason,"trial_deadline"));
    command(&s,"45 config .8 3 4 12 1.7 .04 .3 1 180 .3 .7 15 .4 .2 .3 .004 .2 .8 0",1000102);
    assert(s.parameters.radius==.04 && s.parameters.rpm_max==180 && s.action==CAL_SET);
    command(&s,"46 config .8 3 4 12 1.7 .04 .3 1.5 180 .3 .7 15 .4 .2 .3 .004 .2 .8 0",1000103);
    assert(strstr(reply,"invalid_parameters") && s.parameters.ramp==1);
    command(&s,"47 measure",1000104);assert(s.action==CAL_MEASURE);
    puts("calibration protocol checks passed");
    return 0;
}
