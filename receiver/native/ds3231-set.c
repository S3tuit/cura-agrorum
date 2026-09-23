/* V1 fixed exit protocol; only this child receives cap_sys_time=ep. */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/rtc.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

int main(int argc, char **argv) {
    uint64_t seconds = 0;
    if (argc != 2 || !argv[1][0] || argv[1][0] == '0') return 33;
    for (const char *p = argv[1]; *p; ++p) {
        if (*p < '0' || *p > '9' || seconds > UINT64_C(4102444799) / 10) return 33;
        seconds = seconds * 10 + (unsigned)(*p - '0');
        if (seconds > UINT64_C(4102444799)) return 33;
    }
    if (seconds < UINT64_C(946684800)) return 33;
    time_t value = (time_t)seconds;
    struct tm calendar;
    if ((uint64_t)value != seconds || !gmtime_r(&value, &calendar)) return 33;
    struct rtc_time rtc = {
        .tm_sec = calendar.tm_sec, .tm_min = calendar.tm_min,
        .tm_hour = calendar.tm_hour, .tm_mday = calendar.tm_mday,
        .tm_mon = calendar.tm_mon, .tm_year = calendar.tm_year,
        .tm_wday = calendar.tm_wday, .tm_yday = calendar.tm_yday, .tm_isdst = 0
    };
    int fd = open("/dev/rtc-ds3231", O_RDWR | O_CLOEXEC);
    if (fd < 0) return (errno == ENOENT || errno == ENODEV || errno == ENXIO) ? 34 : 35;
    int result = ioctl(fd, RTC_SET_TIME, &rtc);
    close(fd);
    return result == 0 ? 32 : 36;
}
