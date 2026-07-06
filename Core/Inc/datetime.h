#ifndef DATETIME_H
#define DATETIME_H

#include <stdint.h>
#include "stm32l4xx_hal_rtc.h"
#include "libs.h"

#define EPOCH_YEAR 2000

extern RTC_HandleTypeDef hrtc;

static const uint8_t MONTH_DAYS[12] =
{
    31,28,31,30,31,30,31,31,30,31,30,31
};

typedef struct
{
    RTC_TimeTypeDef ti;
    RTC_DateTypeDef da;
} DateTime;

static inline int is_leap_year(int year)
{
    /* For STM32 RTC years 2000-2099 this is sufficient */
    return (year % 4) == 0;
}

static inline uint8_t days_in_month(int year, int month)
{
    if (month == 2 && is_leap_year(year))
        return 29;

    return MONTH_DAYS[month - 1];
}

static inline uint32_t toepoch(DateTime dt)
{
    uint32_t seconds = 0;

    int year = EPOCH_YEAR + dt.da.Year;

    /* Add complete years */
    for (int y = EPOCH_YEAR; y < year; y++)
    {
        seconds += (uint32_t)(is_leap_year(y) ? 366 : 365) * 86400ULL;
    }

    /* Add complete months */
    for (int m = 1; m < dt.da.Month; m++)
    {
        seconds += (uint32_t)days_in_month(year, m) * 86400ULL;
    }

    /* Add complete days */
    seconds += (uint32_t)(dt.da.Date - 1) * 86400ULL;

    /*
     * RTC configured in 24-hour mode.
     * If using 12-hour mode, convert first.
     */
    seconds += (uint32_t)dt.ti.Hours * 3600ULL;
    seconds += (uint32_t)dt.ti.Minutes * 60ULL;
    seconds += (uint32_t)dt.ti.Seconds;
    return seconds;
}

static inline DateTime fromepoch(uint32_t epoch)
{
    DateTime dt = {0};

    int year = EPOCH_YEAR;

    /* Determine year */
    while (1)
    {
        uint32_t year_seconds =
            (uint32_t)(is_leap_year(year) ? 366 : 365) * 86400ULL;

        if (epoch < year_seconds)
            break;

        epoch -= year_seconds;
        year++;
    }

    dt.da.Year = year - EPOCH_YEAR;

    /* Determine month */
    int month = 1;

    while (1)
    {
        uint32_t month_seconds =
            (uint32_t)days_in_month(year, month) * 86400ULL;

        if (epoch < month_seconds)
            break;

        epoch -= month_seconds;
        month++;
    }

    dt.da.Month = month;

    /* Determine day */
    dt.da.Date = (epoch / 86400ULL) + 1;
    epoch %= 86400ULL;

    /* Hour */
    dt.ti.Hours = epoch / 3600ULL;
    epoch %= 3600ULL;

    /* Minute */
    dt.ti.Minutes = epoch / 60ULL;

    /* Second */
    dt.ti.Seconds = epoch % 60ULL;

    return dt;
}

static uint32_t rtcElapsed() {
	RTC_TimeTypeDef ti = {0};
	RTC_DateTypeDef da = {0};

	HAL_RTC_GetTime(&hrtc, &ti, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &da, RTC_FORMAT_BIN);

	return toepoch((DateTime){ti,da});
}

#endif