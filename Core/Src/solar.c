#include "solar.h"
#include "math.h"
#include "ds1302.h"

int isLeapYear(void)
{
    return (TimeData.year % 4 == 0 && TimeData.year % 100 != 0) || (TimeData.year % 400 == 0);
}

int dayOfYear(void)
{
    int daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    if (isLeapYear())
    {
        daysInMonth[2] = 29;
    }

    int dayCount = TimeData.day;
    for (int i = 1; i < TimeData.month; i++)
    {
        dayCount += daysInMonth[i];
    }

    return dayCount;
}

double Solar_Delta(void)
{
    return 23.45 * sin((2.0 * 3.1415926) * (284.0 + (double)dayOfYear()) / 365.0);
}

double Solar_Angle_Alpha(float latitude)
{
    return asin(sin(latitude / 180.0 * 3.1415926) * sin(Solar_Delta() / 180.0 * 3.1415926) +
                cos(latitude / 180.0 * 3.1415926) * cos(Solar_Delta() / 180.0 * 3.1415926) *
                    cos((15.0 * (double)(TimeData.hour - 12.0) + 15.0 * ((double)TimeData.minute / 60.0)) / 180.0 * 3.1415926)) *
               180 / 3.1415926 +
           3;
}

double Solar_Angle_A(float latitude)
{
    if (15.0 * (TimeData.hour - 12) <= 0)
    {
        return acos((sin(Solar_Delta() / 180.0 * 3.1415926) -
                     sin(Solar_Angle_Alpha(latitude) / 180.0 * 3.1415926) * sin(latitude / 180.0 * 3.1415926)) /
                    (cos(Solar_Angle_Alpha(latitude) / 180.0 * 3.1415926) * cos(latitude / 180.0 * 3.1415926))) *
               180 / 3.1415926;
    }
    else
    {
        return 360 - acos((sin(Solar_Delta() / 180.0 * 3.1415926) -
                           sin(Solar_Angle_Alpha(latitude) / 180.0 * 3.1415926) * sin(latitude / 180.0 * 3.1415926)) /
                          (cos(Solar_Angle_Alpha(latitude) / 180.0 * 3.1415926) * cos(latitude / 180.0 * 3.1415926))) *
                         180 / 3.1415926;
    }
}

double info_range(double x, double min, double max)
{
    double shiftedx = x - min;
    double delta = max - min;
    return fmod(fmod(shiftedx, delta) + delta, delta) + min;
}

void Get_Sun(double latitude, double longitude, double *Alt, double *Azi)                // 直接拿取高度角方位角
{
    double gw_time = TimeData.hour - 8.0 + (double)TimeData.minute / 60.0 + (double)TimeData.second / 3600.0;
    double deynum = 367.0 * TimeData.year - 7.0 * (double)(TimeData.year + (TimeData.month + 9.0) / 12.0) / 4.0 + TimeData.day - 730531.5 +
                    (double)gw_time / 24.0;
    double daynum;
    int y = TimeData.year - 2000;
    int l = y / 4 - y / 100 + y / 400;                                               // 闰年个数
    // 分别计算出平年和闰年数量
    if ((TimeData.year / 4 == 0 && TimeData.year / 100 != 0) || TimeData.year / 400 == 0)
    {
        daynum = 1;
    }
    else
    {
        daynum = 0;
    }
    switch (TimeData.month)
    {
        case 1 :
            daynum = TimeData.day;
            break;
        case 2 :
            daynum = TimeData.day + 31;
            break;
        case 3 :
            daynum = daynum + TimeData.day + 59;
            break;
        case 4 :
            daynum = daynum + TimeData.day + 90;
            break;
        case 5 :
            daynum = daynum + TimeData.day + 120;
            break;
        case 6 :
            daynum = daynum + TimeData.day + 151;
            break;
        case 7 :
            daynum = daynum + TimeData.day + 181;
            break;
        case 8 :
            daynum = daynum + TimeData.day + 212;
            break;
        case 9 :
            daynum = daynum + TimeData.day + 243;
            break;
        case 10 :
            daynum = daynum + TimeData.day + 273;
            break;
        case 11 :
            daynum = daynum + TimeData.day + 304;
            break;
        case 12 :
            daynum = daynum + TimeData.day + 334;
            break;
    }
    daynum = daynum + 365 * y + l + (double)gw_time / 24 - 0.5;
    // 计算太阳的平均经度
    double mean_long = daynum * 0.01720279239 + 4.894967873;
    // 计算太阳的平均近点角
    double mean_anom = daynum * 0.01720197034 + 6.240040768;
    // 计算太阳的黄道经度
    double eclip_long = (mean_long + 0.03342305518 * sin(mean_anom) + 0.0003490658504 * sin(2 * mean_anom));
    // 计算黄道倾角
    double obliquity = 0.4090877234 - 0.000000006981317008 * daynum;
    // 计算太阳的赤经
    double rasc = atan2(cos(obliquity) * sin(eclip_long), cos(eclip_long));
    // 计算太阳的赤纬
    double decl = asin(sin(obliquity) * sin(eclip_long));
    // 计算当地的恒星时
    double sidereal = 4.894961213 + 6.300388099 * daynum + longitude * 3.141592654 / 180;
    // 计算太阳的时角
    double hour_ang = sidereal - rasc;
    // 计算太阳高度角
    *Alt = asin(sin(decl) * sin(latitude * 3.141592654 / 180) + cos(decl) * cos(latitude * 3.141592654 / 180) * cos(hour_ang));
    // 计算太阳的方位角
    *Azi = atan2(-cos(decl) * cos(latitude * 3.141592654 / 180) * sin(hour_ang), sin(decl) - sin(latitude * 3.141592654 / 180) * sin(*Alt));
    *Azi = info_range(*Azi * 180 / 3.141592654, 0.0, 360.0);
    *Alt = info_range(*Alt * 180 / 3.141592654, -180.0, 180.0);
    double targ = (*Alt + (10.3 / (*Alt + 5.11))) * 3.141592654 / 180;
    *Alt = *Alt + (1.02 / tan(targ)) / 60;
}
