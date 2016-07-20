
/*  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2016, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  This is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  This software is distributed in the hope that it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

class TimeSpec{
public:
    static void addms(struct timespec *ts, long ms) {
        int sec = ms / 1000;
        ms = ms - sec * 1000;
        ts->tv_nsec += ms * 1000000;
        ts->tv_sec += ts->tv_nsec / 1000000000 + sec;
        ts->tv_nsec = ts->tv_nsec % 1000000000;
    }

    static void future(struct timespec *ts, unsigned int ms) {
        now(ts);
        addms(ts,ms);
    }

    static void subtract(struct timespec *a, struct timespec *b) {
        a->tv_nsec = a->tv_nsec - b->tv_nsec;
        if (a->tv_nsec < 0) {
            // borrow.
            a->tv_nsec += 1000000000;
            a->tv_sec--;
        }
        a->tv_sec = a->tv_sec - b->tv_sec;
    }

    static int milliseconds(struct timespec *a) {
        return a->tv_sec * 1000 + a->tv_nsec / 1000000;
    }

    static int microseconds(struct timespec *a) {
        return a->tv_sec * 1000000 + a->tv_nsec / 1000;
    }

    static void now(struct timespec *ts) {
        clock_gettime(CLOCK_REALTIME, ts);
    }

    static unsigned long long timestamp() {
        struct timespec ts;
        now(&ts);
        return ((unsigned long long)(ts.tv_sec)) * 1000000 + ((unsigned long long)(ts.tv_nsec)) / 1000;
    }

    static unsigned int timestamp_ms() {
        struct timespec ts;
        now(&ts);
        unsigned long long res = ((unsigned long long)(ts.tv_sec)) * 1000000 + ((unsigned long long)(ts.tv_nsec)) / 1000;
        return (unsigned int)(res/1000);
    }

    static int subtract_to_ms(struct timespec *a, struct timespec *b) {
        subtract(a, b);
        return milliseconds(a);
    }
    static int elapsed_ms(struct timespec *a) {
        struct timespec ts;
        now(&ts);
        subtract(&ts, a);
        return milliseconds(&ts);
    }

    static int elapsed_us(struct timespec *a) {
        struct timespec ts;
        now(&ts);
        subtract(&ts, a);
        return microseconds(&ts);
    }

    static void elapsed_print_us(struct timespec *a, char * text){
        fprintf(stderr, "%s: %d\n", text, elapsed_us(a));
    }

};
