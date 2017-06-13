//! robotkernel module jitter_measurement
/*!
 * author: Florian Schmidt <florian.schmidt@dlr.de>
 */

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef TTY_CONTROL_SIGNALS_H
#define TTY_CONTROL_SIGNALS_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

class tty_control_signals {
    /**
     * host inputs:
     * DCD - Data Carrier Detect TIOCM_CAR pin1
     * CTS - Clear To Send       TIOCM_CTS pin8
     * RI  - Ring Indicator      TIOCM_RNG pin9
     * DSR - Data Set Ready      TIOCM_LE  pin6
     *
     * host outputs:
     * DTR - Data Terminal Ready TIOCM_DTR pin4
     * RTS - Request To Send     TIOCM_RTS pin7
     */

    public:
        int fd;
        int status;

        tty_control_signals(const char* port) {
            fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
            if(fd == -1) {
                perror("open");
                return;
            }
            int ret = ioctl(fd, TIOCMGET, &status);
            if(ret == -1) {
                perror("ioctrl TIOCMGET");
                return;
            }
        }
        ~tty_control_signals() {
            if(fd != -1) 
                close(fd);
        }

        void set_status() {
            ioctl(fd, TIOCMSET, &status);
        }

        void set_rts(bool value) {
            if(value)
                status |= TIOCM_RTS;
            else
                status &= ~TIOCM_RTS;
            set_status();
        }
        void set_dtr(bool value) {
            if(value)
                status |= TIOCM_DTR;
            else
                status &= ~TIOCM_DTR;
            set_status();
        }
        void pulse_rts() {
            set_rts(true);
            set_rts(false);
        }
        void pulse_dtr() {
            set_dtr(true);
            set_dtr(false);
        }
        void pulse_rts_neg() {
            set_rts(false);
            set_rts(true);
        }
        void pulse_dtr_neg() {
            set_dtr(false);
            set_dtr(true);
        }
};

#endif // TTY_CONTROL_SIGNALS_H

