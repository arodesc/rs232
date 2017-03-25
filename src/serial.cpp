#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <math.h>
#include <sstream>

#include "serial.hpp"

#include <iomanip>

int Serial::open_tty(const std::string filename, int baudiovel, int parity, int lchar, int stopbits) {

	/* Declaracion de variables. */
	int handle, status;
	struct termios toptions;

	
	/* Abrir el puerto y obtener su descriptor */
	handle = open(filename.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (handle < 0) {
		return -1;
	}

	/* Obtener las condiciones actuales del puerto */
	tcgetattr(handle, &toptions);

	
	/* obtener los ajustes actuales del puerto */
	 tcgetattr(handle, &toptions);

	 /* establecer la velocidad */
	 //cfsetispeed(&toptions, baudiovel);
	 //cfsetospeed(&toptions, baudiovel);

	 cfmakeraw(&toptions);
	 baud_tty(handle, baudiovel);
	 lChar_tty(handle, lchar);
	 stopBits_tty(handle, stopbits);
	 parity_tty(handle, parity);


	 toptions.c_cflag |= (CS8 | CLOCAL | CREAD | PARODD);
	 toptions.c_oflag = 0;
	 toptions.c_iflag = IGNPAR;
	 //toptions.c_iflag |= (IXON | IXOFF | IXANY);

	/* esperar 1 carácter */
	toptions.c_cc[VMIN] = 1;
	/* Sin tiempo mínimo de lectura entre carácteres */
	toptions.c_cc[VTIME] = 0;

	toptions.c_cc[VSTART] = 0;
	toptions.c_cc[VSTOP] = 0;

	/* Establecer las opciones */
	tcsetattr(handle, TCSANOW, &toptions);

	/* Esperar al reset del puerto serie */
	usleep(1000);

	if (ioctl(handle, TIOCMGET, &status) == -1) {
		return -1;
	}

	status |= TIOCM_DTR;  //turn on DTR
	status |= TIOCM_RTS;  //turn on RTS

	if (ioctl(handle, TIOCMSET, &status) == -1) {
		return -1;
	}

	/* Vaciar el buffer serie */
	tcflush(handle, TCIFLUSH);
	return handle;

}

int Serial::open_alt_tty(const std::string filename, int baudiovel, int parity, int lchar, int stopbits) {

	/* Declaracion de variables. */
	int handle, status;
	struct termios toptions;
	
	/* Abrir el puerto y obtener su descriptor */
	handle = open(filename.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (handle < 0) {
		return -1;
	}
	

	/* obtener los ajustes actuales del puerto */
	 tcgetattr(handle, &toptions);

	 /* establecer la velocidad */
	 //cfsetispeed(&toptions, baudiovel);
	 //cfsetospeed(&toptions, baudiovel);

	 cfmakeraw(&toptions);
	 baud_tty(handle, baudiovel);
	 lChar_tty(handle, lchar);
	 stopBits_tty(handle, stopbits);
	 parity_tty(handle, parity);


	 //toptions.c_cflag = CS8 | CLOCAL | CREAD | PARODD;
	 //toptions.c_oflag = 0;
	 //toptions.c_iflag = IGNPAR;
	 //toptions.c_iflag |= (IXON | IXOFF | IXANY);

	/* esperar 1 carácter */
	toptions.c_cc[VMIN] = 1;
	/* Sin tiempo mínimo de lectura entre carácteres */
	toptions.c_cc[VTIME] = 0;
	
	toptions.c_cc[VSTART] = 0;
	toptions.c_cc[VSTOP] = 0;

	/* Establecer las opciones */
	tcsetattr(handle, TCSANOW, &toptions);

	/* Esperar al reset del puerto serie */
	usleep(1000);


	if (ioctl(handle, TIOCMGET, &status) == -1) {
		return -1;
	}

	status |= TIOCM_DTR;  //turn on DTR
	status |= TIOCM_RTS;  //turn on RTS

	if (ioctl(handle, TIOCMSET, &status) == -1) {
		return -1;
	}

	/* Vaciar el buffer serie */
	//tcflush(handle, TCIFLUSH);
	return handle;

}


int Serial::close_tty(int handle) {

	//No es necesario desactivar el control de flujo antes de cerrar el puerto
	if (close(handle) < 0) {
		return -1;
	} else {
		return 0;
	}
}


int Serial::send_tty(int handle, char *buffer, int len, bool control)
{
	int i, byte;
	std::stringstream envio;

	i = 0;
	stopwatch.start_timer();

	while (i < len) {

		if ((control|| getDSR_tty(handle) == 1)) {
			byte = write(handle, &buffer[i], 1);
			int err = errno;
			if (byte > 0) {
				envio << std::hex <<  (unsigned int)buffer[i] << ".";
				i++;
			} else {
				if (byte < 0 && err != EAGAIN && err != EWOULDBLOCK) {
					return -1;
				}
				usleep(20);
			}

		} else {
			usleep(20);
		}

		if (stopwatch.is_timeout(2000)) {
			return -1;
		}
	}
	return 0;
}


int Serial::receive_tty(int handle, char *buffer, int len) {
    struct timeval timeout;		
    fd_set readfds;				
    int code, err, nBytes = 0;

    stopwatch.start_timer();

    timeout.tv_sec = 0;			
    timeout.tv_usec = 500000;	
    do {

	FD_ZERO(&readfds);		
	FD_SET(handle, &readfds);	
	if(select(handle+1, &readfds, NULL, NULL, &timeout)>0) {
		code = read(handle, &buffer[nBytes], len-nBytes);
         	err = errno;
         	if(code < 0) {
            		if(err != EWOULDBLOCK && err != EAGAIN) {
				return -1;
            		}
			usleep(20);									
         	} else {

            		if (code>0) {   
            			std::stringstream  txt;
                		txt << "recibir: [";
                		for(int i=0; i<code; i++) {
                			txt << std::hex <<  (unsigned int)buffer[nBytes+i] << " ";
               		 	}
            			txt << "]";
            		}
        	 	nBytes +=code;
         	}

         	if (stopwatch.is_timeout(2000)) {                             
               		return -1;
         	}

	} else {	
		break;
	}

         	
    } while (nBytes < len);                                         
    return nBytes;
}

int Serial::rx_tty(int handle, char *buffer, bool is_arm, char stop_char) {
    struct timeval timeout;
    fd_set readfds;
    int code, err, nBytes = 0;

    stopwatch.start_timer();
	
    if (is_arm) {
	    timeout.tv_sec = 2;
	    timeout.tv_usec = 0;
    } else {
	    timeout.tv_sec = 0;
	    timeout.tv_usec = 50000;
    }
    
    do {

	FD_ZERO(&readfds);
	FD_SET(handle, &readfds);

	if (select(handle+1, &readfds, NULL, NULL, &timeout)>0) {
		code = read(handle, &buffer[nBytes], 1);
         	err = errno;
         	if(code < 0) {
            		if(err != EWOULDBLOCK && err != EAGAIN) {
				return -1;
            		}
			usleep(20);									
         	} else {
			
            		if(code>0) {
  				if (buffer[nBytes] == '\0')
					break;
 
            			std::stringstream  txt;
                		txt << "rx: [";
                		for(int i=0; i<code; i++) {
                			txt << std::hex <<  (unsigned int)buffer[nBytes+i] << " ";
               		 	}
            			txt << "]";
            		}
  
        	 	nBytes++;        	 
         	}

        									
         	if (stopwatch.is_timeout(2000)) {                               
               		return -1;
         	}

	} else {
		break;
	}

         	
    } while (select(handle+1, &readfds, NULL, NULL, &timeout)>0);                                        

    return nBytes;
}


int Serial::receive_alt_tty(int handle, char* buffer, int len, int tics){
	struct timeval timeout;
	fd_set readfds;
	char ch;
	int total_bytes;
	int bytes_read;
	bool one_byte_read;
	std::stringstream ss;
	one_byte_read = false;
	bytes_read = 0;
	total_bytes = 0;

	timeout.tv_sec = 0;
	timeout.tv_usec = tics * 30000;

	do {
		FD_ZERO(&readfds);
		FD_SET(handle, &readfds);

		if(select(handle+1, &readfds, NULL, NULL, &timeout)>0){

			bytes_read = read(handle, &ch, 1);
			if(bytes_read==1){
				one_byte_read = true;
				timeout.tv_sec = 0;
				timeout.tv_usec = 500;
				total_bytes++;
				if((total_bytes%len)==0)
					buffer[len-1]=ch;
				else
					buffer[(total_bytes%len)-1]=ch;
			} else {
				return -1;
			}
		} else {
			if(!(total_bytes % len) || one_byte_read)
					break;
		}

	} while(total_bytes != 0);

	ss << "recibir2. read:";

	if((total_bytes>=len) && ((total_bytes%len)==0) ){
		for(int i=0;i<len;i++){
			ss << " ";
			ss << std::hex << std::setw(2) << (int)(unsigned char)buffer[i];
		}
		return len;

	} else {
		if(total_bytes){
			for(int i=0;i<total_bytes;i++){
				ss << " ";
				ss << std::hex << std::setw(2) << (int)(unsigned char)buffer[i];
			}
		}
		return total_bytes%len;
	}
}

int Serial::baud_tty(int handle, int baud)
{
	struct termios toptions;
	int tcbaud;

	switch (baud) {
		case 300:
			tcbaud = B300;
			break;
		case 1200:
			tcbaud = B1200;
			break;
		case 2400:
			tcbaud = B2400;
			break;
		case 4800:
			tcbaud = B4800;
			break;
		case 9600:
			tcbaud = B9600;
			break;
		case 19200:
			tcbaud = B19200;
			break;
		case 38400:
			tcbaud = B38400;
			break;
		default:
			tcbaud = B9600;
			break;
	}

	tcgetattr(handle, &toptions);
	cfsetispeed(&toptions, tcbaud);
	cfsetospeed(&toptions, tcbaud);
	tcsetattr(handle, TCSANOW, &toptions);

	return 0;
}


int Serial::lChar_tty(int handle, int lchar)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);


	toptions.c_cflag &= ~CSIZE;

	switch (lchar)
	{
		case 5:
			toptions.c_cflag |= CS5;
			break;
		case 6:
			toptions.c_cflag |= CS6;
			break;
		case 7:
			toptions.c_cflag |= CS7;
			break;
		case 8:
			toptions.c_cflag |= CS8;
			break;
		default:
			toptions.c_cflag |= CS8;
	}

	tcsetattr(handle, TCSANOW, &toptions);

	return 0;
}

int Serial::parity_tty(int handle, int paridad)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);
	switch (paridad)
	{
		case SINPARIDAD:
			toptions.c_cflag &= ~PARENB;
			toptions.c_iflag &= ~INPCK;
			break;
		case PAR:
			toptions.c_cflag |= PARENB;
			toptions.c_cflag &= ~PARODD;
			toptions.c_iflag &= ~IGNPAR;
			toptions.c_iflag |= INPCK; /* removed "|ISTRIP" */
			break;
		case IMPAR:
			toptions.c_cflag |= PARENB;
			toptions.c_cflag |= PARODD;
			toptions.c_iflag &= ~IGNPAR;
			toptions.c_iflag |= INPCK; /* removed "|ISTRIP" */
			break;
		default:
			toptions.c_cflag |= PARENB;
			toptions.c_cflag |= PARODD;
			toptions.c_iflag &= ~IGNPAR;
			toptions.c_iflag |= INPCK; /* removed "|ISTRIP" */
			break;
	}

	tcsetattr(handle, TCSANOW, &toptions);

	return 0;
}


int Serial::stopBits_tty(int handle, int stopBits)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);
	switch (stopBits) {
		case 1:
			toptions.c_cflag &= ~CSTOPB;
			break;
		case 2:
			toptions.c_cflag |= CSTOPB;
			break;
		default:
			toptions.c_cflag &= ~CSTOPB;
			break;
	}

	tcsetattr(handle, TCSANOW, &toptions);
	return 0;
}


int Serial::echo_tty(int handle)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);
	toptions.c_lflag |= ECHO | ECHOE | ECHOK;
	tcsetattr(handle, TCSANOW, &toptions);

	return 0;
}

int Serial::noEcho_tty(int handle)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);
	toptions.c_lflag &= ~ECHO & ~ECHOE & ~ECHOK;
	tcsetattr(handle, TCSANOW, &toptions);

	return 0;
}




int Serial::nDelay_tty(int handle, int setmode)
{
	int mode;

	mode = fcntl(handle, F_GETFL, 0);

	if(setmode == 1)
		mode |= O_NDELAY; /* Activar no bloqueante */
	else
		mode &= ~O_NDELAY; /* Desactivar no bloqueante */

	if (fcntl(handle, F_SETFL, mode) != 0) {
		return -1;
	}

	return 0;
}


int Serial::hangUp_tty(int handle)
{
	struct termios toptions;

	drain_tty(handle);
	tcgetattr(handle, &toptions);
	cfsetispeed(&toptions, 0);
	cfsetospeed(&toptions, B0);
	tcsetattr(handle, TCSADRAIN, &toptions);

	return 0;
}


int Serial::setup_tty(int handle)
{
	static struct termios toptions;

	toptions.c_iflag = IGNPAR | ISTRIP | ICRNL | IXON | IXANY; /* iflag */
	toptions.c_oflag = OPOST | ONLCR | TAB3; /* oflag */
	toptions.c_cflag = CS8 | CREAD | CLOCAL | HUPCL | B300; /* cflag */
	toptions.c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHOK; /* lflag */
	toptions.c_cc[0] = 3;
	toptions.c_cc[1] = 28;
	toptions.c_cc[2] = 127;
	toptions.c_cc[3] = 24;
	toptions.c_cc[4] = 4;
	toptions.c_cc[5] = 0;
	toptions.c_cc[6] = 0;
	toptions.c_cc[7] = 0;

	tcsetattr(handle, TCSANOW, &toptions);

	return 0;
}


int Serial::sane_tty(int handle)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);
	toptions.c_iflag |= IGNPAR | ISTRIP | ICRNL | IXON | IXANY;
	toptions.c_oflag |= OPOST | ONLCR;
	toptions.c_lflag |= ISIG | ICANON;
	toptions.c_cc[VEOF] = 4;
	tcsetattr(handle, TCSANOW, &toptions);
	echo_tty(handle);

	return 0;
}


int Serial::raw_tty(int handle)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);
	cfmakeraw(&toptions);
	tcsetattr(handle, TCSANOW, &toptions);

	return 0;

}


int Serial::local_tty(int handle, int setmode)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);

	if(setmode == 1)
		toptions.c_cflag |= (CLOCAL | CREAD);
	else
		toptions.c_cflag &= ~CLOCAL;

	tcsetattr(handle, TCSANOW, &toptions);
	return 0;
}


int Serial::flushIn_tty(int handle) {

	/* Vaciar el buffer serie */
	if (tcflush(handle, TCIFLUSH) < 0) {
		return -1;
	} else {
		return 0;
	}
}

int Serial::flushOut_tty(int handle) {

	/* Vaciar el buffer serie */
	if (tcflush(handle, TCOFLUSH) < 0) {
		return -1;
	} else {
		return 0;
	}
}


int Serial::nbytes_tty(int handle) {
	int numbytes;

	if (ioctl(handle, TIOCOUTQ, &numbytes) < 0) {
		return -1;
	}
	return numbytes;
}


int Serial::flowcontrol_type_tty(int handle, int flag)
{
	struct termios toptions;

	tcgetattr(handle, &toptions);

	if (flag) {
		toptions.c_iflag &= ~(IXON | IXOFF | IXANY); /* Control flujo por hw */
		toptions.c_cflag |= CRTSCTS;
	} else {
		toptions.c_cflag &= ~CRTSCTS;
		toptions.c_iflag |= (IXON | IXOFF | IXANY); /* Control flujo por sw */
	}

	tcsetattr(handle, TCSANOW, &toptions);
	noEcho_tty(handle);

	return 0;
}


int Serial::setRTS_tty(int handle, int setmode) {
	int status;

	if (ioctl(handle, TIOCMGET, &status) == -1) {
		return -1;
	}

	if (setmode == 1) {
		status |= TIOCM_RTS;  //turn on RTS
	} else if (setmode == 0) {
		status &= ~TIOCM_RTS;  //turn off RTS
	}

	if (ioctl(handle, TIOCMSET, &status) == -1) {
		return -1;
	}
	return 0;

}

int Serial::setCTS_tty(int handle, int setmode) {
	int status;

	if (ioctl(handle, TIOCMGET, &status) == -1) {
		return -1;
	}

	if (setmode == 1) {
		status |= TIOCM_CTS;  //turn on CTS
	} else if (setmode == 0) {
		status &= ~TIOCM_CTS;  //turn off CTS
	}

	if (ioctl(handle, TIOCMSET, &status) == -1) {
		return -1;
	}
	return 0;

}


int Serial::setDTR_tty(int handle, int setmode) {
	int status;

	if (ioctl(handle, TIOCMGET, &status) == -1) {
		return -1;
	}

	if (setmode == 1) {
		status |= TIOCM_DTR;  //turn on DTR
	} else if (setmode == 0) {
		status &= ~TIOCM_DTR;  //turn off DTR
	}

	if (ioctl(handle, TIOCMSET, &status) == -1) {
		return -1;
	}
	return 0;

}

int Serial::setXON_tty(int handle) {
	char xon[] = "\x11";

	if (send_tty(handle, xon, 1) < 0) {
		return -1;
	}
	return 0;	

}

int Serial::setXOFF_tty(int handle) {
	char xoff[] = "\x13";

	if (send_tty(handle, xoff, sizeof(xoff)-1) < 0) {
		return -1;
	}
	return 0;
}

int Serial::getRTS_tty(int handle) {
	int status;

	ioctl(handle, TIOCMGET, &status);

	if (status & TIOCM_RTS)
		return 1;
	else
		return 0;
}


int Serial::getCTS_tty(int handle) {
	int status;

	ioctl(handle, TIOCMGET, &status);

	if (status & TIOCM_CTS)
		return 1;
	else
		return 0;
}


int Serial::getDTR_tty(int handle) {
	int status;

	ioctl(handle, TIOCMGET, &status);

	if (status & TIOCM_DTR)
		return 1;
	else
		return 0;
}


int Serial::getDSR_tty(int handle) {
	int status;

	ioctl(handle, TIOCMGET, &status);

	if (status & TIOCM_DSR)
		return 1;
	else
		return 0;
}


int Serial::getRNG_tty(int handle) {
	int status;

	ioctl(handle, TIOCMGET, &status);

	if (status & TIOCM_RNG)
		return 1;
	else
		return 0;
}

int Serial::getCD_tty(int handle)
{
	unsigned int modem;

	if (ioctl(handle, TIOCMGET, &modem) < 0) {
		return -1;
	} else {
		if((modem & TIOCM_CD) | (modem & TIOCM_CAR))
			return 1;
		else
			return 0;
	}
}

int Serial::drain_tty(int handle)
{
	sleep(1);
	tcdrain(handle);
	return 0;
}


int Serial::sendBreak_tty(int handle)
{
	tcsendbreak(handle, 150);
	return 0;
}

