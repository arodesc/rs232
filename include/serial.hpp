#ifndef SERIAL_HPP
#define SERIAL_HPP

#include "stopwatch.hpp"

#include <string>


#define LONGDEVICE		15
#define BUFFER_SERIAL		256

#define SINPARIDAD    		0
#define PAR           		1
#define IMPAR         		2

#define BUF_SIZE 		1024

#define SOFTWARE		0
#define HARDWARE		1

class Serial {
	public:

		/**
		 * \brief	abre el puerto serie al que está conectada la impresora
		 * \param	baud: velocidad en baudios de la puerta
		 * \return	descriptor del puerto, si lo abre correctamente
		 * 			-1 en caso contrario
		 */
		int open_tty(const std::string filename, int baud, int parity= 0, int lchar = 8, int stopbits = 1);

		int open_alt_tty(const std::string filename, int baud, int parity = 0, int lchar = 8, int stopbits = 1);



		/**
		 * \brief	cierra el puerto al que está conectada la impresora
		 * \param	handle: descriptor del puerto serie
		 * \return	0 si la salida es correcta
		 * 			-1 en caso contrario
		 */
		int close_tty(int handle);



		/**
		 * \brief	envio de buffer de escritura
		 * \param	handle: descriptor del puerto serie
		 * 			buffer: para la escritura en el puerto serie
		 * 			len: longitud de la cadena
		 * 			control: desactiva el control de flujo
		 * 				-true: desactiva control de flujo
		 * 				-false: activa control de flujo (por defecto)
		 * \return	0 si la salida es correcta
		 * 			-1 en caso contrario
		 */
		int send_tty(int handle, char *buffer, int len, bool control = false);


		/**
		 * \brief	recepción de buffer de lectura
		 * \param	handle: descriptor del puerto serie
		 * 			buffer: para lectura
		 * \return	nº caracteres leídos
		 * 			-1 si no se ha leído nada
		 */
		int receive_tty(int handle, char *buffer, int len);
		int rx_tty(int handle, char *buffer, bool is_arm = false, char stop_char = '\0');		
		int receive_alt_tty(int handle, char* buffer, int len, int tics);

		int baud_tty(int handle, int baud);
		int lChar_tty(int handle, int lchar);
		int parity_tty(int handle, int paridad);
		int stopBits_tty(int handle, int stopBits);
		int echo_tty(int handle);
		int noEcho_tty(int handle);
		int nDelay_tty(int handle, int setmode);
		int hangUp_tty(int handle);
		int setup_tty(int handle);
		int sane_tty(int handle);
		int raw_tty(int handle);
		int local_tty(int handle, int setmode);


		/**
		 * \brief	vaciar buffer de entrada del puerto serie
		 * \param	handle: descriptor del puerto serie
		 * \return	0 si el resultado es correcto
		 * 			-1 en caso contrario
		 */
		int flushIn_tty(int handle);


		/**
		 * \brief	vaciar buffer de salida del puerto serie
		 * \param	handle: descriptor del puerto serie
		 * \return	0 si el resultado es correcto
		 * 			-1 en caso contrario
		 */
		int flushOut_tty(int handle);


		/**
		 * \brief	número de bytes en el buffer
		 * \param	handle: descriptor del puerto serie
		 * \return	número de bytes del buffer
		 * 			0 si el buffer está vacío
		 */
		int nbytes_tty(int handle);

		int flowcontrol_type_tty(int handle, int flag);


		/**
		 * \brief	manejo del modo RTS (Request to Send)
		 * \param	handle: descriptor del puerto serie
		 * 			setmode:	1 para activar el modo RTS
		 * 						0 para desactivar el modo RTS
		 * \return	0 si el resultado es correcto
		 * 			-1 en caso contrario
		 */
		int setRTS_tty(int handle, int setmode);

		int setCTS_tty(int handle, int setmode);
		int setDTR_tty(int handle, int setmode);
		int setXON_tty(int handle);
		int setXOFF_tty(int handle);


		int getRTS_tty(int handle);

		/**
		 * \brief	modo CTS (Clear to Send)
		 * \param	handle: descriptor del puerto serie
		 * \return	1 si Clear to Send es activo
		 * 			0 en caso contrario
		 */
		int getCTS_tty(int handle);

		int getDTR_tty(int handle);


		/**
		 * \brief	modo DSR (Data Send Ready)
		 * \param	handle: descriptor del puerto serie
		 * \return	1 si Data Send Ready es activo
		 * 			0 en caso contrario
		 */
		int getDSR_tty(int handle);

		int getRNG_tty(int handle);
		int getCD_tty(int handle);
		int drain_tty(int handle);
		int sendBreak_tty(int handle);

	private:
		StopWatch stopwatch;

};


#endif /* SERIAL_HPP */
