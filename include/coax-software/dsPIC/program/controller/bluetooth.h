/********************************************************************************

			Bluetooth for e-puck
			Version 1.0 July 2006 Michael Bonani


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2006-2007 Michael Bonani

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup bluetooth
 * \brief Manage Bluetooth.
 *
 * This module manage the bluetooth device.
 * \author Code: Michael Bonani \n Doc: Jonathan Besuchet
 */

/*! \defgroup bluetooth Bluetooth
 * 
 * \section intro_sec Introduction
 * This package contains all the ressources you need to control the bluetooth
 * device you have on your e-puck AS MASTER. If you only want to use the bluetooth
 * AS SLAVE, the uart library is enough (the connection to a master look like a 
 * standard uart connection).
 * \n The bluetooth device is connected on the uart2.
 * \n \n Generally, the bluetooth modules have a bluetooth class device which identify
 * them as PC, mobile, mouse, ... The e-puck bluetooth module has the default
 * device class number, it's 000.
 * \n \n To learn more about the e-puck's bluetooth device see the documentation of the
 * LMX9820A from National Semiconductor (look at
 * http://www.national.com/mpf/LM/LMX9820A.html) and look at the \ref uart module.
 *
 * \author Doc: Jonathan Besuchet
 */

#ifndef _BLUETOOTH
#define _BLUETOOTH

/*------ low level bluettoth fuction ------*/

char e_bt_write_local_pin_number(const char PIN[4]); //give 1 to 16 number PIN (in general e-puck have 4 numbers PIN)
char e_bt_write_local_name(const char name[16]);//write friendly name

#endif
