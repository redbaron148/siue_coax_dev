/*************************************************************
*
* Low-level sensor and control library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by:
*  * Samir Bouabdallah samir.bouabdallah@skybotix.ch
*  * Cedric Pradalier: cedric.pradalier@skybotix.ch
*  * Gilles Caprari: gilles.capraru@skybotix.ch
*  Inspired from the e-puck communication code
* Send modification or corrections as patches (diff -Naur)
* Copyright: Skybotix AG, 2009-2012
* 
* All rights reserved.
* 
* Skybotix API is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* Skybotix API is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.
* 
* 
* Description: This software provides the basic UART comm functions
*************************************************************/
#ifndef _COAX_UART
#define _COAX_UART

// Interrupt counter
extern short U1RXCount;
extern short U1TXCount;
extern short U2RXCount;
extern short U2TXCount;

void e_init_uart1(void);
void e_init_uart2(void);

/*! \brief Check if something is comming on uart 1
 * \return the number of characters available, 0 if none are available */
int  e_ischar_uart1();

/*! \brief If available, read 1 char and put it in pointer
 * \param car The pointer where the caracter will be stored if available
 * \return 1 if a char has been readed, 0 if no char is available
 */
int  e_getchar_uart1(char *car);

/*! \brief Send a buffer of char of size length
 * \param buff The top of the array where the data are stored
 * \param length The length of the array to send
 */
void e_send_uart1_char(const char * buff, int length);

/*! \brief  To check if the sending operation is done
 * \return 1 if buffer sending is in progress, return 0 if not
 */
int  e_uart1_sending(void);

/*! \brief Check if something is comming on uart 2
 * \return the number of characters available, 0 if none are available */
int  e_ischar_uart2();

/*! \brief If available, read 1 char and put it in pointer
 * \param car The pointer where the caracter will be stored if available
 * \return 1 if a char has been readed, 0 if no char is available
 */
int  e_getchar_uart2(char *car);

/*! \brief Send a buffer of char of size length
 * \param buff The top of the array where the datas are stored
 * \param length The length of the array
 */
void e_send_uart2_char(const char * buff, int length);

/*! \brief  To check if the sending operation is done
 * \return 1 if buffer sending is in progress, return 0 if not
 */
int  e_uart2_sending(void);


extern void *e_uart1_int_clr_addr; //address to be clear on interrupt
extern int e_uart1_int_clr_mask; //mask to be use to clear on interrupt
extern void *e_uart2_int_clr_addr; //address to be clear on interrupt
extern int e_uart2_int_clr_mask; //mask to be use to clear on interrupt

/************* Functions Prototypes **********/ 

void init_UART(void);
void init_UART1(void);
void init_UART2(void);

/*************************************************************************
* Function Name     : BusyUART1                                          *
* Description       : This returns status whether the transmission       *
*                     is in progress or not, by checking Status bit TRMT *
* Parameters        : None                                               *
* Return Value      : char info whether transmission is in progress      *
*************************************************************************/
char BusyUART1(void);

/*************************************************************************
* Function Name     : BusyUART2                                          *
* Description       : This returns status whether the transmission       *  
*                     is in progress or not, by checking Status bit TRMT *
* Parameters        : None                                               *
* Return Value      : char info whether transmission is in progress      *
*************************************************************************/
char BusyUART2(void);

/*********************************************************************
* Function Name     : CloseUART1                                     *
* Description       : This function disables the UART and clears the *
*                     Interrupt enable & flag bits                   *  
* Parameters        : None                                           *
* Return Value      : None                                           *  
*********************************************************************/
void CloseUART1(void);

/*********************************************************************
* Function Name     : CloseUART2                                     *
* Description       : This function disables the UART and clears the *
*                     interrupt enable and flag bits                 *
* Parameters        : None                                           *
* Return Value      : None                                           *
*********************************************************************/
void CloseUART2(void);

/**********************************************************************
* Function Name     : ConfigIntUART1                                  *
* Description       : This function sets priority for RX,TX interrupt * 
*                     and enable/disables the interrupt               *
* Parameters        : unsigned int config enable/disable and priority *
* Return Value      : None                                            *  
**********************************************************************/
void ConfigIntUART1(unsigned int config);

/**********************************************************************
* Function Name     : ConfigIntUART2                                  *
* Description       : This function sets priority for  RX and TX      *
*                     interrupt and enable/disables the interrupt     *  
* Parameters        : unsigned int config enable/disable and priority *
* Return Value      : None                                            *
**********************************************************************/
void ConfigIntUART2(unsigned int config);

/*********************************************************************
* Function Name     : DataRdyUart1                                   *
* Description       : This function checks whether there is any data *
*                     that can be read from the input buffer, by     *
*                     checking URXDA bit                             *
* Parameters        : None                                           *
* Return Value      : char if any data available in buffer           *
*********************************************************************/
char DataRdyUART1(void);

/*********************************************************************
* Function Name     : DataRdyUart2                                   *
* Description       : This function checks whether there is any data *
*                     that can be read from the input buffer, by     *
*                     checking URXDA bit                             *
* Parameters        : None                                           *
* Return Value      : char if any data available in buffer           *
*********************************************************************/
char DataRdyUART2(void);

/******************************************************************************
* Function Name     : getsUART1                                               *
* Description       : This function gets a string of data of specified length * 
*                     if available in the UxRXREG buffer into the buffer      *
*                     specified.                                              *
* Parameters        : unsigned int length the length expected                 *
*                     unsigned int *buffer  the received data to be           * 
*                                  recorded to this array                     *
*                     unsigned int uart_data_wait timeout value               *
* Return Value      : unsigned int number of data bytes yet to be received    *
******************************************************************************/
unsigned int getsUART1(unsigned int length,unsigned int *buffer,
                       unsigned int uart_data_wait);

/******************************************************************************
* Function Name     : getsUART2                                               *
* Description       : This function gets a string of data of specified length * 
*                     if available in the UxRXREG buffer into the buffer      *
*                     specified.                                              *
* Parameters        : unsigned int length the length expected                 *
*                     unsigned int *buffer  the received data to be           * 
*                                  recorded to this array                     *
*                     unsigned int uart_data_wait timeout value               *
* Return Value      : unsigned int number of data bytes yet to be received    * 
******************************************************************************/
unsigned int getsUART2(unsigned int length,unsigned int *buffer,
                       unsigned int uart_data_wait);


/*********************************************************************
* Function Name     : OpenUART1                                      *
* Description       : This function configures the UART mode,        * 
*                     UART Interrupt modes and the Baud Rate         *
* Parameters        : unsigned int config1 operation setting         *
*                     unsigned int config2 TX & RX interrupt modes   *
*                     unsigned int ubrg baud rate setting            *
* Return Value      : None                                           *
*********************************************************************/
void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg);

/*********************************************************************
* Function Name     : OpenUART2                                      *
* Description       : This function configures the UART mode,        *
*                     UART Interrupt modes and the Baud Rate         *
* Parameters        : unsigned int config1 operation setting         *
*                     unsigned int config2 TX & RX interrupt modes   *
*                     unsigned int ubrg baud rate setting            *
* Return Value      : None                                           *      
*********************************************************************/
void OpenUART2(unsigned int config1,unsigned int config2, unsigned int ubrg);

/***************************************************************************
* Function Name     : writeUART1                                           *
* Description       : This function puts the data string to be transmitted *
*                     into the transmit buffer                             *
* Parameters        : unsigned char * address of the string buffer to be   *  
*                     transmitted                                          *
* Return Value      : None                                                 *
***************************************************************************/
void writeUART1(unsigned char *buffer, unsigned int size);

/***************************************************************************
* Function Name     : writeUART2                                           *
* Description       : This function puts the data string to be transmitted *
*                     into the transmit buffer                             *
* Parameters        : unsigned char * address of the string buffer to be   *  
*                     transmitted                                          *
* Return Value      : None                                                 *
***************************************************************************/
void writeUART2(unsigned char *buffer, unsigned int size);

/***************************************************************************
* Function Name     : putsUART1                                            *
* Description       : This function puts the data string to be transmitted *
*                     into the transmit buffer (till NULL character)       *
* Parameters        : unsigned int * address of the string buffer to be    *  
*                     transmitted                                          *
* Return Value      : None                                                 *
***************************************************************************/
void putsUART1(unsigned int *buffer);

/***************************************************************************
* Function Name     : putsUART2                                            *
* Description       : This function puts the data string to be transmitted *
*                     into the transmit buffer (till NULL character)       * 
* Parameters        : unsigned int * address of the string buffer to be    *
*                     transmitted                                          *
* Return Value      : None                                                 *  
***************************************************************************/
void putsUART2(unsigned int *buffer);

/***************************************************************************
* Function Name     : ReadUART1                                            *
* Description       : This function returns the contents of UxRXREG buffer * 
* Parameters        : None                                                 *
* Return Value      : unsigned int value from UxRXREG receive buffer       *
***************************************************************************/
unsigned int ReadUART1(void);

/***************************************************************************
* Function Name     : ReadUART2                                            *
* Description       : This function returns the contents of UxRXREG buffer *
* Parameters        : None                                                 *  
* Return Value      : unsigned int value from UxRXREG receive buffer       * 
***************************************************************************/
unsigned int ReadUART2(void);

/*********************************************************************
* Function Name     : WriteUART1                                     *
* Description       : This function writes data into the UxTXREG,    *
* Parameters        : unsigned int data the data to be written       *
* Return Value      : None                                           *
*********************************************************************/
void WriteUART1(unsigned int data);

/*********************************************************************
* Function Name     : WriteUART2                                     *
* Description       : This function writes data into the UxTXREG,    *
* Parameters        : unsigned int data the data to be written       *
* Return Value      : None                                           *
*********************************************************************/
void WriteUART2(unsigned int data);

#endif
