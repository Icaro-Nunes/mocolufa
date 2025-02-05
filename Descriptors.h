/*
	Firmware to set the atmega16u2 on
	arduino UNO R3 as dual configurable,
	as serial interface for arduino and
	also as USB Audio Device interface
	
	based on dualMocoLUFA 2013/09/22
*/
/*
     dualMocoLUFA Project
     Copyright (C) 2013 by morecat_lab

     2013/09/22
              
     http://morecatlab.akiba.coocan.jp/

     based on LUFA-100807
*/
/*
             LUFA Library
     Copyright (C) Dean Camera, 2021.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2021  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for Descriptors.c.
 */

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

	/* Includes: */
		#include <avr/pgmspace.h>

		#include <LUFA/Drivers/USB/USB.h>
		#include <LUFA/Drivers/USB/Class/CDCClass.h>
		#include <LUFA/Drivers/USB/Class/MIDIClass.h>
		
	/* Product-specific definitions: */
		#define ARDUINO_UNO_PID			0x0001
		#define ARDUINO_MEGA2560_PID		0x0010
		#define ATMEL_LUFA_DEMO_PID		0x204B

	/* Macros for CDC */
		/** Endpoint number of the CDC device-to-host notification IN endpoint. */
		#define CDC_NOTIFICATION_EPNUM         2
		/** Endpoint number of the CDC device-to-host data IN endpoint. */
		#define CDC_TX_EPNUM                   3	
		/** Endpoint number of the CDC host-to-device data OUT endpoint. */
		#define CDC_RX_EPNUM                   4	
		/** Size in bytes of the CDC device-to-host notification IN endpoint. */
		#define CDC_NOTIFICATION_EPSIZE        8
		/** Size in bytes of the CDC data IN and OUT endpoints. */
		#define CDC_TXRX_EPSIZE                64	

	/* Macros for AUDIO */
		#define AUDIO_ENDPOINT_NUMBER          1
		#define AUDIO_ENDPOINT_ADDRESS         (ENDPOINT_DIR_OUT | AUDIO_ENDPOINT_NUMBER)
		#define AUDIO_ENDPOINT_SIZE            8

	/* Type Defines for CDC */
		/** Type define for the device configuration descriptor structure. This must be defined in the
		 *  application code, as the configuration descriptor contains several sub-descriptors which
		 *  vary between devices, and which describe the device's usage to the host.
		 */
		typedef struct
		{
			USB_Descriptor_Configuration_Header_t    Config;
			USB_Descriptor_Interface_t               CDC_CCI_Interface;
			USB_CDC_Descriptor_FunctionalHeader_t    CDC_Functional_IntHeader;
			USB_CDC_Descriptor_FunctionalACM_t       CDC_Functional_AbstractControlManagement;
			USB_CDC_Descriptor_FunctionalUnion_t     CDC_Functional_Union;
			USB_Descriptor_Endpoint_t                CDC_NotificationEndpoint;
			USB_Descriptor_Interface_t               CDC_DCI_Interface;
			USB_Descriptor_Endpoint_t                CDC_DataOutEndpoint;
			USB_Descriptor_Endpoint_t                CDC_DataInEndpoint;
		} USB_Descriptor_ConfigurationCDC_t;

	/* Type Defines for AUDIO */
		typedef struct
		{
			// General Configuration Descriptor
			USB_Descriptor_Configuration_Header_t        Config;

			// Audio Control Interface
			USB_Descriptor_Interface_t                   Audio_ControlInterface;
			USB_Audio_Descriptor_Interface_AC_t          Audio_ControlInterface_SPC;
			USB_Audio_Descriptor_InputTerminal_t         Input_Terminal;
			USB_Audio_Descriptor_OutputTerminal_t        Output_Terminal;

			// Audio Streaming Interface
			USB_Descriptor_Interface_t                   Audio_StreamInterface;
			USB_Audio_Descriptor_Interface_AS_t          Audio_StreamInterface_SPC;
			USB_Audio_Descriptor_Format_t                Audio_Format;
			USB_Audio_SampleFreq_t                       Sample_Freq;
			USB_Audio_Descriptor_StreamEndpoint_Std_t    Audio_Endpoint;
			USB_Audio_Descriptor_StreamEndpoint_Spc_t    Audio_Endpoint_SPC;
		} USB_Descriptor_ConfigurationAUDIO_t;
		
	/* Function Prototypes: */
		// uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
		//                                     const uint8_t wIndex,
		//                                     const void** const DescriptorAddress) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);
#endif
