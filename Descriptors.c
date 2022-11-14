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
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.
 */

#include "Descriptors.h"
#include "dualMoco.h"

/* On some devices, there is a factory set internal serial number which can be automatically sent to the host as
 * the device's serial number when the Device Descriptor's .SerialNumStrIndex entry is set to USE_INTERNAL_SERIAL.
 * This allows the host to track a device across insertions on different ports, allowing them to retain allocated
 * resources like COM port numbers and drivers. On demos using this feature, give a warning on unsupported devices
 * so that the user can supply their own serial number descriptor instead or remove the USE_INTERNAL_SERIAL value
 * from the Device Descriptor (forcing the host to generate a serial number for each device from the VID, PID and
 * port location).
 */
#if (USE_INTERNAL_SERIAL == NO_DESCRIPTOR)
	#warning USE_INTERNAL_SERIAL is not available on this AVR - please manually construct a device serial descriptor.
#endif

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
/* for SERIAL */
const USB_Descriptor_Device_t PROGMEM DeviceDescriptorSerial =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
		
	.USBSpecification       = VERSION_BCD(1,10, 0),
	.Class                  = 0x02,
	.SubClass               = 0x00,
	.Protocol               = 0x00,
				
	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,
		
	.VendorID               = ARDUINO_VID, // VID

	.ProductID          	= ARDUINO_MODEL_PID, // PID
	.ReleaseNumber          = 0x0001,
		
	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = USE_INTERNAL_SERIAL,
		
	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/* for AUDIO */
const USB_Descriptor_Device_t PROGMEM DeviceDescriptorAUDIO =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
		
	.USBSpecification       = VERSION_BCD(1, 10, 0),
	.Class                  = 0x00,
	.SubClass               = 0x00,
	.Protocol               = 0x00,
				
	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,
		
	.VendorID               = 0x03EB, // Atmel
	.ProductID              = 0x2048, // LUFA USB-MIDI Demo application
	.ReleaseNumber          = 0x0000,
		
	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = NO_DESCRIPTOR,
		
	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
/* for Serial */
const USB_Descriptor_ConfigurationCDC_t PROGMEM ConfigurationDescriptorSerial =
{
	.Config = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize = sizeof(USB_Descriptor_ConfigurationCDC_t),
			.TotalInterfaces        = 2,
				
			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,
				
			.ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),
			
			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
		},
		
	.CDC_CCI_Interface = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = 0,
			.AlternateSetting       = 0,
			
			.TotalEndpoints         = 1,
				
			.Class                  = 0x02,
			.SubClass               = 0x02,
			.Protocol               = 0x01,
				
			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC_Functional_IntHeader = 
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = 0x24},
			.Subtype                = 0x00,
			
			.CDCSpecification       = (0x01 << 8) | 0x10
		},

	.CDC_Functional_AbstractControlManagement = 
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = 0x24},
			.Subtype                = 0x02,
			
			.Capabilities           = 0x06
		},
		
	.CDC_Functional_Union = 
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = 0x24},
			.Subtype                = 0x06,
			
			.MasterInterfaceNumber  = 0x00,
			.SlaveInterfaceNumber   = 0x01
		},

	.CDC_NotificationEndpoint = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DIR_IN | CDC_NOTIFICATION_EPNUM),
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_NOTIFICATION_EPSIZE,
			.PollingIntervalMS      = 0xFF
		},

	.CDC_DCI_Interface = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = 1,
			.AlternateSetting       = 0,
			
			.TotalEndpoints         = 2,
				
			.Class                  = 0x0A,
			.SubClass               = 0x00,
			.Protocol               = 0x00,
				
			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC_DataOutEndpoint = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DIR_OUT | CDC_RX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_TXRX_EPSIZE,
			.PollingIntervalMS      = 0x01
		},
		
	.CDC_DataInEndpoint = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DIR_IN | CDC_TX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_TXRX_EPSIZE,
			.PollingIntervalMS      = 0x01
		}
};

#define AC_CLASS_SPECIFIC_DESCRIPTOR_SIZE sizeof(USB_Audio_Descriptor_Interface_AC_t) + sizeof(USB_Audio_Descriptor_InputTerminal_t) + sizeof(USB_Audio_Descriptor_OutputTerminal_t)


/* for AUDIO */
const USB_Descriptor_ConfigurationAUDIO_t PROGMEM ConfigurationDescriptorAUDIO =
{
	.Config = 
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize   = sizeof(USB_Descriptor_ConfigurationAUDIO_t),
			.TotalInterfaces          = 2,

			.ConfigurationNumber      = 1,
			.ConfigurationStrIndex    = NO_DESCRIPTOR,
				
			.ConfigAttributes         = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),
			
			.MaxPowerConsumption      = USB_CONFIG_POWER_MA(100)
		},
		
	.Audio_ControlInterface = 
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = 0x00,
			.AlternateSetting         = 0,
			
			.TotalEndpoints           = 0,
				
			.Class                    = 0x01,
			.SubClass                 = 0x01,
			.Protocol                 = 0x00,
				
			.InterfaceStrIndex        = NO_DESCRIPTOR
		},
	
	.Audio_ControlInterface_SPC = 
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AC_t), .Type = AUDIO_DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_Header,
			
			.ACSpecification          = VERSION_BCD(01, 00, 0),
			.TotalLength              = AC_CLASS_SPECIFIC_DESCRIPTOR_SIZE,
			
			.InCollection             = 1,
			.InterfaceNumber          = 1,
		},
	
	.Input_Terminal = 
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_InputTerminal_t), .Type = AUDIO_DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_InputTerminal,
			.TerminalID               = 0x01,
			.TerminalType             = AUDIO_TERMINAL_STREAMING,
			.AssociatedOutputTerminal = 0x00,
			.TotalChannels            = 1,
			.ChannelConfig            = 0x00,
			.ChannelStrIndex          = 0x00,
			.TerminalStrIndex         = 0x00
		},

	.Output_Terminal = 
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_OutputTerminal_t), .Type = AUDIO_DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,
			.TerminalID               = 0x02,
			.TerminalType             = AUDIO_TERMINAL_OUT_SPEAKER,
			.AssociatedInputTerminal  = 0x00,
			.SourceID                 = 0x01,
			.TerminalStrIndex         = 0x00
		},

	.Audio_StreamInterface = 
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = 0x01,
			.AlternateSetting         = 0,
			
			.TotalEndpoints           = 1,
				
			.Class                    = 0x01,
			.SubClass                 = 0x02,
			.Protocol                 = 0x00,
				
			.InterfaceStrIndex        = NO_DESCRIPTOR
		},
		
	.Audio_StreamInterface_SPC = 
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AS_t), .Type = AUDIO_DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_General,

			.TerminalLink             = 0x01,
			
			.FrameDelay               = 1,
			.AudioFormat              = 0x01
		},
	
	.Audio_Format = 
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Format_t) + sizeof(USB_Audio_SampleFreq_t), .Type = AUDIO_DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_FormatType,
			.FormatType               = 0x01,
			.Channels                 = 1,
			.SubFrameSize             = 1,
			.BitResolution            = 0x08,
			.TotalDiscreteSampleRates = 1
		},

	.Sample_Freq = AUDIO_SAMPLE_FREQ(8000),

	.Audio_Endpoint = 
		{
			.Endpoint                 = 
				{
					.Header           = {.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Std_t), .Type = DTYPE_Endpoint},
					.EndpointAddress  = AUDIO_ENDPOINT_ADDRESS,
					.Attributes       = (ENDPOINT_ATTR_NO_SYNC | EP_TYPE_ISOCHRONOUS),
					.EndpointSize     = 1,
					.PollingIntervalMS= 1
				},
			.Refresh                  = 0,
			.SyncEndpointNumber       = 0
		},

	.Audio_Endpoint_SPC = 
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Spc_t), .Type = AUDIO_DTYPE_CSEndpoint},
			.Subtype                  = AUDIO_DSUBTYPE_CSEndpoint_General,
			.Attributes               = 0x00,
			.LockDelayUnits           = 0x00,
			.LockDelay                = 0x0000,
		}
	
	
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
const USB_Descriptor_String_t PROGMEM LanguageString =
{
	.Header                 = {.Size = USB_STRING_LEN(1), .Type = DTYPE_String},
		
	.UnicodeString          = {LANGUAGE_ID_ENG}
};

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
/* for Serial */
const USB_Descriptor_String_t PROGMEM ManufacturerStringSerial =
{
	.Header                 = {.Size = USB_STRING_LEN(24), .Type = DTYPE_String},
		
	.UnicodeString          = L"Arduino (www.arduino.cc)"
};

/* for AUDIO */
const USB_Descriptor_String_t PROGMEM ManufacturerStringAUDIO =
{
	.Header                 = {.Size = USB_STRING_LEN(24), .Type = DTYPE_String},

	.UnicodeString          = L"icarolimanunes@gmail.com"
};
/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
/* for Serial */
const USB_Descriptor_String_t PROGMEM ProductStringSerial =
{
	#if (ARDUINO_MODEL_PID == ARDUINO_UNO_PID)
		.Header                 = {.Size = USB_STRING_LEN(11), .Type = DTYPE_String},
			
		.UnicodeString          = L"Arduino Uno"
	#elif (ARDUINO_MODEL_PID == ARDUINO_MEGA2560_PID)
		.Header                 = {.Size = USB_STRING_LEN(17), .Type = DTYPE_String},
			
		.UnicodeString          = L"Arduino Mega 2560"
	#elif (ARDUINO_MODEL_PID == ATMEL_LUFA_DEMO_PID)
		.Header                 = {.Size = USB_STRING_LEN(14), .Type = DTYPE_String},
			
		.UnicodeString          = L"Lufa USBSerial"
	#endif
	
};
/* for AUDIO */
const USB_Descriptor_String_t PROGMEM ProductStringAUDIO =
{
	.Header                 = {.Size = USB_STRING_LEN(21), .Type = DTYPE_String},

	.UnicodeString          = L"MocoLUFA AUDIO Device"
};

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	void*    Address = NULL;
	uint16_t Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
		case DTYPE_Device: 
		  if (mocoMode == 1) {
			Address = (void*)&DeviceDescriptorAUDIO;
			Size    = sizeof(USB_Descriptor_Device_t);
		  } else {
			Address = (void*)&DeviceDescriptorSerial;
			Size    = sizeof(USB_Descriptor_Device_t);
		  }
			break;
		case DTYPE_Configuration: 
		  if (mocoMode == 1) {
			Address = (void*)&ConfigurationDescriptorAUDIO;
			Size    = sizeof(USB_Descriptor_ConfigurationAUDIO_t);
		  } else {
			Address = (void*)&ConfigurationDescriptorSerial;
			Size    = sizeof(USB_Descriptor_ConfigurationCDC_t);
		  }
			break;
		case DTYPE_String: 
			switch (DescriptorNumber)
			{
				case 0x00:
					Address = (void*)&LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case 0x01:
				  if (mocoMode == 1) {
					Address = (void*)&ManufacturerStringAUDIO;
					Size    = pgm_read_byte(&ManufacturerStringAUDIO.Header.Size);
				  } else {
					Address = (void*)&ManufacturerStringSerial;
					Size    = pgm_read_byte(&ManufacturerStringSerial.Header.Size);
				  }
					break;
				case 0x02:
				  if (mocoMode == 1) {
					Address = (void*)&ProductStringAUDIO;
					Size    = pgm_read_byte(&ProductStringAUDIO.Header.Size);
				  } else {
					Address = (void*)&ProductStringSerial;
					Size    = pgm_read_byte(&ProductStringSerial.Header.Size);
				  }
					break;
			}
			
			break;
	}
	
	*DescriptorAddress = Address;
	return Size;
}
