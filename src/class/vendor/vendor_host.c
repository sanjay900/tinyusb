/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if (TUSB_OPT_HOST_ENABLED && CFG_TUH_VENDOR)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "host/usbh.h"
#include "vendor_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

typedef struct {
  uint8_t  itf_numr;
  uint8_t  ep_in;
  uint8_t  ep_out;

  uint16_t report_size;
} cush_interface_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
cush_interface_t custom_interface[CFG_TUSB_HOST_DEVICE_MAX];


static tusb_error_t cush_validate_paras(uint8_t dev_addr, uint16_t vendor_id, uint16_t product_id, void * p_buffer, uint16_t length)
{
  if ( !tusbh_custom_is_mounted(dev_addr, vendor_id, product_id) )
  {
    return TUSB_ERROR_DEVICE_NOT_READY;
  }

  TU_ASSERT( p_buffer != NULL && length != 0, TUSB_ERROR_INVALID_PARA);

  return TUSB_ERROR_NONE;
}
//--------------------------------------------------------------------+
// APPLICATION API (need to check parameters)
//--------------------------------------------------------------------+
tusb_error_t tusbh_custom_read(uint8_t dev_addr, uint16_t vendor_id, uint16_t product_id, void * p_buffer, uint16_t length)
{
  // TU_ASSERT_ERR( cush_validate_paras(dev_addr, vendor_id, product_id, p_buffer, length) );

  // if ( !hcd_pipe_is_idle(custom_interface[dev_addr-1].pipe_in) )
  // {
  //   return TUSB_ERROR_INTERFACE_IS_BUSY;
  // }

  (void) usbh_edpt_xfer( custom_interface[dev_addr-1].ep_in, p_buffer, length);

  return TUSB_ERROR_NONE;
}

tusb_error_t tusbh_custom_write(uint8_t dev_addr, uint16_t vendor_id, uint16_t product_id, void const * p_data, uint16_t length)
{
  // TU_ASSERT_ERR( cush_validate_paras(dev_addr, vendor_id, product_id, p_data, length) );

  // if ( !hcd_pipe_is_idle(custom_interface[dev_addr-1].pipe_out) )
  // {
  //   return TUSB_ERROR_INTERFACE_IS_BUSY;
  // }

  (void) usbh_edpt_xfer( custom_interface[dev_addr-1].ep_out, p_data, length);

  return TUSB_ERROR_NONE;
}

//--------------------------------------------------------------------+
// USBH-CLASS API
//--------------------------------------------------------------------+
void cush_init(void)
{
  tu_memclr(&custom_interface, sizeof(cush_interface_t) * CFG_TUSB_HOST_DEVICE_MAX);
}

tusb_error_t cush_open_subtask(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *p_interface_desc, uint16_t *p_length)
{
  // FIXME quick hack to test lpc1k custom class with 2 bulk endpoints
  uint8_t const *p_desc = (uint8_t const *) p_interface_desc;
  p_desc = tu_desc_next(p_desc);

  // //------------- Bulk Endpoints Descriptor -------------//
  // for(uint32_t i=0; i<2; i++)
  // {
  //   tusb_desc_endpoint_t const *p_endpoint = (tusb_desc_endpoint_t const *) p_desc;
  //   TU_ASSERT(TUSB_DESC_ENDPOINT == p_endpoint->bDescriptorType, TUSB_ERROR_INVALID_PARA);

  //   cush_interface_t* cus_itf = &custom_interface[dev_addr-1];
  //   cus_itf->itf_num   = desc_itf->bInterfaceNumber;
  //   cus_itf->ep_in     = desc_ep->bEndpointAddress;
  //   cus_itf->epin_size = desc_ep->wMaxPacketSize.size;
  //   pipe_handle_t * p_pipe_hdl =  ( p_endpoint->bEndpointAddress &  TUSB_DIR_IN_MASK ) ?
  //                        &custom_interface[dev_addr-1].pipe_in : &custom_interface[dev_addr-1].pipe_out;
  //   *p_pipe_hdl = usbh_edpt_open(dev_addr, p_endpoint, TUSB_CLASS_VENDOR_SPECIFIC);
  //   TU_ASSERT ( pipehandle_is_valid(*p_pipe_hdl), TUSB_ERROR_HCD_OPEN_PIPE_FAILED );

  //   p_desc = tu_desc_next(p_desc);
  // }

  // (*p_length) = sizeof(tusb_desc_interface_t) + 2*sizeof(tusb_desc_endpoint_t);
  return TUSB_ERROR_NONE;
}

void cush_isr(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event)
{

}

void cush_close(uint8_t rhport, uint8_t dev_addr)
{
  tusb_error_t err1, err2;
  cush_interface_t * p_interface = &custom_interface[dev_addr-1];

  // // TODO re-consider to check pipe valid before calling pipe_close
  // if( pipehandle_is_valid( p_interface->pipe_in ) )
  // {
  //   err1 = hcd_pipe_close( p_interface->pipe_in );
  // }

  // if ( pipehandle_is_valid( p_interface->pipe_out ) )
  // {
  //   err2 = hcd_pipe_close( p_interface->pipe_out );
  // }

  tu_memclr(p_interface, sizeof(cush_interface_t));

  TU_ASSERT(err1 == TUSB_ERROR_NONE && err2 == TUSB_ERROR_NONE, (void) 0 );
}

#endif
