/*
 * composite.h
 *
 *  Created on: 2 août 2014
 *      Author: michel
 */

#ifndef COMPOSITE_H_
#define COMPOSITE_H_

#include <stdint.h>

enum AddFuncFlags_e {
    /*  FIXME if these come to be larget than 8 bit review option in func struct */
    CFG_DESC        =0x0001, /* if not set assumed array or descriptor  ptr ended by a null ptr */
    FORCE_AUTO_EP   =0x0002, /* when set al ep "address" are automatic even if not marked as such */
    /* do we have option for  that ?  likely not  if interface number are to be suscessive ?
     * at lest mnual interafce number must be known in advance */
    FORCE_AUTO_IF   =0x0004, /* when set al ep "address" are automatic even if not marked as such */

    /* force auto i/f and ep is to used "one config" descriptor ST class sample without any adjustment
     * yet the code must take into account the fact final ep address may be different and must be
     * Adjust using TBD or parsing the descriptor or by storing that in some user data ptr
     * at init time (after final ep allocation)
     */
};

#define AUTO_INTERFACE_BIT_MASK 0x40    /* add these to a interface number to make it be treated as auto */
#define AUTO_EP_BIT_MASK        0x40    /* add these to an ep addr to make it be treated as auto */

struct CDev_t;     /* forward definition of internal structure*/
struct  usbfunc_t; /* forward definition of internal structure*/
struct usbfunc_itf_t   {
    int  (*setup)   (struct  usbfunc_t *f, USBD_SetupReqTypedef *req );
    void (*disable) (struct  usbfunc_t *f);
    int  (*enable)  (struct  usbfunc_t *f);
    int  (*set_alt) (struct  usbfunc_t *f, int itf, int alt);
    int  (*init)    (struct  usbfunc_t *f);
    int  (*in_data) (struct  usbfunc_t *f, int ep);
    int  (*out_data)(struct  usbfunc_t *f, int ep);
};


/** Initialize a CDev object
 * @param cdev  The object to init (static or allocated)
 * @return 0 on success
 */
int CDev_Init(struct CDev_t *cdev);

 /** Allocate and init a new composite device
  *
  * @return Non NUMM if success
  */
struct CDev_t * CDev_NewDevice();

/** CDev_ReservedEp
 * set an end-point not free for automatic attribution
 * @param ep_addr full ep address with direction set
 */
void CDev_ReservedEp(int ep_addr);

/** CDev_Register register composite devcie to USBD controler
 *
 * @param cdev  Composite device to register
 * @param pdev  USBD controler  to register to
 * @return
 */
int CDev_Register(struct CDev_t *cdev, USBD_HandleTypeDef *pdev);

/** allocate and init a new dynamic function
 *  new allocated function is not added to any device/config
 * @param desc_hs       high speed descriptor to use
 * @param desc_fs       full speed descriptor to use
 * @param itf           function driver interface
 * @param option_flag   options see enum definition
 * @return              pointer to function
 */
struct usbfunc_t *CDev_NewFunc( uint8_t *desc_hs, uint8_t *desc_fs, struct usbfunc_itf_t *itf, enum AddFuncFlags_e option_flag  );

/** set string table associaetd to the function
 * all interafce string id non 0 with auto_str_index mask will be maped to that string array
 * composite devcie will arrange final str idnex and string descripo retrun to host
 *
 * @param func          the function
 * @param str_table     The string table ["s#1","s#2"..., "s#n_str"(,NULL)]
 * @param n_str         optional set <=0  if tabel is NULLL terminated when not >0 these idicate the len of table
 * @return              0 on sucess
 */
int CDev_SetFuncStrings(struct usbfunc_t *func, char **str_table, int n_str);

/**
 * CDev_AddFunc Add an existing valid usb function to a composite device
 * no valid check is performed ! be sure function init correctly as  it can lead to malfunction
 * mutable init value are very important for run time life
 *
 * @param cdev     device to add function too
 * @param uf       function ptr
 * @param cfg_desc True if func descr are mono config desc if fasle desc are desc array end by null
 * @return 0 on sucess
 */
int CDev_AddFunc(struct CDev_t *cdev, struct usbfunc_t *uf, int cfg, const char *name);


/** usbfunc_init initialized function structure
 *
 * @param uf    ptr to the function
 */
void usbfunc_init( struct usbfunc_t *uf);


/**
 * common definition for any usb descriptor
 */
struct usb_desc_t {
    uint8_t len;
    uint8_t type;
};


struct usb_config_descr_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} __attribute__ ((packed));

struct usb_interface_descr_t  {
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} __attribute__ ((packed));

struct usb_endpoint_descr_t {
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} __attribute__ ((packed));

struct ctrl_req_t {
    uint8_t  bRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__ ((packed));


#endif /* COMPOSITE_H_ */
