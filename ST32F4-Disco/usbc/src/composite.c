/*
 * composite.c
 *
 *  Created on: 23 juil. 2014
 *      Author: michel
 *
 descriptor
    compact :
       save space group in a byte array or packed struct
       duplication of sub desc when common
    desc ptr array ( :
       consume more space when no descriptor re-used
 an dev  lookign similar in hs and fs can re-used most of the decriptor expect any ep descriptor that
 take advantage of bigger maxpacket
 when ep are to use max for it's psed then set max_packet_msize to 0 and use same descriptto for alls peed
 at descriptiton construction tiem that will be auto filled

 ep number => in/out use std usb ep 0x80 mask extra 0x40 is added for auto allocated ep number
 non automatic number must be filled and valid from registration and all function life cycle
 interface number => updated

interface and tehre alternate setting must be consecutive
manual interface number are almost immpossible to manage

to some extent str index in "interface" could be made automatic
 */

#include "usbd_core.h"
#include <assert.h>
#include "composite.h"


/****
 * CONFIG FLAGS
 */
#define ONE_SHARED_STATIC_CTRL_BUFFER   512
// when set a unqie static buffer si shared across all composite dev as only oen can eb actieva at a tiem ther's no risk

#define COMPDEV_MAX_IN_EP  16 /* not being 16 would cause cos reveiw to protect array boaund  */
#define COMPDEV_MAX_OUT_EP 16 /* not being 16 would cause cos reveiw to protect array boaund  */
#define COMPDEV_MAX_INTF   8
#define COMPDEV_MAX_FUNC   4
#define COMPDEV_MAX_FUNC_EP 4 /* 2x as among int /out */
#define COMPDEV_MAX_FUNC_IF 4
#define COMPDEV_MAX_FUNC_STR    16


/**
 * debug log and error
 */
#define cdev_assert(cond ) assert(cond)
#define cdev_err( msg, ... )\
    log_err(cdev->name, msg, ##__VA_ARGS__)

int n_err=0;
void log_err(){
    n_err++;
}

uint8_t _CDev_Init           (struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
uint8_t _CDev_DeInit         (struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
/* Control Endpoints*/
uint8_t  _CDev_Setup         (struct _USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req);
uint8_t  _CDev_EP0_TxSent    (struct _USBD_HandleTypeDef *pdev );
uint8_t  _CDev_EP0_RxReady   (struct _USBD_HandleTypeDef *pdev );
 /* Class Specific Endpoints*/
uint8_t  _CDev_DataIn        (struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
uint8_t  _CDev_DataOut       (struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
uint8_t  _CDev_SOF              (struct _USBD_HandleTypeDef *pdev);
uint8_t  _CDev_IsoINIncomplete  (struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
uint8_t  _CDev_IsoOUTIncomplete (struct _USBD_HandleTypeDef *pdev , uint8_t epnum);

uint8_t  *_CDev_GetHSConfigDescriptor(uint16_t *length);
uint8_t  *_CDev_GetFSConfigDescriptor(uint16_t *length);
//uint8_t  *_CDev_GetOtherSpeedConfigDescriptor(uint16_t *length);
uint8_t  *_CDev_GetDeviceQualifierDescriptor(uint16_t *length);
#if (USBD_SUPPORT_USER_STRING == 1)
uint8_t  *_CDev_GetUsrStrDescriptor(struct _USBD_HandleTypeDef *pdev ,uint8_t index,  uint16_t *length);
#endif

USBD_ClassTypeDef CDev_Class={
    _CDev_Init,
    _CDev_DeInit,
    /* Control Endpoints*/
    _CDev_Setup,
    _CDev_EP0_TxSent,
    _CDev_EP0_RxReady,
    /* Class Specific Endpoints*/
    _CDev_DataIn,
    _CDev_DataOut,
    _CDev_SOF,
    _CDev_IsoINIncomplete,
    _CDev_IsoOUTIncomplete,

    _CDev_GetHSConfigDescriptor,
    _CDev_GetFSConfigDescriptor,
    _CDev_GetFSConfigDescriptor,  // GetOtherSpeedConfigDescriptor => FS
    _CDev_GetDeviceQualifierDescriptor,
    #if (USBD_SUPPORT_USER_STRING == 1)
    _CDev_GetUsrStrDescriptor,
    #endif
};

#define _PRIVATE static
#define _PRIVATE_V  static
#define _M_INLINE inline

/*
 * interface descr must follow in order [ie if0 alt0 if0 alt 1][if1 alt0 .. ] .. [ifx_alt0 ... ifx_altz]
 */


struct  usbfunc_t {
    const char *name;                   // for debug purpose

    uint8_t InEps[COMPDEV_MAX_FUNC_EP];     // 0 mean not used entry else the ep address
    uint8_t OutEps[COMPDEV_MAX_FUNC_EP];    // 0 mean not used entry else the ep address

    struct usbfunc_itf_t *intf;             // function driver
    char **StrTable;                        // string table of that function 0 terminated
    uint8_t str_1st;                        // index of first string of that fucntion (set at cfg select)
    uint8_t str_cnt;                        // str cnt even if not all used that will be set
    uint8_t *desc_fs;                       // Descriptor compact form or struct xxx_desc ** ?
    uint8_t *desc_hs;                       // as above but for hs
    //if hs and fs are fully auto (or manual ) where all ep use auto max packet size of fixed siz (ie  no bulk hs)
    // then hs and fs desc can be same all ep max packet will be auto adjusted when providing descr to host

    uint8_t option;
    uint8_t index;                             // index in cdev for back link
};


struct CDev_t  {
    char *name;
    uint8_t Intf2Func[COMPDEV_MAX_INTF];
    uint8_t InEpf2Func [COMPDEV_MAX_IN_EP];     /* ep 0 and we used 0 for "not assigned"  */
    uint8_t OutEpf2Func [COMPDEV_MAX_OUT_EP];
    struct usbfunc_t *Funcs[COMPDEV_MAX_FUNC];
    char **StrTable;
    uint8_t n_func;
    uint8_t n_String;
};

/* use during configuration descriptor elaboration  */
struct cfg_context_t {
    uint8_t *buffer;
    uint8_t *AutoInEps;
    uint8_t *AutoOutEps;


    uint16_t used_in_ep; // bit mask
    uint16_t used_out_ep; // bti mask

    uint16_t buf_size;
    uint16_t cur_offset;

    uint8_t auto_intf_index;  // todo start index for consecutive function with pre asigned intf
    uint8_t speed;
    uint8_t manual_itf;
    uint8_t Activate;
};

#define CFG_INTF_OVER   1
#define CFG_EP_OVER     2
#define CFG_FUNC_EP_OVER 3
#define CFG_EP_INVALID  4
#define CFG_TOO_BIG     5



_PRIVATE_V struct _USBD_HandleTypeDef *active_usbd;
_PRIVATE_V uint16_t CDev_NonFreeInEp=0;         // bit mask of initial non free ep
_PRIVATE_V uint16_t CDev_NonFreeOutEp=0;        // bit mask of initial non free ep

_PRIVATE_V const uint16_t max_packet_size_lut[]={512, 64, 64}; //TODO check low spped max bulk packet size

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t _CDev_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


#ifdef ONE_SHARED_STATIC_CTRL_BUFFER
/* config/funcion add while a composite device is active on usb create usage conflict
 * if that used case is needed use option one buffer per config or protect exclusive buffer access */
    static uint8_t ctrl_buffer[ONE_SHARED_STATIC_CTRL_BUFFER];
    #define __CDev_CtrlBuffer(...) ctrl_buffer
    #define __CDev_CtrlBufferSize(...) ONE_SHARED_STATIC_CTRL_BUFFER
#endif


//#define _M_INLINE __attribute__((always_inline)) /* as much as possible always inline but for deep debug */

#define __CDev_GetFirstStrIndex(cdev)   (USBD_IDX_INTERFACE_STR+1)

_PRIVATE struct CDev_t * GetActiveUSBD(){
    assert( active_usbd!=NULL);
    return (struct CDev_t *)active_usbd->pClassData;
}

_PRIVATE struct CDev_t * usbd_2_cdev(const struct _USBD_HandleTypeDef *dev ){
    return dev->pClassData;
}


int _PRIVATE AssignEp(struct CDev_t *cdev , uint16_t *used_ep_flag){
    int ep,ep_bit;
    for( ep=1; ep<COMPDEV_MAX_IN_EP; ep++ ){
        ep_bit=(1<<ep);
        if(  !((*used_ep_flag)& ep_bit )  ){
            (*used_ep_flag)|=ep_bit;
            return ep;
        }
    }
    return 0;
}



_PRIVATE _M_INLINE int IsAutoEp(struct usbfunc_t *uf, uint8_t addr ){
    return (addr&AUTO_EP_BIT_MASK) || (uf->option&FORCE_AUTO_EP);
}



//if retun more than max_data :( error case to long config or func
_PRIVATE
int ParseFuncDesc(struct CDev_t * cdev, struct usbfunc_t *uf, struct cfg_context_t *ctx, uint8_t *pdesc)
{
    uint8_t *pb, *cfg_end;
    uint8_t *o_data;

    struct usb_desc_t *desc_cur;
    struct usb_desc_t **desc_array;
    struct usb_interface_descr_t *itf;
    struct usb_endpoint_descr_t *ep;
    int status = 0;
    int max_packet_size = max_packet_size_lut[ctx->speed];
    int ep_addr, ep_in, ep_nr;
    int n_intf = 0, intf_nr;

    int prev_inteface_number = -1; /* set a value that is not auto nor a valid interface number ie 1<<5 or -1*/

    int do_desc = 1; /* real run with descr copy and ep intf assignment or check */
    int cfg_mode = uf->option & CFG_DESC;

    if (cfg_mode) {
        struct usb_config_descr_t *cfg;
        cfg = (struct usb_config_descr_t *) pdesc;
        cfg_end = pdesc + cfg->wTotalLength;
        /* skeep initial configuration that will not be added */
        desc_cur =(struct usb_desc_t *) (pdesc + cfg->bLength);
    } else {
        desc_array = (struct usb_desc_t **) pdesc;
        desc_cur = *desc_array;
    }

    o_data = ctx->buffer + ctx->cur_offset;
    while (cfg_mode ? (uint8_t*)desc_cur < cfg_end : desc_cur != NULL) {
        if (o_data - ctx->buffer + desc_cur->len > ctx->buf_size) {
            // to many data return > max_data  more TODO ? */
            status =CFG_TOO_BIG;
            break;
        }
        switch (desc_cur->type) {
        case USB_DESC_TYPE_INTERFACE:
            itf = (struct usb_interface_descr_t *) desc_cur;
            if (itf->bInterfaceNumber != prev_inteface_number) {
                n_intf++;
                prev_inteface_number = itf->bInterfaceNumber;
                /* if auto or forced assign itf to it */
                if ( (uf->option & FORCE_AUTO_IF)|| (itf->bInterfaceNumber & AUTO_INTERFACE_BIT_MASK)) {
                    intf_nr = ctx->auto_intf_index;
                    ctx->auto_intf_index++;
                    /* if func need an index to auto index lookup add it here that
                     * will ne beed if not all interface are auto assign else only first alt is needed*/
                } else {
                    ctx->manual_itf++;
                    intf_nr = itf->bInterfaceNumber;
                }

                if (intf_nr >= COMPDEV_MAX_INTF) {
                    /* no more interface  todo any better handle error ?*/
                    cdev_err("too many interface");
                    status = CFG_INTF_OVER;
                    goto error;
                }
                /* update int lookup if needed */
                if( ctx->Activate) {
                    // if host ask for desc when cdev in used these coudl trash existign valeus ?
                    // but can parsing give twice different result ? => yes if ran for in used speed or  config
                    cdev->Intf2Func[intf_nr] = uf->index;
                }
            }
            /* else it's an alternate setting only update intf nr hat is valdi from 1st intf loop */
            if (do_desc) {
                memcpy(o_data, desc_cur, desc_cur->len);
                itf = (struct usb_interface_descr_t *) o_data;
                itf->bInterfaceNumber = intf_nr;
            }
            break;

        case USB_DESC_TYPE_ENDPOINT:
            ep = (struct usb_endpoint_descr_t *) desc_cur;
            ep_addr = ep->bEndpointAddress;
            ep_in = ep->bEndpointAddress & 0x80;
            ep_nr = ep_addr & 0x0F;
            if( ep_nr <= 0 ){
                status=CFG_EP_INVALID;
                goto error;
            }
            if ( IsAutoEp( uf, ep_addr) ) {
                if (ep_nr < COMPDEV_MAX_FUNC_EP) {
                    uint8_t *AutoEps;
                    AutoEps = ep_in ? ctx->AutoInEps : ctx->AutoOutEps;
                    // if already assigned re-used
                    if (AutoEps[ep_nr]) {
                        ep_addr = AutoEps[ep_nr];
                    } else {
                        /* assigned auto ep */
                        ep_addr = AssignEp(cdev, ep_addr & 0x80 ? &ctx->used_in_ep : &ctx->used_out_ep);
                        if (ep_addr == 0) {
                            status = CFG_EP_OVER;
                            break;
                        }
                        ep_addr |= ep_in; /* set direction */
                        AutoEps[ep_nr] = ep_addr;
                    }
                } else {
                    cdev_err("too many ep for function");
                    status = CFG_FUNC_EP_OVER;
                    break;
                }
            }
            if (do_desc) {
                int  pack_size;
                // take care of potentially unaligned wMaxPacketSize
                pb = (uint8_t*) ep;
                pack_size = (pb[5] << 8) + pb[4];
                if (pack_size == 0)
                    pack_size = max_packet_size;
                memcpy(o_data, ep, ep->bLength);
                o_data[4] = pack_size & 0xFF;
                o_data[5] = (pack_size >> 8) & 0xFF;
                ep = (struct usb_endpoint_descr_t *) o_data;
                ep->bEndpointAddress = ep_addr;
            }
            if( ctx->Activate){
                uint8_t *Ep2Func;
                Ep2Func= ep_in ? cdev->InEpf2Func : cdev->OutEpf2Func;
                Ep2Func[ep_nr] = uf->index;
            }
            break;
        default:
            if (do_desc) {
                memcpy(o_data, desc_cur, desc_cur->len);
            }
        }
        /* advance output ptr */
        o_data += desc_cur->len;
        ctx->cur_offset+=desc_cur->len;
        if( cfg_mode )
            desc_cur = (struct usb_desc_t *) (((uint8_t*) desc_cur) + desc_cur->len);
        else
            desc_cur =(struct usb_desc_t *) *(desc_array++);
    }
error:
    return status;
}


int __CDev_do_cfg_descr(struct CDev_t * cdev, int activate_cfg, USBD_SpeedTypeDef speed, struct cfg_context_t *ctx)
{
    struct  usbfunc_t *uf;
    int f;
    int status;
    int str_1st;
    uint8_t *pdesc;
//    struct cfg_context_t ctx;
    uint8_t InEps[COMPDEV_MAX_FUNC_EP];   // 0 mean not used entry else the ep address
    uint8_t OutEps[COMPDEV_MAX_FUNC_EP];   // 0 mean not used entry else the ep address
    struct usb_config_descr_t *cfg;
    memset(ctx,0, sizeof(*ctx));
    ctx->buffer=__CDev_CtrlBuffer(cdev);
    ctx->buf_size=__CDev_CtrlBufferSize(cdev);
    ctx->cur_offset=sizeof(struct usb_config_descr_t);
    ctx->used_in_ep = CDev_NonFreeInEp;
    ctx->used_out_ep = CDev_NonFreeOutEp;
    ctx->auto_intf_index = 0;
    ctx->AutoInEps =InEps;   /* overridden if activating the function */
    ctx->AutoOutEps = OutEps;
    ctx->Activate = activate_cfg;
    ctx->speed = speed;

    str_1st=__CDev_GetFirstStrIndex(cdev);
    for( f=0; f<cdev->n_func; f++){
        uf=cdev->Funcs[f];
        uf->str_1st=str_1st;
        str_1st+=uf->str_cnt;
        /* reset auto ep fucntion */
        if( activate_cfg ){
            ctx->AutoInEps = uf->InEps;
            ctx->AutoOutEps = uf->OutEps;
        }
        /* reset auto ep for that function when checking these are local dummy ep */
        memset(ctx->AutoInEps,0, COMPDEV_MAX_FUNC_EP);
        memset(ctx->AutoOutEps,0, COMPDEV_MAX_FUNC_EP);

        pdesc = speed==USBD_SPEED_HIGH  ? uf->desc_hs : uf->desc_fs;
        if( pdesc ){
            /* function is not required to  exist at all speed */
            status = ParseFuncDesc( cdev, uf, ctx,  pdesc );
            if( status ){
               break;
            }
        }
    }
    if( status ){
        /* kill the config so that it appear as a void device */
        ctx->cur_offset= sizeof(struct usb_config_descr_t);
        ctx->auto_intf_index =0;
    }
    cfg= ( struct usb_config_descr_t *)ctx->buffer;
    cfg->bDescriptorType = USB_DESC_TYPE_CONFIGURATION;
    cfg->bLength = sizeof(struct usb_config_descr_t);
    cfg->bNumInterfaces= ctx->auto_intf_index;
    cfg->wTotalLength = ctx->cur_offset;
    return status;
}

uint8_t* _CDev_GetHSConfigDescriptor(uint16_t *length){
    struct CDev_t * cdev=GetActiveUSBD() ;
    struct cfg_context_t ctx;
    __CDev_do_cfg_descr(cdev, 0, USBD_SPEED_HIGH, &ctx);
    *length=ctx.cur_offset;
    return __CDev_CtrlBuffer(cdev);
}

uint8_t* _CDev_GetFSConfigDescriptor(uint16_t *length){
    struct CDev_t * cdev=GetActiveUSBD();
    struct cfg_context_t ctx;
    __CDev_do_cfg_descr(cdev, 0, USBD_SPEED_FULL, &ctx);
    *length=ctx.cur_offset;
    return __CDev_CtrlBuffer(cdev);
}

uint8_t* _CDev_GetDeviceQualifierDescriptor(uint16_t *length){
    *length=USB_LEN_DEV_QUALIFIER_DESC;
    return _CDev_DeviceQualifierDesc;
}

uint8_t* _CDev_GetUsrStrDescriptor(struct _USBD_HandleTypeDef *pdev ,uint8_t index,  uint16_t *length){
    struct CDev_t * cdev=usbd_2_cdev(pdev);

}

uint8_t  _CDev_Setup (struct _USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req){
    uint8_t status=USBD_FAIL;
    uint8_t recip = req->bRequest&USB_REQ_RECIPIENT_MASK;
    switch ( recip ){
    case USB_REQ_RECIPIENT_INTERFACE:
        // std

        break;
    case  USB_REQ_RECIPIENT_ENDPOINT:
        // normlay we only receive halt here bt yet filter that
        break;

    default:
        break;
    }
    return status;
}


uint8_t _CDev_EP0_TxSent(struct _USBD_HandleTypeDef *pdev )
{
    struct CDev_t * cdev=usbd_2_cdev(pdev);
    return 0;
}

uint8_t _CDev_EP0_RxReady(struct _USBD_HandleTypeDef *pdev )
{
    struct CDev_t * cdev=usbd_2_cdev(pdev);
    return 0;
}


uint8_t _CDev_DataIn (struct _USBD_HandleTypeDef *pdev , uint8_t epnum)
{
    struct CDev_t * cdev=usbd_2_cdev(pdev);
    int f;
    epnum&=0x0F;
    cdev_assert(epnum < COMPDEV_MAX_OUT_EP);
    f = cdev->InEpf2Func[epnum];
    if( f < COMPDEV_MAX_FUNC ){
        struct usbfunc_t *uf;
        uf=cdev->Funcs[f];
        if( uf && uf->intf->in_data){
            uf->intf->in_data(uf, epnum);
        }
    }
    return USBD_FAIL;
}

uint8_t  _CDev_DataOut (struct _USBD_HandleTypeDef *pdev , uint8_t epnum){
    struct CDev_t * cdev=usbd_2_cdev(pdev);
    int f;

    epnum&=0x0F;
    cdev_assert(epnum < COMPDEV_MAX_OUT_EP);
    f = cdev->OutEpf2Func[epnum];
    if( f < COMPDEV_MAX_FUNC ){
        struct usbfunc_t *uf;
        uf=cdev->Funcs[f];
        if( uf && uf->intf->out_data){
            return uf->intf->out_data(uf, epnum);
        }
    }
    return USBD_FAIL;
}

uint8_t  _CDev_SOF (struct _USBD_HandleTypeDef *pdev){
    struct CDev_t * cdev=usbd_2_cdev(pdev);
    return 0;
}
uint8_t  _CDev_IsoINIncomplete  (struct _USBD_HandleTypeDef *pdev , uint8_t epnum){
    struct CDev_t * cdev=usbd_2_cdev(pdev);
    return 0;
}

uint8_t  _CDev_IsoOUTIncomplete (struct _USBD_HandleTypeDef *pdev , uint8_t epnum){
    struct CDev_t * cdev=usbd_2_cdev(pdev);
    return 0;
}


/**
 * Disable all function
 * @param cdev  the cdev object
 */
void __CDev_DisableFuncs(struct CDev_t * cdev ){
    struct usbfunc_t *uf;
    int i;
    for( i=0 ; i<cdev->n_func ;i++ ){
        uf=cdev->Funcs[i];
        if( uf->intf->disable)
            uf->intf->disable(uf);
    }
}

uint8_t _CDev_DeInit(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx){
    struct CDev_t * cdev = usbd_2_cdev(pdev);
    /* cfg idx = 0 normaly  already test by usbd no need to check */
    __CDev_DisableFuncs(cdev);
    return 0;
}

/**
 * IS called at  "set config time for config 0"
 *
 * @param pdev
 * @param cfgidx
 * @return
 */
uint8_t _CDev_Init(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    struct CDev_t * cdev = usbd_2_cdev(pdev);
    struct usbfunc_t *uf;
    int status = 0;
    int i;
/* note cfg idx = 0 = reset cfg is receive via "Deinit" */
    active_usbd=pdev;   /* store "unique active pdev as some call do nto havea,y as ref to retrivee the class ptr */
    if (cfgidx == 1) {
        //TODO return all func to alt 0 ? call init first
        for (i = 0; i < cdev->n_func; i++) {
            uf = cdev->Funcs[i];
            if (uf->intf->enable) {
                status = uf->intf->enable(uf);
                if (!status)
                    break; /* if any interface is not enable to init stop init and stall the set cfg */
            }
        }
    }
    else {
        // TODO handle multi config idx = 2 etc ..
        status = -1;
    }
    return status;
}


uint8_t* CDev_GetDeviceDescriptor( USBD_SpeedTypeDef speed , uint16_t *length){
    /* TDDO */
    return NULL;
}



void usbfunc_init( struct usbfunc_t *uf){
    memset(uf->InEps, 0, sizeof(uf->InEps));
    memset(uf->OutEps, 0, sizeof(uf->OutEps));
    uf->index=-1;
}


/******************************************************************************
 *  interface to user
 *****************************************************************************/

void CDev_ReservedEp(int ep_addr){
    int ep_nr = ep_addr&0x0F;
    int ep_in = ep_addr&0x80;
    int ep_mask = 1<<ep_nr;

    if( ep_in )
        CDev_NonFreeInEp|=ep_mask;
    else
        CDev_NonFreeOutEp|=ep_mask;
}

int CDev_Init(struct CDev_t *cdev){
    memset(cdev, 0, sizeof(struct CDev_t));
    return 0;
}

struct CDev_t * CDev_NewDevice(){
    struct CDev_t *cdev;
    cdev=malloc(sizeof(struct CDev_t));
    if( cdev!=NULL){
        CDev_Init(cdev);
    }
    return cdev;
}

struct usbfunc_t *CDev_NewFunc( uint8_t *desc_hs, uint8_t *desc_fs, struct usbfunc_itf_t *itf, enum AddFuncFlags_e option_flag  )
{
    struct usbfunc_t *uf;
    uf = malloc(sizeof(*uf));
    if( uf!=NULL ){
        usbfunc_init(uf);
        uf->desc_fs = desc_fs;
        uf->desc_hs = desc_hs;
        uf->option  = option_flag;
        uf->intf    = itf;
    }
    return uf;
}

int CDev_SetFuncStrings(struct usbfunc_t *uf, char **str_table, int n_str)
{
    uf->StrTable=str_table;
    if( n_str<=0 ){
        int n=0;
        while(*str_table!=NULL && n < COMPDEV_MAX_FUNC_STR){
            str_table++;
            n++;
        }
        uf->str_cnt=n;
        if( *str_table!=NULL )
            return -1;
    }
    else{
        uf->str_cnt=n_str;
    }
    return 0;
}

int CDev_AddFunc(struct CDev_t *cdev, struct usbfunc_t *uf, int cfg_desc, const char *name){
    int status;

    /* TODO handle multi cfg ? */
    cdev_assert(uf!=NULL);
    uf->name=name;
    cdev_assert( !(uf->desc_fs==NULL || uf->desc_hs==NULL)); /* not 2 desc cannot be null */
    if( cdev->n_func < COMPDEV_MAX_FUNC){
        /* TODO try a dry run of the cfg desc to cjeck ep /intf? */
        cdev->Funcs[cdev->n_func]=uf;
        uf->index=cdev->n_func;
        cdev->n_func++;
        status = 0;
    }
    else{
        /* to many function already */
        status = -1;
    }
    return status;
}

int CDev_Register(struct CDev_t *cdev, USBD_HandleTypeDef *pdev){
    /* validate the configuration at all speed */
    int status;
    struct cfg_context_t ctx;
    status = __CDev_do_cfg_descr(cdev, 0, USBD_SPEED_FULL, &ctx);
    if( !status  ){
        status = __CDev_do_cfg_descr(cdev, 0, USBD_SPEED_HIGH, &ctx);
        if( !status )
            status = USBD_RegisterClass(pdev, &CDev_Class);
    }
    return status;
}



