#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include "libusb_helper.h"
#include <helper/command.h>

#include <stdint.h>

#define USB_VID 0x6874
#define USB_PID 0x6263

#define COLOBUS_INTERFACE 4
#define BULK_EP_OUT 0x07
#define BULK_EP_IN  0x88

#define SWD_RSP_OK          0b001
#define SWD_RSP_WAIT        0b010
#define SWD_RSP_FAULT       0b100
#define SWD_RSP_LINE_ERROR  0b111

#pragma mark libgeneral defines
// -- cleanup --
#define safeFree(ptr) ({if (ptr) {free(ptr); ptr=NULL;}})
#define safeFreeCustom(ptr,func) ({if (ptr) {func(ptr); ptr=NULL;}})
#define safeFreeConst(ptr) ({if(ptr){void *fbuf = (void*)ptr;ptr = NULL; free(fbuf);}})
#define safeClose(fd) ({if (fd != -1) {close(fd); fd=-1;}})
#define safeCloseCustom(fd,func) ({if (fd != -1) {func(fd); fd=-1;}})
// -- assure --
#define cassure(a) do{ if ((a) == 0){err=__LINE__; goto error;} }while(0)
#define cretassure(cond, errstr ...) do{ if ((cond) == 0){err=__LINE__;LOG_ERROR(errstr); goto error;} }while(0)
#define creterror(errstr ...) do{LOG_ERROR(errstr);err=__LINE__; goto error; }while(0)



#pragma mark defines
enum COLOBUS_CMDS {
    kCOLOBUS_CMD_INVALID = 0,
    kCOLOBUS_CMD_READ,
    kCOLOBUS_CMD_WRITE,
    kCOLOBUS_CMD_RESET,
    kCOLOBUS_CMD_FREQ
};

struct __attribute__((__packed__)) colobus_cmd {
    uint8_t id;
    uint8_t cmd;
    uint8_t req;
    uint8_t res;
    uint32_t data;
};

struct colobus {
	libusb_device_handle *usb_handle;
    struct colobus_cmd *queueCmd;
    uint32_t **queueRsp;
    uint16_t gid;
    size_t queue_size;
	size_t queue_elements;
};

#pragma mark globals
static struct colobus *gHandle = NULL;

#pragma mark helpers
static void logCmd(struct colobus_cmd cmd, bool isReq){
    switch (cmd.cmd){
        case kCOLOBUS_CMD_READ:
            LOG_DEBUG("COLOBUS: %s READ  id: 0x%02x req: 0x%02x data: 0x%08x res: %d", isReq ? "REQ" : "RSP", cmd.id, cmd.req, cmd.data, cmd.res);
            break;

        case kCOLOBUS_CMD_WRITE:
            LOG_DEBUG("COLOBUS: %s WRITE id: 0x%02x req: 0x%02x data: 0x%08x res: %d", isReq ? "REQ" : "RSP", cmd.id, cmd.req, cmd.data, cmd.res);
            break;

        case kCOLOBUS_CMD_RESET:
            LOG_DEBUG("COLOBUS: %s RESET id: 0x%02x ------------------------- res: %d", isReq ? "REQ" : "RSP", cmd.id, cmd.res);
            break;

        case kCOLOBUS_CMD_FREQ:
            LOG_DEBUG("COLOBUS: %s FREQ  id: 0x%02x ----------- freq: %d --- res: %d", isReq ? "REQ" : "RSP", cmd.id, cmd.data, cmd.res);
            break;

        default:
            LOG_DEBUG("COLOBUS: %s unkown cmd %d", isReq ? "REQ" : "RSP", cmd.cmd);
            break;
    };
}


#pragma mark exported functions 
static const struct command_registration command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const char * const colobus_transports[] = { 
    "swd", 
    NULL
};

#pragma mark SWD functions 
static void colobus_queue_pushcmd(struct colobus_cmd cmd, uint32_t *rsp){
    logCmd(cmd, 1);
    if (gHandle->queue_size < gHandle->queue_elements + 1){
        size_t newsize = gHandle->queue_size*1.5;
        gHandle->queueCmd = realloc(gHandle->queueCmd, newsize * sizeof(struct colobus_cmd));
        gHandle->queueRsp = realloc(gHandle->queueRsp, newsize * sizeof(uint32_t *));
        gHandle->queue_size = newsize;
    }
	gHandle->queueCmd[gHandle->queue_elements] = cmd;
	gHandle->queueRsp[gHandle->queue_elements] = rsp;
    gHandle->queue_elements++;
}

static int colobus_swd_init(void){
	return ERROR_OK;
}

static void colobus_swd_reg_read(uint8_t req, uint32_t *value, uint32_t ap_delay_clk){
    (void)ap_delay_clk;
    struct colobus_cmd cmd = {
        .id = gHandle->gid++,
        .cmd = kCOLOBUS_CMD_READ,
        .req = req | 0x81,
        .data =  0,
	};
    colobus_queue_pushcmd(cmd, value);
}

static void colobus_swd_reg_write(uint8_t req, uint32_t value, uint32_t ap_delay_clk){
    (void)ap_delay_clk;
    struct colobus_cmd cmd = {
        .id = gHandle->gid++,
        .cmd = kCOLOBUS_CMD_WRITE,
        .req = req | 0x81,
        .data =  value,
	};
    colobus_queue_pushcmd(cmd, NULL);
}

static void colobus_swd_reset(void){
    struct colobus_cmd ccmd = {
        .id = gHandle->gid++,
        .cmd = kCOLOBUS_CMD_RESET,
	};
    colobus_queue_pushcmd(ccmd, NULL);
}

static int colobus_transfer_queue_elements(int start, int end){
    if (start == end) return 0;
    size_t cmdsSize = sizeof(*gHandle->queueCmd)*(end-start);
    int ret = 0;
    int didTransfer = 0;
    ret = jtag_libusb_bulk_write(gHandle->usb_handle, BULK_EP_OUT, (char*)&gHandle->queueCmd[start], cmdsSize, LIBUSB_TIMEOUT_MS, &didTransfer);
    if (ret < 0){
        return ERROR_JTAG_DEVICE_ERROR;
    }
    return didTransfer/sizeof(*gHandle->queueCmd);
}

static int colobus_receive_queue_elements(int start, int end){
    if (start == end) return 0;
    size_t cmdsSize = sizeof(*gHandle->queueCmd)*(end-start);
    int ret = 0;
    int didTransfer = 0;
    ret = jtag_libusb_bulk_read(gHandle->usb_handle, BULK_EP_IN | LIBUSB_ENDPOINT_IN, (char*)&gHandle->queueCmd[start], cmdsSize, LIBUSB_TIMEOUT_MS, &didTransfer);
    if (ret < 0){
        return ERROR_JTAG_DEVICE_ERROR;
    }
    if (didTransfer % sizeof(*gHandle->queueCmd)){
        // we don't recover from partial receive :(
        return ERROR_JTAG_DEVICE_ERROR;
    }
    return didTransfer/sizeof(*gHandle->queueCmd);
}

static int colobus_swd_run_queue(void){
    int err = 0;
    size_t transferredElements = 0;
    size_t receivedElements = 0;

        LOG_DEBUG("COLOBUS: run queue %zu",gHandle->queue_elements);

    for (size_t i = 0; i < 100; i++){
        int curTransfer = 0;
        int curReceive = 0;
        cassure((curTransfer = colobus_transfer_queue_elements(transferredElements, gHandle->queue_elements)) >= 0);
        transferredElements += curTransfer;
        cassure((curReceive = colobus_receive_queue_elements(receivedElements,transferredElements)) >= 0);
        receivedElements += curReceive;
		keep_alive();
        if (receivedElements == gHandle->queue_elements) break;
    }

    {
        size_t processedElements = 0;
        for (; processedElements < receivedElements; processedElements++){
            logCmd(gHandle->queueCmd[processedElements], 0);

            cassure(gHandle->queueCmd[processedElements].res == SWD_RSP_OK);
            uint32_t *dst = gHandle->queueRsp[processedElements]; gHandle->queueRsp[processedElements] = NULL;
            if (dst){
                *dst = gHandle->queueCmd[processedElements].data;
            }
        }
        cassure(processedElements == gHandle->queue_elements);
    }

error:
	gHandle->queue_elements = 0;
    if (err){
        return ERROR_JTAG_DEVICE_ERROR;
    }    
	return ERROR_OK;
}

static int colobus_swd_switch_seq(enum swd_special_seq seq){
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
        colobus_swd_reset();
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
        colobus_swd_reset();
		break;
	default:
		LOG_DEBUG("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct swd_driver colobus_swd = {
	.init = colobus_swd_init,
	.switch_seq = colobus_swd_switch_seq,
	.read_reg = colobus_swd_reg_read,
	.write_reg = colobus_swd_reg_write,
	.run = colobus_swd_run_queue,
};

#pragma mark general jtag functions 
static int colobus_init(void) {
    int err = 0;
    int res = ERROR_JTAG_INIT_FAILED;

	cretassure(gHandle = calloc(1, sizeof(struct colobus)), "Failed to allocate memory for handle");
    cretassure(gHandle->queueCmd = calloc(gHandle->queue_size = 64, sizeof(struct colobus_cmd)),"Failed to allocate memory for queue");
    cretassure(gHandle->queueRsp = calloc(gHandle->queue_size = 64, sizeof(uint32_t *)),"Failed to allocate memory for queue");

    {
        int usberr = 0;
        const uint16_t vids[] = { USB_VID, 0 };
        const uint16_t pids[] = { USB_PID, 0 };

        cretassure(jtag_libusb_open(vids, pids, NULL, &gHandle->usb_handle, NULL) == ERROR_OK, "Failed to open or find the device");
        cretassure(!(usberr = libusb_claim_interface(gHandle->usb_handle, COLOBUS_INTERFACE)), "Failed to claim colobus cable interface %s", libusb_strerror(usberr));
    }

    res = ERROR_OK;
error:
    if (err){
        safeFree(gHandle->queueCmd);
        safeFree(gHandle->queueRsp);
        safeFree(gHandle);
    }
    return res;
}

static int colobus_quit(void){
    safeFreeCustom(gHandle->usb_handle, jtag_libusb_close);
	safeFree(gHandle);
	return ERROR_OK;
}

static int colobus_set_freq(int hz){
    struct colobus_cmd ccmd = {
        .id = gHandle->gid++,
        .cmd  = kCOLOBUS_CMD_FREQ,
        .data =  hz/1e3,
	};
    colobus_queue_pushcmd(ccmd, NULL);
    return ERROR_OK;
}

static int colobus_speed_div(int speed, int *khz){
	*khz = speed / 1000;
	return ERROR_OK;
}

static int colobus_khz(int khz, int *jtag_speed){
	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

#pragma mark public interface
struct adapter_driver colobus_adapter_driver = {
	.name = "colobus",
	.commands = command_handlers,
	.transports = colobus_transports,
	.swd_ops = &colobus_swd,
	.init = colobus_init,
	.quit = colobus_quit,
	.speed = colobus_set_freq,
	.speed_div = colobus_speed_div,
	.khz = colobus_khz,
};