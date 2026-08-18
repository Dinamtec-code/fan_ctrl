#ifndef USB_TMC_PROCESS_H_
#define USB_TMC_PROCESS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define IEEE4882_STB_QUESTIONABLE (0x08u)
#define IEEE4882_STB_MAV (0x10u)
#define IEEE4882_STB_SER (0x20u)
#define IEEE4882_STB_SRQ (0x40u)
    /* Status Byte Masks (IEEE 488.2) */
    typedef enum
    {
        STB_NONE = 0x00,
        STB_MAV_BIT = IEEE4882_STB_MAV, // Message Available
        STB_ESB_BIT = IEEE4882_STB_SER, // Event Status Bit
        STB_RQS_BIT = IEEE4882_STB_SRQ, // Request Service
        STB_QST_BIT = IEEE4882_STB_QUESTIONABLE
    } usb_tmc_status_t;

    /* ESTADOS DE LA FSM */
    typedef enum
    {
        STATE_TMC_IDLE = 0,
        STATE_TMC_RECEIVING,
        STATE_TMC_PROCESSING,
        STATE_TMC_REPLY_READY
    } usb_tmc_state_t;

    /* EVENTOS CONTROLANDOS POR LA FSM */
    typedef enum
    {
        EV_TMC_RX_START, // Inicia recepción (Bulk OUT)
        EV_TMC_RX_CHUNK, // Llega un pedazo de datos
        EV_TMC_RX_END,   // Llega el final del mensaje
        EV_TMC_TX_REQ,   // Host pide leer (Bulk IN)
        EV_TMC_TX_DONE,  // Host terminó de leer
        EV_TMC_SCPI_DONE // libscpi no encontró query
    } usb_tmc_event_t;
    
    bool tmc_scpi_has_errors(void);
    bool tmc_scpi_has_connected(void);
    void scpi_engine_init(void);
    void usb_tmc_fsm_process(usb_tmc_event_t event, void *data, size_t len);
    usb_tmc_status_t usb_tmc_get_stb(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_TMC_FSM_PROCESS_H_ */