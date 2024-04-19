#ifndef INC_UDP_HANDLER_H_
#define INC_UDP_HANDLER_H_

#include <udp.h>
err_t udp_send_message(struct udp_pcb *upcb,const ip_addr_t *addr, u16_t port, const char *dataSource);
#endif
