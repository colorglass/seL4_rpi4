/* This file has been autogenerated by Ivory
 * Compiler version  0.1.0.5
 */
#ifndef __SERVER_H__
#define __SERVER_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "ivory.h"
#include "input_decrypt2self_dequeue_monitor.h"
#include "input_framing2self_dequeue_monitor.h"
#include "input_vm2self_read_monitor.h"
#include <smaccm_types.h>
void component_entry(const int64_t *n_var0);
void component_init(const int64_t *n_var0);
bool self2encrypt_enqueue(const SMACCM_DATA__GIDL_container *self2encrypt);
bool decrypt2self_dequeue(SMACCM_DATA__GIDL_container *decrypt2self);
bool self2server_enqueue(const SMACCM_DATA__GIDL_container *);
bool self2vm_reboot_enqueue(const bool *);

#ifdef __cplusplus
}
#endif
#endif /* __SERVER_H__ */