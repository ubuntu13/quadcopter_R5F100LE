#ifndef KEY_H
#define KEY_H

#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "r_cg_port.h"

#define key1 P7.4
#define key2 P7.2

uint8_t key_scan(void);

#endif