
#ifndef _SD_CARD_H_
#define _SD_CARD_H_

#include <stdint.h>
#include <stdbool.h>

void sd_card_init();
bool sd_card_inited();
uint8_t* sd_card_get_buffer();
void sd_card_set_write_ready();
void sd_card_dump_files();

#endif