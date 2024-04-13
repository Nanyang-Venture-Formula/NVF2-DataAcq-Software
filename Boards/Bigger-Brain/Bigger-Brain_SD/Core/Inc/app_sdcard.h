/*
** EPITECH PROJECT, 2024
** Bigger-Brain_SD
** File description:
** app_sdcard
*/

#ifndef APP_SDCARD_H_
#define APP_SDCARD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "usbd_cdc_if.h"
#include "ffconf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int read_directory(char *path);
int read_file(char *filename);
int get_sdcard_status();

#ifdef __cplusplus
}
#endif

#endif /* !APP_SDCARD_H_ */
