#include "app_sdcard.h"

/* Function to read directory contents and transmit via CDC */
int read_directory(char *path)
{
    DIR dir;
    FILINFO fno;
    FRESULT res;

    if (f_opendir(&dir, path) != FR_OK)
    {
        CDC_Transmit_HS((uint8_t *)"Failed to open directory\n", strlen("Failed to open directory\n"));
        return 1;
    }

    while ((res = f_readdir(&dir, &fno)) == FR_OK && fno.fname[0] != 0)
    {
        if (fno.fattrib & AM_DIR)
        {
            // Directory
            CDC_Transmit_HS((uint8_t *)"\t<DIR> ", strlen("\t<DIR> "));
        }
        else
        {
            // File
            CDC_Transmit_HS((uint8_t *)"\t", strlen("\t"));
        }
        CDC_Transmit_HS((uint8_t *)fno.fname, strlen(fno.fname));
        CDC_Transmit_HS((uint8_t *)"\n", strlen("\n"));
    }

    f_closedir(&dir); // Close the directory

    return 0;
}

/* Function to read a file on the SD card and transmit via CDC */
int read_file(char *filename)
{
    FIL file;
    FRESULT res;
    UINT bytes_read;
    char buffer[512]; // Adjust buffer size as needed

    if (f_open(&file, filename, FA_READ) != FR_OK)
    {
        CDC_Transmit_HS((uint8_t *)"Failed to open file: ", strlen("Failed to open file: "));
        CDC_Transmit_HS((uint8_t *)filename, strlen(filename));
        CDC_Transmit_HS((uint8_t *)"\n", strlen("\n"));
        return 1;
    }

    // Read the file content into the buffer
    while ((res = f_read(&file, buffer, sizeof(buffer), &bytes_read)) == FR_OK && bytes_read > 0)
    {
        // Transmit the read data via CDC
        CDC_Transmit_HS((uint8_t *)buffer, bytes_read);
    }

    if (res != FR_OK)
    {
        CDC_Transmit_HS((uint8_t *)"Error reading file: ", strlen("Error reading file: "));
        CDC_Transmit_HS((uint8_t *)filename, strlen(filename));
        CDC_Transmit_HS((uint8_t *)"\n", strlen("\n"));
        f_close(&file);
        return 1;
    }

    CDC_Transmit_HS((uint8_t *)"Finished reading file: ", strlen("Finished reading file: "));
    CDC_Transmit_HS((uint8_t *)filename, strlen(filename));
    CDC_Transmit_HS((uint8_t *)"\n", strlen("\n"));
    f_close(&file);

    return 0;
}

int get_sdcard_status()
{

    if (BSP_PlatformIsDetected() == SD_PRESENT)
    {
        CDC_Transmit_HS((uint8_t *)"BSP: SD card inserted\n", strlen("BSP: SD card inserted\n"));
    }
    else
    {
        CDC_Transmit_HS((uint8_t *)"BSP: SD card not inserted\n", strlen("BSP: SD card not inserted\n"));
    }

    FRESULT res;

    // Attempt to mount the SD card
    res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1);

    if (res == FR_OK)
    {
        // SD card mounted successfully
        CDC_Transmit_HS((uint8_t *)"SD card inserted\n", strlen("SD card inserted\n"));

        // read_file("/hello.txt");
    }
    else
    {
        // SD card not mounted or error
        switch (res)
        {
        case FR_NO_FILESYSTEM:
            CDC_Transmit_HS((uint8_t *)"SD card not formatted\n", strlen("SD card not formatted\n"));
            break;
        case FR_INT_ERR:
            CDC_Transmit_HS((uint8_t *)"SD card internal error\n", strlen("SD card internal error\n"));
            break;
        case FR_NO_PATH:
            CDC_Transmit_HS((uint8_t *)"SD card mount path error\n", strlen("SD card mount path error\n"));
            break;
        default:
            // Handle other potential errors
            char message[32];
            sprintf(message, "SD card error: %d\n", res);
            CDC_Transmit_HS((uint8_t *)message, strlen(message));
        }
    }
}