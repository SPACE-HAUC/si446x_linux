#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "Si446x.h"

#define print_var(var) \
    printf(#var": 0x%x", info.##var)

int main()
{
    eprintf("Initializing...");
    Si446x_init();
    eprintf("Initialized");
    sleep(1);
    si446x_info_t info;
    memset(&info, 0x0, sizeof(si446x_info_t));
    Si446x_getInfo(&info);
    eprintf("Got info");
    printf("chipRev"": 0x%x", info.chipRev);
    printf("\n");
    printf("partBuild"": 0x%x", info.partBuild);
    printf("\n");
    printf("id"": 0x%x", info.id);
    printf("\n");
    printf("customer"": 0x%x", info.customer);
    printf("\n");
    printf("romId"": 0x%x", info.romId);
    printf("\n");
    printf("revExternal"": 0x%x", info.revExternal);
    printf("\n");
    printf("revBranch"": 0x%x", info.revBranch);
    printf("\n");
    printf("revInternal"": 0x%x", info.revInternal);
    printf("\n");
    printf("patch"": 0x%x", info.patch);
    printf("\n");
    printf("func"": 0x%x", info.func);
    printf("\n");
    eprintf("Press enter for PIPE mode: ");
    int c = 0;
    while (((c = getchar()) != EOF) && (c != '\n'));
    Si446x_en_pipe();
    eprintf("Pipe mode activated");
    eprintf("Press enter to send PIPE mode again or Ctrl + D to exit");
    c = 0;
    while (((c = getchar()) != EOF) && (c != '\n'));
    if (c == EOF)
        return 0;
    else
        Si446x_en_pipe();
    eprintf("Press enter to send 128 bytes of data or Ctrl + D to exit\n");
    c = 0;
    while (((c = getchar()) != EOF) && (c != '\n'));
    char buf[] = "\r\nLet's try sending a text, shall we?\r\nReally trying hard to make this 128 bytes long!\r\nI could try any random text but I won't.";
    Si446x_write(buf, 128); // write all but null character
    eprintf("Press enter to send 128 bytes of data or Ctrl + D to exit\n");
    c = 0;
    while (((c = getchar()) != EOF) && (c != '\n'));
    Si446x_write(buf, 128); // write all but null character
    memset(buf, 0x0, sizeof(buf));
    eprintf("Trying to receive: ");
    short rssi;
    uint8_t rd_sz = Si446x_read(buf, 128, &rssi);
    eprintf("RSSI: %d dBm Read %u bytes: %s", rssi, rd_sz, buf);
    memset(buf, 0x0, sizeof(buf));
    eprintf("Trying to receive: ");
    rd_sz = Si446x_read(buf, 128, &rssi);
    eprintf("RSSI: %d dBm Read %u bytes: %s", rssi, rd_sz, buf);
    return 0;
}