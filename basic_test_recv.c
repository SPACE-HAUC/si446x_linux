#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "si446x.h"

#define print_var(var) \
    printf(#var": 0x%x", info.##var)

int main()
{
    eprintf("Initializing...");
    si446x_init();
    eprintf("Initialized");
    sleep(1);
    si446x_info_t info;
    memset(&info, 0x0, sizeof(si446x_info_t));
    si446x_getInfo(&info);
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
    si446x_en_pipe();
    eprintf("Pipe mode activated");
    eprintf("Press enter to send PIPE mode again or Ctrl + D to exit");
    c = 0;
    while (((c = getchar()) != EOF) && (c != '\n'));
    if (c == EOF)
        return 0;
    else
        si446x_en_pipe();
    while (1)
    {
        char buf[128];
        memset(buf, 0, sizeof(buf));
        eprintf("Press enter to receive or Ctrl + D to exit\n");
        // c = 0;
        // while (((c = getchar()) != EOF) && (c != '\n'));
        // if (c == EOF)
        //     break;
        printf("Read %d bytes: ", si446x_read(buf, sizeof(buf), NULL));
        for (int i = 0; i < sizeof(buf); i++)
            if (buf[i] != 0) printf("%c", buf[i]);
        printf("\n");
    }
    return 0;
}
