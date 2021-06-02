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
    return 0;
}