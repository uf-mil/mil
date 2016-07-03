//
//  Utility for triggering camera resets using libdc1394
//

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <dc1394/dc1394.h>

int main(int argc, char *argv[])
{
    int i;
    int len = 0;
    long long int temp;
    char guid_str[15];  // GUIDs are 14 characters long

    dc1394_t * d;
    dc1394camera_list_t * list;
    dc1394camera_t *camera;
    dc1394error_t err;
    char *guid_to_kill = argv[1];

    len = strlen(argv[1]);

    d = dc1394_new ();
    if (!d)
        return 1;

    err=dc1394_camera_enumerate (d, &list);
    DC1394_ERR_RTN(err,"Failed to enumerate cameras");

    if (list->num == 0)
    {
        dc1394_log_error("No cameras found");
        return 1;
    }

    for(i = 0; i < list->num; i++)
    {
        sprintf(guid_str, "%llx", list->ids[i].guid);
        if(!strcmp(guid_str, guid_to_kill))
        {
            camera = dc1394_camera_new (d, list->ids[i].guid);
            if (!camera)
            {
                dc1394_log_error("Failed to initialize camera with guid %llx", list->ids[i].guid);
                return 1;
            }
            printf("GUID found - Reseting bus...\n");
            if (dc1394_reset_bus (camera) != DC1394_SUCCESS)
                printf ("Warning: reset reported error\n");

            // Clearing memory for structs
            dc1394_camera_free_list (list);
            dc1394_camera_free (camera);
            dc1394_free (d);
            return 0;
        }
    }
    //printf("Using camera with GUID %"PRIx64"\n", camera->guid);
    dc1394_log_error("No cameras found");
    return 1;
}

