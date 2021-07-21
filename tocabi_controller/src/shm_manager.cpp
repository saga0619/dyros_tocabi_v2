
#include <iostream>
#include "shm_msgs.h"

int main(void)
{
    int shm_id_;

    if ((shm_id_ = shmget(shm_msg_key, 0, IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed " << std::endl;
    }

    std::cout << "succesfully received shared memory id! : " << shm_id_ << std::endl;

    if (shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        printf("shared memoty failed to remove. maybe you need sudo?\n");
    }
    else
    {
        printf("Shared memory succesfully removed\n");
    }



    if ((shm_id_ = shmget(shm_rd_key, 0, IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed " << std::endl;
    }

    std::cout << "succesfully received shared memory id! : " << shm_id_ << std::endl;

    if (shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        printf("shared memoty failed to remove. maybe you need sudo?\n");
    }
    else
    {
        printf("Shared memory succesfully removed\n");
    }

    return 0;
}