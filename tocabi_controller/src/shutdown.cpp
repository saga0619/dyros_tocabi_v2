#include "shm_msgs.h"

int main()
{
  int shm_id;
  SHMmsgs *shm_msgs_;

  init_shm(shm_msg_key, shm_id, &shm_msgs_);

  shm_msgs_->shutdown = true;

  deleteSharedMemory(shm_id, shm_msgs_);

  return 0;
}
