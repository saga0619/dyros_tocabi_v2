#include "tocabi_controller/robot_data.h"



class TocabiController
{
public:
    TocabiController(DataContainer &dc_global);
    ~TocabiController();
    void *thread1();
    void *thread2();
    void *thread3();

    DataContainer &dc;

    static void *thread1_helper(void *context) { return ((TocabiController *)context)->thread1(); }
    static void *thread2_helper(void *context) { return ((TocabiController *)context)->thread2(); }
    static void *thread3_helper(void *context) { return ((TocabiController *)context)->thread3(); }



};