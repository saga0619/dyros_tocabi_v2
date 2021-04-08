#include "tocabi_controller/tocabi_controller.h"

using namespace std;

TocabiController::TocabiController(DataContainer &dc_global, StateManager &stm_global) : dc(dc_global), stm(stm_global)
{
    cout << "TocabiController Initialized" << endl;
}

TocabiController::~TocabiController()
{
    cout << "TocabiController Terminated" << endl;
}

void *TocabiController::thread1()
{

}

void *TocabiController::thread2()
{

}

void *TocabiController::thread3()
{

}

/*
void TocabiController::SetCommand()
{

}*/