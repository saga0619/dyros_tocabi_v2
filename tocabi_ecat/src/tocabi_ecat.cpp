#include "tocabi_ecat/tocabi_ecat.h"

int64 toff, gl_delta;

void ethercatCheck()
{
    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
        if (needlf)
        {
            printf("S\n");
            needlf = FALSE;
            printf("\n");
        }
        // one ore more slaves are not responding
        ec_group[currentgroup].docheckstate = FALSE;
        ec_readstate();
        for (int slave = 1; slave <= ec_slavecount; slave++)
        {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
                ec_group[currentgroup].docheckstate = TRUE;
                if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                {
                    printf("%sERROR : slave %d is in SAFE_OP + ERROR, attempting ack.%s\n", cred.c_str(), slave - 1, creset.c_str());
                    ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                    ec_writestate(slave);
                }
                else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                {
                    printf("%sWARNING : slave %d is in SAFE_OP, change to OPERATIONAL.%s\n", cred.c_str(), slave - 1, creset.c_str());
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                }
                else if (ec_slave[slave].state > 0)
                {
                    if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("%sMESSAGE : slave %d reconfigured%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                    }
                }
                else if (!ec_slave[slave].islost)
                {
                    // re-check state
                    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                    if (!ec_slave[slave].state)
                    {
                        ec_slave[slave].islost = TRUE;
                        printf("%sERROR : slave %d lost %s\n", cred.c_str(), slave - 1, creset.c_str());
                    }
                }
            }
            if (ec_slave[slave].islost)
            {
                if (!ec_slave[slave].state)
                {
                    if (ec_recover_slave(slave, EC_TIMEOUTMON))
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("%sMESSAGE : slave %d recovered%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                    }
                }
                else
                {
                    ec_slave[slave].islost = FALSE;
                    printf("%sMESSAGE : slave %d found%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                }
            }
        }
    }
}
void elmoInit()
{
    elmofz[R_Armlink_Joint].init_direction = -1.0;
    elmofz[L_Armlink_Joint].init_direction = -1.0;
    elmofz[R_Elbow_Joint].init_direction = -1.0;
    elmofz[Upperbody_Joint].init_direction = -1.0;

    elmofz[Waist2_Joint].init_direction = -1.0;

    elmofz[R_Elbow_Joint].req_length = 0.06;
    elmofz[L_Elbow_Joint].req_length = 0.09;
    elmofz[L_Forearm_Joint].req_length = 0.09;
    elmofz[R_Forearm_Joint].req_length = 0.14;

    elmofz[L_Shoulder1_Joint].req_length = 0.18;
    elmofz[L_Shoulder2_Joint].req_length = 0.15;
    elmofz[R_Shoulder2_Joint].req_length = 0.08;

    elmofz[R_Shoulder3_Joint].req_length = 0.03;
    elmofz[L_Shoulder3_Joint].req_length = 0.04;

    elmofz[R_Wrist2_Joint].req_length = 0.05;
    elmofz[L_Wrist2_Joint].req_length = 0.05;

    elmofz[Waist2_Joint].req_length = 0.07;
    elmofz[Waist2_Joint].init_direction = -1.0;
    elmofz[Waist1_Joint].req_length = 0.07;

    q_zero_mod_elmo_[8] = 15.46875 * DEG2RAD;
    q_zero_mod_elmo_[7] = 16.875 * DEG2RAD;
    q_zero_mod_elmo_[Waist1_Joint] = -15.0 * DEG2RAD;
    q_zero_mod_elmo_[Upperbody_Joint] = 0.0541;

    memset(ElmoSafteyMode, 0, sizeof(int) * ec_slavecount);
}

void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

void *ethercatThread1(void *data)
{
    char IOmap[4096] = {};
    bool reachedInitial[ELMO_DOF] = {false};

    const char *ifname = soem_port.c_str();

    if (ec_init(ifname))
    {
        printf("ELMO : ec_init on %s succeeded.\n", ifname);
        elmoInit();
        //initSharedMemory();

        init_shm(shm_msg_key, shm_id_, &shm_msgs_);

        /* find and auto-config slaves */
        /* network discovery */
        //ec_config_init()
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("ELMO : %d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num
            if (ec_slavecount == expected_counter)
            {
                ecat_number_ok = true;
            }
            else
            {
                std::cout << cred << "WARNING : SLAVE NUMBER INSUFFICIENT" << creset << std::endl;
                de_shutdown = true;
            }
            /** CompleteAccess disabled for Elmo driver */
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //printf("ELMO : Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
                {
                    printf("ELMO : slave[%d] CA? : false , shutdown request \n ", slave);
                }
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //0x1605 :  Target Position             32bit
                //          Target Velocity             32bit
                //          Max Torque                  16bit
                //          Control word                16bit
                //          Modes of Operation          16bit
                uint16 map_1c12[2] = {0x0001, 0x1605};

                //0x1a00 :  position actual value       32bit
                //          Digital Inputs              32bit
                //          Status word                 16bit
                //0x1a11 :  velocity actual value       32bit
                //0x1a13 :  Torque actual value         16bit
                //0x1a1e :  Auxiliary position value    32bit
                uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
                //uint16 map_1c13[6] = {0x0005, 0x1a04, 0x1a11, 0x1a12, 0x1a1e, 0X1a1c};
                int os;
                os = sizeof(map_1c12);
                ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, map_1c12, EC_TIMEOUTRXM);
                os = sizeof(map_1c13);
                ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, map_1c13, EC_TIMEOUTRXM);
            }
            /** if CA disable => automapping works */
            ec_config_map(&IOmap);

            ec_configdc();

            /* wait for all slaves to reach SAFE_OP state */
            printf("ELMO : EC WAITING STATE TO SAFE_OP\n");
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("ELMO : Request operational state for all slaves. Calculated workcounter : %d\n", expectedWKC);

            if (expectedWKC != 3 * expected_counter)
            {
                std::cout << cred << "WARNING : Calculated Workcounter insufficient!" << creset << std::endl;
                ecat_WKC_ok = true;
            }

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);

            int wait_cnt = 200;

            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);

            } while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                inOP = TRUE;

                /* cyclic loop */
                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    txPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
                    rxPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
                }

                //Commutation Checking
                st_start_time = std::chrono::steady_clock::now();
                cout << "ELMO : Initialization Mode" << endl;

                query_check_state = true;

                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);

                ts.tv_nsec += PERIOD_NS;
                while (ts.tv_nsec >= SEC_IN_NSEC)
                {
                    ts.tv_sec++;
                    ts.tv_nsec -= SEC_IN_NSEC;
                }

                while (!de_shutdown)
                {
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

                    if (force_control_mode)
                    {
                        break;
                    }

                    ts.tv_nsec += PERIOD_NS;
                    while (ts.tv_nsec >= SEC_IN_NSEC)
                    {
                        ts.tv_sec++;
                        ts.tv_nsec -= SEC_IN_NSEC;
                    }
                    //std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);
                    cycle_count++;
                    wkc = ec_receive_processdata(0);
                    control_time_real_ = std::chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - st_start_time).count() / 1000000.0;

                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        elmost[i].state = getElmoState(rxPDO[i]->statusWord);

                        if (elmost[i].state != elmost[i].state_before)
                        {
                            if (elmost[i].first_check)
                            {
                                if (elmost[i].state == ELMO_NOTFAULT)
                                {
                                    elmost[i].commutation_required = true;
                                    joint_state_elmo_[i] = ESTATE::COMMUTATION_INITIALIZE;
                                }
                                else if (elmost[i].state == ELMO_FAULT)
                                {
                                    //cout << "slave : " << i << " commutation check complete at first" << endl;
                                    elmost[i].commutation_not_required = true;
                                    joint_state_elmo_[i] = ESTATE::COMMUTATION_DONE;
                                }
                                else if (elmost[i].state == ELMO_OPERATION_ENABLE)
                                {
                                    //cout << "slave : " << i << " commutation check complete with operation enable" << endl;
                                    elmost[i].commutation_not_required = true;
                                    joint_state_elmo_[i] = ESTATE::COMMUTATION_DONE;
                                    elmost[i].commutation_ok = true;
                                }
                                else
                                {
                                    //cout << "first missing : slave : " << i << " state : " << elmost[i].state << endl;
                                }
                                elmost[i].first_check = false;
                            }
                            else
                            {
                                if (elmost[i].state == ELMO_OPERATION_ENABLE)
                                {
                                    joint_state_elmo_[i] = ESTATE::COMMUTATION_DONE;
                                    //cout << "slave : " << i << " commutation check complete with operation enable 2" << endl;
                                    elmost[i].commutation_ok = true;
                                    elmost[i].commutation_required = false;
                                }
                            }
                            query_check_state = true;
                        }
                        elmost[i].state_before = elmost[i].state;
                    }

                    if (check_commutation)
                    {
                        if (check_commutation_first)
                        {
                            cout << "Commutation Status : " << endl;
                            for (int i = 0; i < ec_slavecount; i++)
                                printf("--");
                            cout << endl;
                            for (int i = 0; i < ec_slavecount; i++)
                                printf("%2d", (i - i % 10) / 10);
                            printf("\n");
                            for (int i = 0; i < ec_slavecount; i++)
                                printf("%2d", i % 10);
                            cout << endl;
                            cout << endl;
                            cout << endl;
                            check_commutation_first = false;
                        }
                        if (query_check_state)
                        {
                            printf("\x1b[A\x1b[A\33[2K\r");
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (elmost[i].state == ELMO_OPERATION_ENABLE)
                                {
                                    printf("%s%2d%s", cgreen.c_str(), elmost[i].state, creset.c_str());
                                }
                                else
                                {
                                    printf("%2d", elmost[i].state);
                                }
                            }
                            cout << endl;
                            for (int i = 0; i < ec_slavecount; i++)
                                printf("--");
                            cout << endl;
                            fflush(stdout);
                            query_check_state = false;
                        }
                    }

                    bool waitop = true;
                    for (int i = 0; i < ec_slavecount; i++)
                        waitop = waitop && elmost[i].commutation_ok;

                    if (waitop)
                    {
                        static bool pub_once = true;

                        if (pub_once)
                        {
                            std::chrono::milliseconds commutation_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - st_start_time);
                            cout << "ELMO : All slaves Operational in " << commutation_time.count() << "ms" << endl;
                            if (commutation_time.count() < 500)
                            {

                                de_commutation_done = true;
                                check_commutation = false;
                                cout << "ELMO : Load ZP ... " << endl;
                            }
                            else
                            {
                                if (saveCommutationLog())
                                {
                                    cout << "\nELMO : Commutation is done, logging success" << endl;
                                }
                                else
                                {
                                    cout << "\nELMO : Commutaion is done, logging failed" << endl;
                                }
                                de_commutation_done = true;
                                check_commutation = false;
                            }

                            pub_once = false;
                        }
                    }

                    if (de_commutation_done)
                    {
                        static bool pub_once = true;
                        if (pub_once)
                        {

                            if (loadZeroPoint())
                            {
                                cout << "ELMO : Initialize Complete " << endl;
                                break;
                            }
                            else
                            {
                                cout << "ELMO : ZeroPoint load failed. Ready to Search Zero Point " << endl;
                                de_zp_sequence = true;
                            }
                            pub_once = false;
                        }
                    }

                    bool waitcm = true;
                    for (int i = 0; i < ec_slavecount; i++)
                        waitcm = waitcm && elmost[i].commutation_not_required;

                    if (waitcm)
                    {
                        if (wait_kill_switch)
                        {
                            cout << "ELMO : Commutation state OK" << endl;
                            //loadCommutationLog();
                            loadZeroPoint();
                            wait_kill_switch = false;
                            check_commutation = false;
                        }
                        if (wait_cnt == 200)
                        {
                            cout << "ELMO : slaves status are not OP! maybe kill switch is on?" << endl;
                        }

                        wait_cnt++;
                    }
                    else
                    {
                        int total_commutation_cnt = 0;
                        for (int i = 0; i < ec_slavecount; i++)
                        {
                            if (elmost[i].commutation_required)
                            {
                                total_commutation_cnt++;
                                if (total_commutation_cnt < 10)
                                    controlWordGenerate(rxPDO[i]->statusWord, txPDO[i]->controlWord);
                                txPDO[i]->maxTorque = (uint16)1000; // originaly 1000
                            }
                        }
                    }

                    for (int slave = 1; slave <= ec_slavecount; slave++)
                    {
                        if (!elmost[slave - 1].commutation_required)
                        {
                            if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                            {
                                reachedInitial[slave - 1] = true;
                            }

                            if (reachedInitial[slave - 1])
                            {
                                q_elmo_[slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[slave - 1] * elmo_axis_direction[slave - 1];
                                hommingElmo[slave - 1] =
                                    (((uint32_t)ec_slave[slave].inputs[4]) +
                                     ((uint32_t)ec_slave[slave].inputs[5] << 8) +
                                     ((uint32_t)ec_slave[slave].inputs[6] << 16) +
                                     ((uint32_t)ec_slave[slave].inputs[7] << 24));
                                q_dot_elmo_[slave - 1] =
                                    (((int32_t)ec_slave[slave].inputs[10]) +
                                     ((int32_t)ec_slave[slave].inputs[11] << 8) +
                                     ((int32_t)ec_slave[slave].inputs[12] << 16) +
                                     ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                                    CNT2RAD[slave - 1] * elmo_axis_direction[slave - 1];
                                torque_elmo_[slave - 1] =
                                    (((int16_t)ec_slave[slave].inputs[14]) +
                                     ((int16_t)ec_slave[slave].inputs[15] << 8));
                                q_ext_elmo_[slave - 1] =
                                    (((int32_t)ec_slave[slave].inputs[16]) +
                                     ((int32_t)ec_slave[slave].inputs[17] << 8) +
                                     ((int32_t)ec_slave[slave].inputs[18] << 16) +
                                     ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[slave - 1]) *
                                    EXTCNT2RAD[slave - 1] * elmo_ext_axis_direction[slave - 1];
                                if (slave == 1 || slave == 2 || slave == 19 || slave == 20 || slave == 16)
                                {
                                    hommingElmo[slave - 1] = !hommingElmo[slave - 1];
                                }
                                txPDO[slave - 1]->maxTorque = (uint16)500; // originaly 1000
                            }
                        }
                    }

                    if (de_zp_sequence)
                    {
                        static bool zp_upper = false;
                        static bool zp_lower = false;

                        if (de_zp_upper_switch)
                        {
                            cout << "starting upper zp" << endl;
                            for (int i = 0; i < 8; i++)
                                cout << "L" << i << "\t";
                            for (int i = 0; i < 8; i++)
                                cout << "R" << i << "\t";
                            cout << endl;
                            elmofz[R_Shoulder3_Joint].findZeroSequence = 7;
                            elmofz[R_Shoulder3_Joint].initTime = control_time_real_;
                            elmofz[L_Shoulder3_Joint].findZeroSequence = 7;
                            elmofz[L_Shoulder3_Joint].initTime = control_time_real_;

                            for (int i = 0; i < ec_slavecount; i++)
                                hommingElmo_before[i] = hommingElmo[i];

                            zp_upper = true;
                            de_zp_upper_switch = false;
                        }

                        if (de_zp_lower_switch)
                        {
                            cout << "starting lower zp" << endl;
                            de_zp_lower_switch = false;
                            zp_lower = true;
                        }

                        if (zp_upper)
                        {
                            if (fz_group == 0)
                            {
                                for (int i = 0; i < 18; i++)
                                {
                                    findZeroPoint(fz_group1[i]);
                                }
                            }
                            else if (fz_group == 1)
                            {
                                for (int i = 0; i < 3; i++)
                                {
                                    findZeroPoint(fz_group2[i]);
                                }
                            }
                            for (int i = 0; i < ec_slavecount; i++)
                                hommingElmo_before[i] = hommingElmo[i];
                        }

                        if (zp_lower)
                        {
                            if (zp_lower_calc)
                            {
                                findzeroLeg();
                                zp_lower_calc = false;
                            }
                            else
                            {
                                for (int i = 0; i < 6; i++)
                                {
                                    findZeroPointlow(i + R_HipYaw_Joint);
                                    findZeroPointlow(i + L_HipYaw_Joint);
                                }
                            }
                        }

                        fz_group1_check = true;
                        for (int i = 0; i < 18; i++)
                        {
                            fz_group1_check = fz_group1_check && (elmofz[fz_group1[i]].result == ElmoHommingStatus::SUCCESS);
                        }

                        fz_group2_check = true;
                        for (int i = 0; i < 3; i++)
                        {
                            fz_group2_check = fz_group2_check && (elmofz[fz_group2[i]].result == ElmoHommingStatus::SUCCESS);
                        }
                        fz_group3_check = true;
                        for (int i = 0; i < 12; i++)
                        {
                            fz_group3_check = fz_group3_check && (elmofz[fz_group3[i]].result == ElmoHommingStatus::SUCCESS);
                        }

                        if (fz_group1_check && (fz_group == 0))
                        {
                            cout << "ELMO : arm zp done " << endl;
                            fz_group++;
                        }
                        if (fz_group2_check && (fz_group == 1))
                        {
                            fz_group++;
                            cout << "ELMO : waist zp done" << endl;
                        }

                        static bool low_verbose = true;
                        if (low_verbose && fz_group3_check)
                        {
                            cout << "ELMO : lowerbody zp done " << endl;
                            low_verbose = false;
                        }

                        if (fz_group1_check && fz_group2_check)
                        {
                            if (!fz_group3_check)
                            {
                                cout << "ELMO : lowerbody zp by 0 point " << endl;
                            }
                            else
                            {
                                cout << "ELMO : lowerbody zp by ext encoder " << endl;
                            }

                            if (saveZeroPoint())
                            {
                                cout << "ELMO : zeropoint searching complete, saved " << endl;
                                de_zp_sequence = false;
                                break;
                            }
                            else
                            {
                                cout << "ELMO : zeropoint searching complete, save failed" << endl;
                                de_zp_sequence = false;
                                break;
                            }
                        }
                    }

                    //command
                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        if (ElmoMode[i] == EM_POSITION)
                        {
                            txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                            txPDO[i]->targetPosition = (int)(elmo_axis_direction[i] * RAD2CNT[i] * q_desired_elmo_[i]);

                            //if (i == 0)
                            //   cout << i << " : " << txPDO[i]->targetPosition << endl;
                        }
                        else if (ElmoMode[i] == EM_TORQUE)
                        {
                            txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;

                            txPDO[i]->targetTorque = (int)(torque_desired_elmo_[i] * NM2CNT[i] * elmo_axis_direction[i]);
                        }
                        else if (ElmoMode[i] == EM_COMMUTATION)
                        {
                        }
                        else
                        {
                            txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                            txPDO[i]->targetTorque = (int)0;
                        }
                    }

                    ec_send_processdata();
                }

                cout << "ELMO : Control Mode Start ... " << endl;

                memset(joint_state_elmo_, ESTATE::OPERATION_READY, sizeof(int) * ec_slavecount);
                st_start_time = std::chrono::steady_clock::now();
                cycle_count = 1;
                ////////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////////
                //Starting
                ///////////////////////////////////////////////////////////////////////////////////////////

                int64_t total1, total2;
                int64_t total_dev1, total_dev2;
                float lmax, lmin, ldev, lavg, lat;
                float smax, smin, sdev, savg, sat;

                total1 = 0;
                total2 = 0;
                total_dev1 = 0;
                total_dev2 = 0;

                ldev = 0.0;
                lavg = 0.0;
                lat = 0;

                lmax = 0.0;
                lmin = 10000.00;
                smax = 0.0;
                smin = 100000.0;

                sdev = 0;
                savg = 0;
                sat = 0;

                const int PRNS = period_ns;

                clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_nsec += PRNS;
                while (ts.tv_nsec >= SEC_IN_NSEC)
                {
                    ts.tv_sec++;
                    ts.tv_nsec -= SEC_IN_NSEC;
                }

                struct timespec ts1, ts2;

                while (!de_shutdown)
                {
                    chrono::steady_clock::time_point rcv_ = chrono::steady_clock::now();

                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

                    clock_gettime(CLOCK_MONOTONIC, &ts1);

                    lat = ts1.tv_nsec - ts.tv_nsec;
                    if (lat < 0)
                    {
                        lat += SEC_IN_NSEC;
                    }

                    ts.tv_nsec += PRNS;
                    while (ts.tv_nsec >= SEC_IN_NSEC)
                    {
                        ts.tv_sec++;
                        ts.tv_nsec -= SEC_IN_NSEC;
                    }

                    chrono::steady_clock::time_point rcv2_ = chrono::steady_clock::now();
                    //std::this_thread::sleep_for(std::chrono::microseconds(30));

                    /** PDO I/O refresh */
                    //ec_send_processdata();
                    wkc = ec_receive_processdata(200);

                    if (wkc >= expectedWKC)
                    {
                        for (int slave = 1; slave <= ec_slavecount; slave++)
                        {
                            if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                            {
                                reachedInitial[slave - 1] = true;
                            }
                            if (reachedInitial[slave - 1])
                            {
                                q_elmo_[slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[slave - 1] * elmo_axis_direction[slave - 1];
                                hommingElmo[slave - 1] =
                                    (((uint32_t)ec_slave[slave].inputs[4]) +
                                     ((uint32_t)ec_slave[slave].inputs[5] << 8) +
                                     ((uint32_t)ec_slave[slave].inputs[6] << 16) +
                                     ((uint32_t)ec_slave[slave].inputs[7] << 24));
                                q_dot_elmo_[slave - 1] =
                                    (((int32_t)ec_slave[slave].inputs[10]) +
                                     ((int32_t)ec_slave[slave].inputs[11] << 8) +
                                     ((int32_t)ec_slave[slave].inputs[12] << 16) +
                                     ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                                    CNT2RAD[slave - 1] * elmo_axis_direction[slave - 1];
                                torque_elmo_[slave - 1] =
                                    (((int16_t)ec_slave[slave].inputs[14]) +
                                     ((int16_t)ec_slave[slave].inputs[15] << 8));
                                q_ext_elmo_[slave - 1] =
                                    (((int32_t)ec_slave[slave].inputs[16]) +
                                     ((int32_t)ec_slave[slave].inputs[17] << 8) +
                                     ((int32_t)ec_slave[slave].inputs[18] << 16) +
                                     ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[slave - 1]) *
                                    EXTCNT2RAD[slave - 1] * elmo_ext_axis_direction[slave - 1];
                                if (slave == 1 || slave == 2 || slave == 19 || slave == 20 || slave == 16)
                                {
                                    hommingElmo[slave - 1] = !hommingElmo[slave - 1];
                                }
                                txPDO[slave - 1]->maxTorque = (uint16)0; // originaly 1000
                            }
                        }
                    }

                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        q_[i] = q_elmo_[JointMap[i]];
                        q_dot_[i] = q_dot_elmo_[JointMap[i]];
                        torque_[i] = torque_elmo_[JointMap[i]];
                        q_ext_[i] = q_ext_elmo_[JointMap[i]];
                        joint_state_[i] = joint_state_elmo_[JointMap[i]];
                    }

                    sendJointStatus();

                    //getJointCommand();

                    //ECAT JOINT COMMAND
                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        if (ElmoMode[i] == EM_POSITION)
                        {
                            txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                            txPDO[i]->targetPosition = (int)(elmo_axis_direction[i] * RAD2CNT[i] * q_desired_elmo_[i]);
                        }
                        else if (ElmoMode[i] == EM_TORQUE)
                        {
                            txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                            txPDO[i]->targetTorque = (int)(torque_desired_elmo_[i] * NM2CNT[i] * elmo_axis_direction[i]);
                            /*
                            if (dc.customGain)
                            {
                                txPDO[i]->targetTorque = (int)(ELMO_torque[i] * CustomGain[i] * elmo_axis_direction[i]);
                            }
                            else
                            {
                                txPDO[i]->targetTorque = (roundtoint)(ELMO_torque[i] * ELMO_NM2CNT[i] * elmo_axis_direction[i]);
                            }*/
                        }
                        else
                        {
                            txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                            txPDO[i]->targetTorque = (int)0;
                        }
                    }
                    /*
                            bool ecat_lost_before = de_ecat_lost;
                            de_ecat_lost = false;
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ec_slave[i].islost)
                                {
                                    de_ecat_lost = de_ecat_lost || true;
                                }
                            }

                            if ((ecat_lost_before) && (!de_ecat_lost))
                            {
                                de_ecat_recovered = true;
                            }

                            if (de_ecat_lost)
                            {
                                for (int i = 0; i < ec_slavecount; i++)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }

                            //Hold position if safety limit breached
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ElmoMode[i] != EM_POSITION)
                                {
                                    checkPosSafety[i] = false;
                                }

                                checkSafety(i, joint_velocity_limit[i], 10.0 * CYCLETIME / 1E+6); //if angular velocity exceeds 0.5rad/s, Hold to current Position ///
                            }
                            */

                    //Torque off if emergency off received
                    if (de_emergency_off)
                        emergencyOff();

                    ec_send_processdata();

                    sat = chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - rcv2_).count();

                    //lat = latency1.count();
                    total1 += lat;

                    lavg = total1 / cycle_count;
                    if (lmax < lat)
                    {
                        lmax = lat;
                    }
                    if (lmin > lat)
                    {
                        lmin = lat;
                    }
                    total_dev1 += sqrt(((lat - lavg) * (lat - lavg)));
                    ldev = total_dev1 / cycle_count;

                    shm_msgs_->lat_avg = lavg;
                    shm_msgs_->lat_max = lmax;
                    shm_msgs_->lat_min = lmin;
                    shm_msgs_->lat_dev = ldev;

                    //sat = latency2.count();
                    total2 += sat;
                    savg = total2 / cycle_count;
                    if (smax < sat)
                    {
                        smax = sat;
                    }
                    if (smin > sat)
                    {
                        smin = sat;
                    }
                    // int sdev = (sat - savg)
                    total_dev2 += sqrt(((sat - savg) * (sat - savg)));
                    sdev = total_dev2 / cycle_count;

                    shm_msgs_->send_avg = savg;
                    shm_msgs_->send_max = smax;
                    shm_msgs_->send_min = smin;
                    shm_msgs_->send_dev = sdev;

                    cycle_count++;

                    if (de_debug_level == 1)
                    {
                        printf("\x1b[A\x1b[A\x1b[A\33[2K\r");

                        printf("Current Count : %d\n", cycle_count);
                        printf("Lat Act : %7.3f Min : %7.3f Max : %7.3f Avg : %7.3f Dev : %7.3f\n", lat / 1000.0, lmin / 1000.0, lmax / 1000.0, lavg / 1000.0, ldev / 1000.0);
                        printf("Sen Act : %7.3f Min : %7.3f Max : %7.3f Avg : %7.3f Dev : %7.3f\n", sat / 1000.0, smin / 1000.0, smax / 1000.0, savg / 1000.0, sdev / 1000.0);

                        fflush(stdout);
                    }
                    /*
                            if (dc.disableSafetyLock)
                            {
                                for (int i = 0; i < ec_slavecount; i++)
                                {
                                    ElmoSafteyMode[i] = 0;
                                }
                                dc.disableSafetyLock = false;
                            }
                            

                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        if (ElmoMode[i] == EM_POSITION)
                        {
                            checkPosSafety[i] = true;
                        }
                    }

                     */
                }

                inOP = FALSE;
            }
            else
            {
                printf("%sELMO : Not all slaves reached operational state.%s\n", cred.c_str(), creset.c_str());
                ec_readstate();
                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    if (ec_slave[slave - 1].state != EC_STATE_OPERATIONAL)
                    {
                        printf("%sELMO : EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s%s\n",
                               cred.c_str(), slave - 1, ec_slave[slave - 1].state, ec_slave[slave - 1].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[slave - 1].ALstatuscode), creset.c_str());
                    }
                }
            }
            printf("\nELMO : Request init state for all slaves\n");
            /** request INIT state for all slaves
                    *  slave number = 0 -> write to all slaves
                    */
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);

            printf("ELMO : Checking EC STATE ... \n");
            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            printf("ELMO : Checking EC STATE Complete \n");
        }
        else
        {
            printf("%sELMO : No slaves found!%s\n", cred.c_str(), creset.c_str());
        }
    }
    else
    {
        printf("ELMO : No socket connection on %s\nExcecute as root\n", ifname);
    }

    deleteSharedMemory(shm_id_, shm_msgs_);
    std::cout << "ELMO : EthercatThread1 Shutdown" << std::endl;
}

void *ethercatThread2(void *data)
{
    while (!de_shutdown)
    {
        this_thread::sleep_for(std::chrono::milliseconds(10));
        ethercatCheck();

        int ch = kbhit();
        if (ch != -1)
        {
            //std::cout << "key input : " << (char)(ch % 256) << std::endl;
            if ((ch % 256 == 'q'))
            {
                std::cout << "ELMO : shutdown request" << std::endl;
                de_shutdown = true;
                shm_msgs_->shutdown = true;
            }
            else if ((ch % 256 == 'l'))
            {
                std::cout << "ELMO : start searching zero point lower" << std::endl;
                de_zp_lower_switch = true;
            }
            else if ((ch % 256 == 'u'))
            {
                std::cout << "ELMO : start searching zero point upper" << std::endl;
                de_zp_upper_switch = true;
            }
            else if ((ch % 256 == 'd'))
            {
                de_debug_level++;
                if (de_debug_level > 2)
                    de_debug_level = 0;

                std::cout << "ELMO : debug mode, level : " << (int)de_debug_level << std::endl;
            }
            else if ((ch % 256 == 'p'))
            {
                std::cout << "------------------------------------------------------" << std::endl;
                for (int i = 0; i < ec_slavecount; i++)
                { //std::cout << i << ELMO_NAME[i] <<
                    printf("%4d   %20s  %12f  ext : %12f\n", i, ELMO_NAME[i].c_str(), (double)q_elmo_[i], (double)q_ext_elmo_[i]);
                }
            }
            else if ((ch % 256 == 'h'))
            {
                for (int i = 0; i < ec_slavecount; i++)
                    std::cout << i << ELMO_NAME[i] << "\t" << hommingElmo[i] << std::endl;
            }
            else if ((ch % 256 == 'c'))
            {
                std::cout << "Force Control Mode" << std::endl;
                force_control_mode = true;
            }
        }
    }

    std::cout << "ELMO : EthercatThread2 Shutdown" << std::endl;
}

double elmoJointMove(double init, double angle, double start_time, double traj_time)
{
    double des_pos;

    if (control_time_real_ < start_time)
    {
        des_pos = init;
    }
    else if ((control_time_real_ >= start_time) && (control_time_real_ < (start_time + traj_time)))
    {
        des_pos = init + angle * (control_time_real_ - start_time) / traj_time;
    }
    else
    {
        des_pos = init + angle;
    }

    return des_pos;
}

bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord)
{
    if (!(statusWord & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT)))
        {
            if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (statusWord & (1 << FAULT_BIT))
                {
                    controlWord = 0x80;
                    return false;
                }
                else
                {
                    controlWord = CW_SHUTDOWN;
                    return false;
                }
            }
            else
            {
                controlWord = CW_SWITCHON;
                return true;
            }
        }
        else
        {
            controlWord = CW_ENABLEOP;
            return true;
        }
    }
    else
    {
        controlWord = CW_ENABLEOP;
        return true;
    }
    controlWord = 0;
    return false;
}
void checkJointSafety()
{
    for (int i = 0; i < ec_slavecount; i++)
    {
        if ((joint_lower_limit[i] > q_elmo_[i]) || (joint_upper_limit[i] < q_elmo_[i]))
        {
            //joint limit reached
            //joint_state_elmo_[i] = ESTATE::SAFETY_JOINT_LIMIT;
        }

        if (joint_velocity_limit[i] < abs(q_dot_elmo_[i]))
        {
            //joint_state_elmo_[i] = ESTATE::SAFETY_VELOCITY_LIMIT;
        }
    }
}

void checkJointStatus()
{
}

void sendJointStatus()
{
    shm_msgs_->t_cnt = cycle_count;
    memcpy(&shm_msgs_->pos, q_, sizeof(float) * MODEL_DOF);
    memcpy(&shm_msgs_->posExt, q_ext_, sizeof(float) * MODEL_DOF);
    memcpy(&shm_msgs_->vel, q_dot_, sizeof(float) * MODEL_DOF);
    memcpy(&shm_msgs_->torqueActual, torque_, sizeof(float) * MODEL_DOF);
    memcpy(&shm_msgs_->status, joint_state_elmo_, sizeof(int) * MODEL_DOF);
}

void getJointCommand()
{
    memcpy(command_mode_, &shm_msgs_->commandMode, sizeof(int) * MODEL_DOF);
    memcpy(q_desired_elmo_, &shm_msgs_->positionCommand, sizeof(float) * MODEL_DOF);
    memcpy(torque_desired_elmo_, &shm_msgs_->torqueCommand, sizeof(float) * MODEL_DOF);
}

bool saveCommutationLog()
{
    std::ofstream comfs(commutation_cache_file, std::ios::binary);

    if (!comfs.is_open())
    {
        return false;
    }

    auto const cache_time = (chrono::system_clock::now()).time_since_epoch().count();
    comfs.write(reinterpret_cast<char const *>(&cache_time), sizeof cache_time);
    comfs.close();
    return true;
}

bool loadCommutationLog()
{
    std::ifstream ifs(commutation_cache_file, std::ios::binary);

    if (!ifs.is_open())
    {
        return false;
    }

    std::chrono::system_clock::rep file_time_rep;
    if (!ifs.read(reinterpret_cast<char *>(&file_time_rep), sizeof file_time_rep))
    {
        return false;
    }
    ifs.close();

    std::chrono::system_clock::time_point const cache_valid_time{std::chrono::system_clock::duration{file_time_rep}};

    commutation_save_time_ = cache_valid_time;

    //std::time_t const file_time = std::chrono::system_clock::to_time_t(cache_valid_time);

    std::chrono::duration<double> commutation_before = std::chrono::system_clock::now() - cache_valid_time;
    //std::cout << std::ctime(&file_time);
    std::cout << "Commutation done " << commutation_before.count() << "seconds before .... " << std::endl;

    return true;
}

bool saveZeroPoint()
{
    std::ofstream comfs(zeropoint_cache_file, std::ios::binary);

    if (!comfs.is_open())
    {
        return false;
    }

    auto const cache_time = (chrono::system_clock::now()).time_since_epoch().count();
    comfs.write(reinterpret_cast<char const *>(&cache_time), sizeof cache_time);

    for (int i = 0; i < ec_slavecount; i++)
        comfs.write(reinterpret_cast<char const *>(&q_zero_elmo_[i]), sizeof(double));

    comfs.close();
    return true;
}

bool loadZeroPoint()
{
    std::ifstream ifs(zeropoint_cache_file, std::ios::binary);

    if (!ifs.is_open())
    {
        std::cout << "open failed " << std::endl;
        return false;
    }

    std::chrono::system_clock::rep file_time_rep;

    ifs.read(reinterpret_cast<char *>(&file_time_rep), sizeof file_time_rep);
    double getzp[ELMO_DOF];
    for (int i = 0; i < ec_slavecount; i++)
        ifs.read(reinterpret_cast<char *>(&getzp[i]), sizeof(double));

    ifs.close();

    std::chrono::system_clock::time_point const cache_valid_time{std::chrono::system_clock::duration{file_time_rep}};
    std::time_t const file_time = std::chrono::system_clock::to_time_t(cache_valid_time);

    std::chrono::duration<double> commutation_before = std::chrono::system_clock::now() - cache_valid_time;

    loadCommutationLog();

    if (commutation_save_time_ > cache_valid_time)
    {
        std::cout << "commutation saved time is longer then cached valid time" << std::endl;
        return false;
    }

    //std::cout << "ZP saved at " << std::ctime(&file_time);
    //std::cout << "ZP saved at " << commutation_before.count() << "seconds before .... " << std::endl;
    //for (int i = 0; i < ELMO_DOF; i++)
    //   cout << getzp[i] << endl;

    //check commutation time save point

    //memset(joint_state_elmo_, ESTATE::ZP_SUCCESS, sizeof(int) * ELMO_DOF);

    return true;
}

void emergencyOff() //TorqueZero
{
    for (int i = 0; i < ec_slavecount; i++)
    {
        txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
        txPDO[i]->targetTorque = (int)0;
    }
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch = 0;
    int oldf = 0;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO | ISIG);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    int nread = read(0, &ch, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (nread >= 1)
    {
        return ch;
    }
    else
    {
        return -1;
    }
}

int getElmoState(uint16_t state_bit)
{
    if (!(state_bit & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(state_bit & (1 << SWITCHED_ON_BIT)))
        {
            if (!(state_bit & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (state_bit & (1 << FAULT_BIT))
                {
                    return ELMO_FAULT;
                }
                else
                {
                    return ELMO_NOTFAULT;
                }
            }
            else
            {
                return ELMO_READY_TO_SWITCH_ON;
            }
        }
        else
        {
            return ELMO_SWITCHED_ON;
        }
    }
    else
    {
        return ELMO_OPERATION_ENABLE;
    }
}

void findzeroLeg()
{
    for (int i = 0; i < 6; i++)
    {
        q_zero_elmo_[i + R_HipYaw_Joint] = q_elmo_[i + R_HipYaw_Joint] - q_ext_elmo_[i + R_HipYaw_Joint];
        //pub_to_gui(dc, "jointzp %d %d", i + R_HipYaw_Joint, 1);
        q_zero_elmo_[i + L_HipYaw_Joint] = q_elmo_[i + L_HipYaw_Joint] - q_ext_elmo_[i + L_HipYaw_Joint];
        //pub_to_gui(dc, "jointzp %d %d", i + TOCABI::L_HipYaw_Joint, 1);
        //std::cout << ELMO_NAME[i + R_HipRoll_Joint] << " pz IE P : " << q_elmo_[i + R_HipRoll_Joint] << " pz EE P : " << q_ext_elmo_[i + R_HipRoll_Joint] << std::endl;
        //std::cout << ELMO_NAME[i + L_HipRoll_Joint] << " pz ELMO : " << q_elmo_[i + L_HipRoll_Joint] << " pz ELMO : " << q_ext_elmo_[i + L_HipRoll_Joint] << std::endl;
    }
}
void findZeroPointlow(int slv_number)
{
    //double velocity = 0.05;
    double fztime = 1.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {
        elmofz[slv_number].initTime = control_time_real_;
        elmofz[slv_number].initPos = q_elmo_[slv_number];
        elmofz[slv_number].desPos = q_ext_elmo_[slv_number];
        elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;

        if (q_ext_elmo_[slv_number] > 0)
        {
            elmofz[slv_number].init_direction = -1;
        }
        else
        {
            elmofz[slv_number].init_direction = 1;
        }

        if ((q_ext_elmo_[slv_number] > 3.14) || (q_ext_elmo_[slv_number < -3.14]))
        {

            std::cout << cred << "elmo reboot required. joint " << slv_number << "external encoder error" << q_ext_elmo_[slv_number] << std::endl;
        }
    }

    if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, -elmofz[slv_number].desPos, elmofz[slv_number].initTime, fztime);

        if (control_time_real_ == elmofz[slv_number].initTime)
        {
            //std::cout << "joint " << slv_number << "  init pos : " << elmofz[slv_number].initPos << "   goto " << elmofz[slv_number].initPos + elmofz[slv_number].init_direction * 0.6 << std::endl;
        }
        static int sucnum = 0;

        if (control_time_real_ > (elmofz[slv_number].initTime + fztime + 2.0))
        {
            if (q_ext_elmo_[slv_number] == 0.0)
            {
                elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
                elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
                sucnum++;
                std::cout << slv_number << "success : " << sucnum << std::endl;
            }
            else
            {
                elmofz[slv_number].initTime = control_time_real_;
                elmofz[slv_number].initPos = q_elmo_[slv_number];
                elmofz[slv_number].desPos = q_ext_elmo_[slv_number];
                elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
            }
        }
        // if (control_time_real_ > elmofz[slv_number].initTime + fztime * 4.0)
        // {
        //     elmofz[slv_number].result == ElmoHommingStatus::FAILURE;
        // }
    }
}
void findZeroPoint(int slv_number)
{
    double fztime = 3.0;
    double fztime_manual = 300.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {
        //joint_state_elmo_[slv_number] = ESTATE::ZP_SEARCHING_ZP;
        //pub_to_gui(dc, "jointzp %d %d", slv_number, 0);
        if (hommingElmo[slv_number])
        {
            //std::cout << "motor " << slv_number << " init state : homming on" << std::endl;
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
        else
        {
            //std::cout << "motor " << slv_number << " init state : homming off" << std::endl;
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMING;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {
        //go to + 0.3rad until homming sensor turn off
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, 0.3, elmofz[slv_number].initTime, fztime);

        if ((hommingElmo[slv_number] == 0) && (hommingElmo_before[slv_number] == 0))
        {
            //std::cout << "motor " << slv_number << " seq 1 complete, wait 1 sec" << std::endl;
            hommingElmo_before[slv_number] = hommingElmo[slv_number];
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].posStart = q_elmo_[slv_number];
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }

        if (control_time_real_ > elmofz[slv_number].initTime + fztime)
        {
            std::cout << cred << "warning, " << ELMO_NAME[slv_number] << " : homming sensor not turning off" << creset << std::endl;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGEND)
    {
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].posStart, -0.3, elmofz[slv_number].initTime, fztime);

        //go to -20deg until homming turn on, and turn off
        if ((hommingElmo_before[slv_number] == 1) && (hommingElmo[slv_number] == 0))
        {
            if (abs(elmofz[slv_number].posStart - q_elmo_[slv_number]) > elmofz[slv_number].req_length)
            {
                elmofz[slv_number].posEnd = q_elmo_[slv_number];
                elmofz[slv_number].endFound = 1;
            }
            else
            {
                std::cout << "Joint " << slv_number << " " << ELMO_NAME[slv_number] << " : Not enough distance, required : " << elmofz[slv_number].req_length << ", detected : " << abs(elmofz[slv_number].posStart - q_elmo_[slv_number]) << endl;

                std::cout << "Joint " << slv_number << " " << ELMO_NAME[slv_number] << "if you want to proceed with detected length, proceed with manual mode " << endl;

                //joint_state_elmo_[slv_number] = ESTATE::ZP_NOT_ENOUGH_HOMMING;
                elmofz[slv_number].findZeroSequence = 7;
                elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
                elmofz[slv_number].initTime = control_time_real_;
            }
        }
        else if ((hommingElmo_before[slv_number] == 0) && (hommingElmo[slv_number] == 0))
        {
            if (elmofz[slv_number].endFound == 1)
            {
                elmofz[slv_number].findZeroSequence = FZ_GOTOZEROPOINT;
                //joint_state_elmo_[slv_number] = ESTATE::ZP_GOTO_ZERO;

                elmofz[slv_number].initPos = q_elmo_[slv_number];
                q_zero_elmo_[slv_number] = (elmofz[slv_number].posEnd + elmofz[slv_number].posStart) * 0.5 + q_zero_mod_elmo_[slv_number];
                elmofz[slv_number].initTime = control_time_real_;
            }
        }

        if (control_time_real_ > elmofz[slv_number].initTime + fztime)
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMING)
    { //start from unknown

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].init_direction * 0.3, elmofz[slv_number].initTime, fztime);
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime))
        {
            q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos + 0.3 * elmofz[slv_number].init_direction, -0.6 * elmofz[slv_number].init_direction, elmofz[slv_number].initTime + fztime, fztime * 2.0);
        }

        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }

        if (control_time_real_ > (elmofz[slv_number].initTime + fztime * 3.0))
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_GOTOZEROPOINT)
    {
        ElmoMode[slv_number] = EM_POSITION;

        double go_to_zero_dur = fztime * (abs(q_zero_elmo_[slv_number] - elmofz[slv_number].initPos) / 0.3);
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, q_zero_elmo_[slv_number] - elmofz[slv_number].initPos, elmofz[slv_number].initTime, go_to_zero_dur);

        //go to zero position
        if (control_time_real_ > (elmofz[slv_number].initTime + go_to_zero_dur))
        {
            //std::cout << "go to zero complete !" << std::endl;
            //printf("Motor %d %s : Zero Point Found : %8.6f, homming length : %8.6f ! ", slv_number, ELMO_NAME[slv_number].c_str(), q_zero_elmo_[slv_number], abs(elmofz[slv_number].posStart - elmofz[slv_number].posEnd));
            //fflush(stdout);

            // printf("\33[2K\r");
            // for(int i=0;i<16;i++)
            // {

            // }

            //pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
            //joint_state_elmo_[slv_number] = ESTATE::ZP_SUCCESS;
            //std::cout << slv_number << "Start : " << elmofz[slv_number].posStart << "End:" << elmofz[slv_number].posEnd << std::endl;
            //q_desired_elmo_[slv_number] = positionZeroElmo(slv_number);
            elmofz[slv_number].findZeroSequence = 8; // torque to zero -> 8 position hold -> 5
            ElmoMode[slv_number] = EM_TORQUE;
            torque_desired_elmo_[slv_number] = 0.0;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 5)
    {
        //find zero complete, hold zero position.
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = q_zero_elmo_[slv_number];
    }
    else if (elmofz[slv_number].findZeroSequence == 6)
    {
        //find zero point failed
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].firstPos - elmofz[slv_number].initPos, elmofz[slv_number].initTime, fztime);
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime))
        {
            elmofz[slv_number].findZeroSequence = 7;
            printf("Motor %d %s : Zero point detection Failed. Manual Detection Required. \n", slv_number, ELMO_NAME[slv_number].c_str());
            //pub_to_gui(dc, "jointzp %d %d", slv_number, 2);
            elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
            //joint_state_elmo_[slv_number] = ESTATE::ZP_MANUAL_REQUIRED;
            elmofz[slv_number].initTime = control_time_real_;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 7)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torque_desired_elmo_[slv_number] = 0.0;
        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime_manual))
        {
            printf("Motor %d %s :  Manual Detection Failed. \n", slv_number, ELMO_NAME[slv_number].c_str());
            //pub_to_gui(dc, "jointzp %d %d", slv_number, 3);
            elmofz[slv_number].findZeroSequence = 8;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 8)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torque_desired_elmo_[slv_number] = 0.0;
    }
}