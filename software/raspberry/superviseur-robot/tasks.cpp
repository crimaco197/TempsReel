
/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>;.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATTERY 20
#define PRIORITY_TWATCHDOG 30
#define PRIORITY_TARENA 20


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 *
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 *
 * 3- Data flow is probably not optimal
 *
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 *
 * 5- Same behavior existe for ComMonitor::Write !
 *
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 *
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex,
 * semaphore, etc.)
 */




//bool cameraActivated = false ;

//Arena arena ;

void Tasks::Init() {
    int status;
    int err;


    /**************************************************************************************/
    /*      Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_Camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_WatchDog, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_Position, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_FindArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }


    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /*      Semaphors creation                                                  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_WatchDog_rst, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_OpenCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_CloseCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_FindArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_SendImage, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_FoundArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_FindRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_WatchD, "th_WatchD", 0, PRIORITY_TWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_OpenCamera, "th_OpenCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_CloseCamera, "th_CloseCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_FindArena, "th_FindArena", 0, PRIORITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_SendImage, "th_SendImage", 0, PRIORITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_FindRobot, "th_FindRobot", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }


    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_WatchD, (void(*)(void*)) & Tasks::Robot_WatchD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_OpenCamera, (void(*)(void*)) & Tasks::OpenCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_CloseCamera, (void(*)(void*)) & Tasks::CloseCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_FindArena, (void(*)(void*)) & Tasks::FindArena, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_SendImage, (void(*)(void*)) & Tasks::SendImage, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_FindRobot, (void(*)(void*)) & Tasks::FindRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
    //robot.GetBattery();
    //Tasks::BatteryTask(void *arg)RT_SEM
}

void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            cout << msgRcv->ToString() << endl << flush; /* TASK5 :A tester : tu lances les deux monit et supervis , apres fermes monit , un message de connection lost dpoit safficher sur terminal de superviseur*/
            cout << msgRcv;
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) { // if(12)

            rt_sem_v(&sem_openComRobot);

        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            cout << "starting robot without watchdog" << endl << flush;
            rt_mutex_acquire(&mutex_WatchDog, TM_INFINITE);
            WatchDog = 0;
            rt_mutex_release(&mutex_WatchDog);
            

            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            cout << "starting robot with watchdog" << endl << flush;
            rt_mutex_acquire(&mutex_WatchDog, TM_INFINITE);
            WatchDog = 1;
            cout << "WATCHDOG ACTIVATED = " << WatchDog << endl << flush;
            rt_mutex_release(&mutex_WatchDog);

            rt_sem_v(&sem_startRobot);


        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
        //    rt_mutex_acquire(&mutex_Camera, TM_INFINITE);  // a tester
            rt_sem_v(&sem_OpenCamera);
        //    rt_mutex_release(&mutex_Camera);            // a tester
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
        //    rt_mutex_acquire(&mutex_Camera, TM_INFINITE);  // a tester
            rt_sem_v(&sem_CloseCamera);
        //    rt_mutex_release(&mutex_Camera);            // a tester
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            arena_req = true;
            rt_sem_v(&sem_FindArena);

        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) { // MESSAGE ARENA FOUND
            a = true;
            confirm_arena = true;

            rt_sem_v(&sem_OpenCamera);
            //   rt_sem_v(&sem_FoundArena); 

        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) { // MESSAGE ARENA NOT FOUND
            //  rt_sem_v(&sem_CloseCamera);
            a = false;
            confirm_arena = true;

            //   rt_sem_v(&sem_FoundArena);
            rt_sem_v(&sem_OpenCamera);

        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            // findRobot = true ;
            cout << "START POSITION ROBOT ";

            rt_mutex_acquire(&mutex_Position, TM_INFINITE);
            rt_sem_v(&sem_FindRobot);
            rt_mutex_release(&mutex_Position);
            //789 

        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            // findRobot = false ;
            // rt_sem_v(&sem_FindRobot);
            cout << "STOP POSITION ROBOT ";

        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        cout << "waiting in StartRobotTask" << endl << flush;
        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);


        if (WatchDog == 0) {
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);

        } else {
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_mutex_release(&mutex_robot);
        }
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
        rt_sem_v(&sem_WatchDog_rst);
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);

            cout << " move: " << cpMove;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID) cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/*void Tasks::MoniAvecRobot(Message * mensaje) {
    static int cmpt =0;
    if( (* mensaje == MESSAGE_ANSWER_NACK) || (* mensaje == MESSAGE_ANSWER_ROBOT_TIMEOUT ) || (* mensaje == MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND) || (* mensaje == MESSAGE_ANSWER_ROBOT_ERROR) || (* mensaje == MESSAGE_ANSWER_COM_ERROR)) cmpt=cmpt;
    else cmpt++;
    if (cmpt >= 3){ rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted=0; rt_mutex_release(&mutex_robotStarted); } }*/

// Tarea 8 + 9

void Tasks::MoniAvecRobot(Message* message) {
    static int cmpt = 0;

    if (*message == MESSAGE_ANSWER_ROBOT_TIMEOUT) {
        cmpt++;
    } else {
        cmpt = 0;
    }


    if (cmpt >= 3) {
        cout << "\nConnection Lost ";
        WriteInQueue(&q_messageToMon, message);

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;

        rt_mutex_release(&mutex_robotStarted);
    }
}

//Task 10 +11

void Tasks::Robot_WatchD(void* args) {
    int watch, rs;
    Message * msg_moni;
    cout << "Starting WATCHDOG " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_WatchDog_rst, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);


        rt_mutex_acquire(&mutex_WatchDog, TM_INFINITE);
        watch = WatchDog;
        rt_mutex_release(&mutex_WatchDog);

        if (rs == 1 and watch == 1) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg_moni = robot.Write(robot.ReloadWD());
            // MoniAvecRobot(msg_moni);
            rt_mutex_release(&mutex_robot);

            cout << "WatchDog activated - " << msg_moni->ToString() << endl << flush;
        }
    }
}


// TASK 13

void Tasks::BatteryTask(void *arg) {
    Message* Batt_lvl;

    cout << "Starting BATTERY LEVEL " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 800000000);

    while (1) {
        rt_task_wait_period(NULL);
        //cout << "Periodic battery update" << endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        int rs = robotStarted;

        rt_mutex_release(&mutex_robotStarted);

        if (rs == 1) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);

            Batt_lvl = robot.Write(robot.GetBattery());
            MoniAvecRobot(Batt_lvl);
            rt_mutex_release(&mutex_robot);

            cout << "\nBattery level:" << Batt_lvl->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, Batt_lvl);
        }
    }
}

void Tasks::OpenCamera(void *arg) {
    cout << "CAMERA ACTIVATED" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    Message * msgSend;

    while (1) {
        rt_task_wait_period(NULL);
        rt_sem_p(&sem_OpenCamera, TM_INFINITE);             // WAITING FOR SIGNAL SENT BY rt_sem_v(&sem_OpenCamera)
        rt_mutex_acquire(&mutex_Camera, TM_INFINITE);       // LOCK THE THREAD, SO NO ONE CAN ACCESS HERE BEFORE IT RELEASES
        if (cam.Open()) {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            cout << "camera opened " << endl << flush;

        } else {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            cout << "camera not opened " << endl << flush;

        }

        rt_mutex_release(&mutex_Camera);                    // THREAD RELEASED, IT'S UP TO BE USED AGAIN.
        WriteInQueue(&q_messageToMon, msgSend);
        send_image = true;
        rt_sem_v(&sem_SendImage);                           // SENDING SIGNAL TO SEND IMAGES (PERIODIC)
    }
}

void Tasks::CloseCamera(void *arg) {
    cout << "CAMERA DESACTIVATED " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);


    while (1) {
        rt_task_wait_period(NULL);
        rt_sem_p(&sem_CloseCamera, TM_INFINITE);            // WAITING FOR SIGNAL SENT BY rt_sem_v(&sem_CloseCamera)
        rt_mutex_acquire(&mutex_Camera, TM_INFINITE);       // LOCK THE THREAD, SO NO ONE CAN ACCESS HERE BEFORE IT RELEASES
        cam.Close();                                        // CLOSES CAMERA

        Message * msgSend;
        if (cam.IsOpen()) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            cout << "camera not closed " << endl << flush;

        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            cout << "camera closed " << endl << flush;
        }
        rt_mutex_release(&mutex_Camera);                    // THREAD RELEASED, IT'S UP TO BE USED AGAIN.
        WriteInQueue(&q_messageToMon, msgSend);
    }
}

/**
 * @brief Camera send images
 */
void Tasks::SendImage(void *arg) {
    cout << "SENDING IMAGE " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/

    rt_sem_p(&sem_SendImage, TM_INFINITE);                  // WAITING FOR SIGNAL SENT BY rt_sem_v(&sem_SendImage)
    rt_task_set_periodic(NULL, TM_NOW, 100000000);          // SEND IMAGES EVERY 100ns
    MessageImg * msgImg = new MessageImg();
    while (1) {

        rt_task_wait_period(NULL);                          // STOPS THE WHILE, SO IT IS NOT EXECUTED INFINITE
        rt_mutex_acquire(&mutex_Camera, TM_INFINITE);       // BLOCKS THE THREAD
        if (cam.IsOpen()) {
            
            img = cam.Grab();                               // GET / CAPTURE AN IMAGE
            
            if (arena_saved.IsEmpty()) {
                img.DrawArena(arena_saved);                 // DRAWS THE RED SQUARE

            }


            msgImg->SetID(MESSAGE_CAM_IMAGE);               // SEND MESSAGE WITH THE IMAGE AND THE ARENA 
            msgImg->SetImage(&img);
            
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);  
            monitor.Write(msgImg);                          // SEND THE MESSAGE TO THE MONITOR
            rt_mutex_release(&mutex_monitor);
        }
        rt_mutex_release(&mutex_Camera);


    }
}

/**
 * @brief FInd arena 
 */

void Tasks::FindArena(void *arg) {
    cout << "LOOKING FOR ARENA" << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    Message * msgSend;
    //123

    rt_sem_p(&sem_FindArena, TM_INFINITE);                  // WAITING FOR SIGNAL SENT BY rt_sem_v(&sem_FindArena)
    MessageImg *msgImg = new MessageImg();
    while (1) {
        rt_task_wait_period(NULL);                          // STOPS THE WHILE, SO IT IS NOT EXECUTED INFINITE
        rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
        img = cam.Grab();
        arena = img.SearchArena();                          // IDENTIFY THE ARENA ON THE IMAGE
        img.DrawArena(arena);                               // DRAWS ARENA IF IT EXISTS (BEFORE AND AFTER IF TO GUARANTEE THE DRAW)
        if ((!arena.IsEmpty() && a)) {
            img.DrawArena(arena);
            msgSend = new Message(MESSAGE_ANSWER_ACK);      // ARENA FOUND - SEND MESSAGE ACK
            cout << "Arena found" << endl;


            msgImg->SetID(MESSAGE_CAM_IMAGE);
            msgImg->SetImage(&img);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgImg);
            WriteInQueue(&q_messageToMon, msgImg);
            rt_mutex_release(&mutex_monitor);
            //    rt_sem_p(&sem_CloseCamera,TM_INFINITE);
            //     rt_sem_p(&sem_FoundArena, TM_INFINITE);

            if (confirm_arena) {
                arena_saved = arena;                        // WE SAVE THE ARENA FOUND TO USE IT LATER
            }
        } else {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            cout << "Arena not found" << endl;
            WriteInQueue(&q_messageToMon, msgSend);
        }
        rt_mutex_release(&mutex_Camera);
    }
}

void Tasks::FindRobot(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    Message *msgSend;

    rt_sem_p(&sem_FindRobot, TM_INFINITE);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
        img = cam.Grab();
        std::list<Position> robots = img.SearchRobot(arena);        // LOOK FOR ROBOTS ON THE ARENA
        // Agrega una impresión de debug para verificar si la lista de robots está vacía
        std::cout << "Verificando si la lista de robots está vacía..." << std::endl;
        std::cout << "Tamaño de la lista de robots: " << robots.size() << std::endl;
        // Verifica si arena no es nulo antes de usarlo
        if (!arena.IsEmpty()) {
            std::cout << "Arena is not empty" << std::endl;
        } else {
            std::cout << "Arena is empty" << std::endl;
        }


        if (!robots.empty()) {
            // IF ROBOT IS FOUND, MAKE A SQUEARE AND SEND A MESSAGE WITH POSITIONS TO THE MONITOR
            cout << "Robot found" << endl;
            // HACER CASO SOLO PARA UN ROBOT, NO TODOS.
            //robotPos = img.DrawRobot();
            img.DrawAllRobots(robots);
            for (Position robot : robots) {
                MessagePosition *msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, robot);    // SEND MESSAGE WITH FOUND POSITION
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msgPosition);
                WriteInQueue(&q_messageToMon, msgPosition);
                rt_mutex_release(&mutex_monitor);
            }
        } else {
            // IF ROBOT IS NOT FOUND, THEN POSITION IS (-1, -1)
            cout << "Robot NOT found" << endl;
            Position noRobotPos;
            noRobotPos.center = cv::Point2f(-1.0, -1.0);
            noRobotPos.direction = cv::Point2f(-1.0, -1.0);
            noRobotPos.robotId = -1;
            noRobotPos.angle = -1.0;

            MessagePosition *msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, noRobotPos);  // SEND MESSAGE WITH -1,-1
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgPosition);
            WriteInQueue(&q_messageToMon, msgPosition);
            rt_mutex_release(&mutex_monitor);
            
        }
        rt_mutex_release(&mutex_Camera);
    }
}

void Tasks::SendPositionMessage(Position pos) {
    // Crear un nuevo mensaje de posición
    MessagePosition *msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, pos);

    // Escribir el mensaje en la cola para el monitor
    WriteInQueue(&q_messageToMon, msgPosition);
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

