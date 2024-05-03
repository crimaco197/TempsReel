
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

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
   
    /**
     * @brief Suspends main thread
     */
    void Join();
   
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    Camera camera;    // USAR ESTA VARIABLE
    int robotStarted = 0;
    int move = MESSAGE_ROBOT_STOP;
    int WatchDog = 0 ;
    Camera cam;       // BORRAR
    Arena arena;
    Arena arena_saved;
    bool arena_req = false;
    bool confirm_arena = false;
    bool send_image;
    Position robotPosition;
    bool findRobot;
    Img img =cam.Grab() ;
    bool a = true ;
    Position robotPos;
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;
    RT_TASK th_battery;
    RT_TASK th_WatchD;
    RT_TASK th_CloseCamera ;
    RT_TASK th_FindArena;
    RT_TASK th_OpenCamera;
    RT_TASK th_SendImage;
    RT_TASK th_FindRobot;
   
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;
    RT_MUTEX mutex_WatchDog;
    RT_MUTEX mutex_comRobot;
    RT_MUTEX mutex_Camera;
    RT_MUTEX mutex_FindArena;
    RT_MUTEX mutex_Position;
 

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;
    RT_SEM sem_WatchDog_rst;
    
    RT_SEM sem_OpenCamera;
    RT_SEM sem_CloseCamera;
    
    RT_SEM sem_FindArena;
    RT_SEM sem_FoundArena;
    
    RT_SEM sem_SendImage;
    
    RT_SEM sem_FindRobot;
    RT_SEM sem_FoundRobot;
   
   

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
   
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
   
    void BatteryTask(void *arg);
   
    void Robot_WatchD(void* args);
   
   void SendImage(void *arg);
    void CloseCamera(void * arg);
    void OpenCamera(void * arg);
    void FindArena(void * arg);
   
    void FindRobot(void * arg);
   
    void MoniAvecRobot(Message* message);
     void SendPositionMessage(Position pos);
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
       
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
   
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
   
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
   
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
   
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);
   
   

};

#endif // __TASKS_H__





