#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>
#include <vector>
#include <cstring>
#include <openssl/sha.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "interface/IOSDK.h"

bool running = true;  


void ShutDown(int sig) 
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()  // 实时调度设置
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv) {
    
    setProcessScheduler();
    std::cout << std::fixed << std::setprecision(3);
    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.02;
    ctrlComp->running = &running;

    ControlFrame ctrlFrame(ctrlComp);
    signal(SIGINT, ShutDown);

    while (running) {
        if (ctrlComp->exitFlag) break;
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
