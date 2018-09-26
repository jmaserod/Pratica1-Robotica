#include "my_qtimer.h"



void my_qtimer::run()
{
     sttop = false;
     while(true)
     {
        if(!sttop)
        {
            emit timeout();
            msleep(1000);
             
        }
    }
    
}

void my_qtimer::stop_time(){
    sttop = !sttop;
}

my_qtimer::my_qtimer(){}
my_qtimer::~my_qtimer(){}

