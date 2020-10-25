#include "core.h"

System sys = System();

void setup(){
    while ( sys.init() != SYSTEM_READY ){
        sys.buzzer(true);
        delay(500);
        sys.buzzer(false);
        delay(500);
    }
}

void loop(){

}