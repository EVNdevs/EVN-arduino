#include "EVNRGBLED.h"

mutex_t EVNRGBLED::_mutex;
alarm_id_t EVNRGBLED::_alarm_id;
int EVNRGBLED::dma_channels[] = { };
bool EVNRGBLED::ports_started[] = { };
