#include "fcu_states.h"


bool FCU_STATES::is_in_contact_sequence(FCU_STATES::STATES state)
{
    return (state == FCU_STATES::APPROACH)
            || (state == FCU_STATES::BOARD)
            || (state == FCU_STATES::DOCK);
}