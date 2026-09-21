#include "remote_input.h"

#include "app_config.h"

void Remote_ApplyMapping(Remote_t *remote)
{
    /*
     * 右拨杆决定使能级别，右上时才由左拨杆选模式。
     * 具体档位对应哪个模式全部由 remote_input.h 的 REMOTE_MAP_* 决定，
     * 改拨杆分配不需要动这里。右下急停由 rightSwitch 字段单独上报。
     */
    if (remote->rightSwitch == REMOTE_SWITCH_MID)
    {
        remote->modeRequest = REMOTE_MAP_RIGHT_MID;
    }
    else if (remote->rightSwitch == REMOTE_SWITCH_UP)
    {
        switch (remote->leftSwitch)
        {
        case REMOTE_SWITCH_UP:
            remote->modeRequest = REMOTE_MAP_LEFT_UP;
            break;

        case REMOTE_SWITCH_MID:
            remote->modeRequest = REMOTE_MAP_LEFT_MID;
            break;

        case REMOTE_SWITCH_DOWN:
            remote->modeRequest = REMOTE_MAP_LEFT_DOWN;
            break;

        case REMOTE_SWITCH_UNKNOWN:
        default:
            remote->modeRequest = REMOTE_MODE_NONE;
            break;
        }
    }
    else
    {
        remote->modeRequest = REMOTE_MODE_NONE;
    }

    if (remote->dialValid == 0U)
    {
        remote->legRequest = REMOTE_LEG_KEEP;
    }
    else if (remote->dial > APP_RC_DIAL_THRESHOLD)
    {
        remote->legRequest = REMOTE_LEG_SHORT;
    }
    else if (remote->dial < -APP_RC_DIAL_THRESHOLD)
    {
        remote->legRequest = REMOTE_LEG_LONG;
    }
    else
    {
        remote->legRequest = REMOTE_LEG_MIDDLE;
    }
}
