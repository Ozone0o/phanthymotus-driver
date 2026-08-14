#include "waypoint.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#ifdef PSDK_ENABLED
#include "dji_waypoint_v2.h"
#endif

/*
 * M300 uses DJI Waypoint V2, not the Mavic 3E Waypoint V3/KMZ API.
 *
 * The Python card uploads a structured JSON mission with fields that map
 * directly to a Waypoint V2 mission builder:
 *   repeat_times, finished_action, max_flight_speed, auto_flight_speed,
 *   action_when_rc_lost, goto_first_waypoint_mode, waypoints[].
 *
 * This bridge keeps a small state machine in builds where the target PSDK V2
 * headers are not present.  The hardware-specific implementation should build
 * T_DjiWayPointV2MissionSettings from s_last_mission_json and call
 * DjiWaypointV2_UploadMission(), then DjiWaypointV2_Start/Pause/Resume/Stop().
 */

static char s_last_mission_json[16384];
static int s_uploaded = 0;
static const char *s_state = "idle";

#ifdef PSDK_ENABLED
#define M300_WAYPOINT_MAX 256
static T_DjiWaypointV2 s_waypoints[M300_WAYPOINT_MAX];
static T_DjiWaypointV2MissionStatePush s_last_fc_state;
static T_DjiWaypointV2MissionEventPush s_last_fc_event;
static volatile int s_has_fc_state = 0;
static volatile int s_has_fc_event = 0;
static uint8_t s_last_logged_fc_state = 0xFF;

static const char *_fc_state_name(uint8_t state) {
    switch (state) {
        case DJI_WAYPOINT_V2_MISSION_STATE_GROUND_STATION_NOT_START:
            return "ground_station_not_start";
        case DJI_WAYPOINT_V2_MISSION_STATE_MISSION_PREPARED:
            return "mission_prepared";
        case DJI_WAYPOINT_V2_MISSION_STATE_ENTER_MISSION:
            return "enter_mission";
        case DJI_WAYPOINT_V2_MISSION_STATE_EXECUTING:
            return "executing";
        case DJI_WAYPOINT_V2_MISSION_STATE_PAUSED:
            return "paused";
        case DJI_WAYPOINT_V2_MISSION_STATE_ENTER_MISSION_AFTER_ENDING_PAUSE:
            return "enter_mission_after_pause";
        case DJI_WAYPOINT_V2_MISSION_STATE_EXIT_MISSION:
            return "exit_mission";
        default:
            return "unknown";
    }
}

static T_DjiReturnCode _waypoint_state_cb(T_DjiWaypointV2MissionStatePush stateData) {
    s_last_fc_state = stateData;
    s_has_fc_state = 1;
    if (s_last_logged_fc_state != stateData.state) {
        s_last_logged_fc_state = stateData.state;
        printf("[waypoint] state_cb: state=%u(%s) cur=%u velocity=%.2f m/s\n",
               stateData.state,
               _fc_state_name(stateData.state),
               stateData.curWaypointIndex,
               stateData.velocity / 100.0f);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _waypoint_event_cb(T_DjiWaypointV2MissionEventPush eventData) {
    s_last_fc_event = eventData;
    s_has_fc_event = 1;
    printf("[waypoint] event_cb: event=0x%02X timestamp=%u\n",
           eventData.event,
           eventData.FCTimestamp);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static int _wait_for_fc_state(uint8_t expected_state, int timeout_ms) {
    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        if (s_has_fc_state && s_last_fc_state.state == expected_state)
            return 1;
        usleep(100000);
        elapsed_ms += 100;
    }
    return 0;
}

static int _fc_state_is_uploaded(void) {
    if (!s_has_fc_state)
        return 0;
    return s_last_fc_state.state == DJI_WAYPOINT_V2_MISSION_STATE_MISSION_PREPARED ||
           s_last_fc_state.state == DJI_WAYPOINT_V2_MISSION_STATE_ENTER_MISSION ||
           s_last_fc_state.state == DJI_WAYPOINT_V2_MISSION_STATE_EXECUTING ||
           s_last_fc_state.state == DJI_WAYPOINT_V2_MISSION_STATE_PAUSED ||
           s_last_fc_state.state == DJI_WAYPOINT_V2_MISSION_STATE_ENTER_MISSION_AFTER_ENDING_PAUSE;
}

static int _json_get_float(const char *json, const char *key, float fallback, float *out) {
    char pattern[64];
    const char *p;
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    p = strstr(json, pattern);
    if (!p) {
        *out = fallback;
        return 0;
    }
    p = strchr(p, ':');
    if (!p) return -1;
    *out = (float)atof(p + 1);
    return 0;
}

static int _json_get_int(const char *json, const char *key, int fallback, int *out) {
    char pattern[64];
    const char *p;
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    p = strstr(json, pattern);
    if (!p) {
        *out = fallback;
        return 0;
    }
    p = strchr(p, ':');
    if (!p) return -1;
    *out = atoi(p + 1);
    return 0;
}

static int _json_has_string_value(const char *json, const char *key, const char *value) {
    char pattern[128];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"%s\"", key, value);
    return strstr(json, pattern) != NULL;
}

static int _parse_waypoints(const char *json, float auto_speed, float max_speed) {
    const char *p = json;
    int count = 0;

    while ((p = strstr(p, "\"latitude\"")) != NULL && count < M300_WAYPOINT_MAX) {
        double lat = 0.0;
        double lon = 0.0;
        float alt = 0.0f;
        const char *lat_val = strchr(p, ':');
        const char *lon_key = strstr(p, "\"longitude\"");
        const char *alt_key = strstr(p, "\"relative_altitude\"");
        if (!lat_val || !lon_key || !alt_key) break;
        lat = atof(lat_val + 1);
        lon = atof(strchr(lon_key, ':') + 1);
        alt = (float)atof(strchr(alt_key, ':') + 1);

        memset(&s_waypoints[count], 0, sizeof(s_waypoints[count]));
        s_waypoints[count].latitude = lat;
        s_waypoints[count].longitude = lon;
        s_waypoints[count].relativeHeight = alt;
        s_waypoints[count].waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
        if (count == 0)
            s_waypoints[count].waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_FIRST_POINT_ALONG_STRAIGHT_LINE;
        s_waypoints[count].headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
        s_waypoints[count].config.useLocalCruiseVel = 1;
        s_waypoints[count].config.useLocalMaxVel = 1;
        s_waypoints[count].dampingDistance = 0;
        s_waypoints[count].heading = 0.0f;
        s_waypoints[count].turnMode = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;
        s_waypoints[count].maxFlightSpeed = max_speed;
        s_waypoints[count].autoFlightSpeed = auto_speed;
        count++;
        p = alt_key + 1;
    }
    if (count > 1)
        s_waypoints[count - 1].waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_STRAIGHT_OUT;
    return count;
}
#endif

int waypoint_init(void) {
    s_uploaded = 0;
    s_state = "idle";
    s_last_mission_json[0] = '\0';
#ifdef PSDK_ENABLED
    s_has_fc_state = 0;
    s_has_fc_event = 0;
    s_last_logged_fc_state = 0xFF;
    T_DjiReturnCode rc = DjiWaypointV2_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] DjiWaypointV2_Init failed: 0x%08llX\n", (unsigned long long)rc);
        return (int)rc;
    }
    rc = DjiWaypointV2_RegisterMissionStateCallback(_waypoint_state_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] RegisterMissionStateCallback failed: 0x%08llX\n", (unsigned long long)rc);
    }
    rc = DjiWaypointV2_RegisterMissionEventCallback(_waypoint_event_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] RegisterMissionEventCallback failed: 0x%08llX\n", (unsigned long long)rc);
    }
#endif
    printf("[waypoint] M300 Waypoint V2 bridge initialized\n");
    return 0;
}

int waypoint_upload(const char *mission_json) {
    if (mission_json == NULL || strstr(mission_json, "\"waypoints\"") == NULL) {
        printf("[waypoint] upload rejected: mission JSON missing waypoints\n");
        return -1;
    }
    snprintf(s_last_mission_json, sizeof(s_last_mission_json), "%s", mission_json);
#ifdef PSDK_ENABLED
    float max_speed = 10.0f;
    float auto_speed = 5.0f;
    int repeat_times = 0;
    int waypoint_count;
    T_DjiWayPointV2MissionSettings settings;

    _json_get_float(mission_json, "max_flight_speed", 10.0f, &max_speed);
    _json_get_float(mission_json, "auto_flight_speed", 5.0f, &auto_speed);
    _json_get_int(mission_json, "repeat_times", 0, &repeat_times);
    if (max_speed < 2.0f) max_speed = 2.0f;
    if (max_speed > 15.0f) max_speed = 15.0f;
    if (auto_speed < -15.0f) auto_speed = -15.0f;
    if (auto_speed > 15.0f) auto_speed = 15.0f;

    waypoint_count = _parse_waypoints(mission_json, auto_speed, max_speed);
    if (waypoint_count < 2) {
        printf("[waypoint] upload rejected: M300 Waypoint V2 needs at least 2 waypoints\n");
        return -1;
    }

    memset(&settings, 0, sizeof(settings));
    settings.missionID = (uint32_t)(time(NULL) & 0x7fffffff);
    settings.repeatTimes = (uint8_t)(repeat_times < 0 ? 0 : repeat_times);
    settings.finishedAction = DJI_WAYPOINT_V2_FINISHED_GO_HOME;
    if (_json_has_string_value(mission_json, "finished_action", "no_action"))
        settings.finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
    else if (_json_has_string_value(mission_json, "finished_action", "auto_land"))
        settings.finishedAction = DJI_WAYPOINT_V2_FINISHED_AUTO_LANDING;
    settings.maxFlightSpeed = max_speed;
    settings.autoFlightSpeed = auto_speed;
    settings.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
    settings.gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT;
    settings.mission = s_waypoints;
    settings.missTotalLen = (uint16_t)waypoint_count;
    settings.actionList.actions = NULL;
    settings.actionList.actionNum = 0;

    T_DjiReturnCode rc = DjiWaypointV2_UploadMission(&settings);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] DjiWaypointV2_UploadMission failed: 0x%08llX\n", (unsigned long long)rc);
        if (!_wait_for_fc_state(DJI_WAYPOINT_V2_MISSION_STATE_MISSION_PREPARED, 3000) &&
            !_fc_state_is_uploaded()) {
            return (int)rc;
        }
        printf("[waypoint] UploadMission return ignored because FC state is %u(%s)\n",
               s_last_fc_state.state, _fc_state_name(s_last_fc_state.state));
    }
#endif
    s_uploaded = 1;
    s_state = "uploaded";
    printf("[waypoint] M300 Waypoint V2 mission accepted by bridge\n");
    return 0;
}

int waypoint_start(void) {
    if (!s_uploaded) {
        printf("[waypoint] start rejected: upload a mission first\n");
        return -1;
    }
#ifdef PSDK_ENABLED
    T_DjiReturnCode rc = DjiWaypointV2_Start();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] DjiWaypointV2_Start failed: 0x%08llX\n", (unsigned long long)rc);
        if (!_wait_for_fc_state(DJI_WAYPOINT_V2_MISSION_STATE_EXECUTING, 3000)) {
            return (int)rc;
        }
        printf("[waypoint] Start return ignored because FC state is executing\n");
    }
#endif
    s_state = "executing";
    return 0;
}

int waypoint_pause(void) {
    if (!s_uploaded) return -1;
#ifdef PSDK_ENABLED
    T_DjiReturnCode rc = DjiWaypointV2_Pause();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return (int)rc;
#endif
    s_state = "paused";
    return 0;
}

int waypoint_resume(void) {
    if (!s_uploaded) return -1;
#ifdef PSDK_ENABLED
    T_DjiReturnCode rc = DjiWaypointV2_Resume();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return (int)rc;
#endif
    s_state = "executing";
    return 0;
}

int waypoint_stop(void) {
#ifdef PSDK_ENABLED
    T_DjiReturnCode rc = DjiWaypointV2_Stop();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return (int)rc;
#endif
    s_state = "idle";
    return 0;
}

int waypoint_get_status(char *buf, size_t buflen) {
#ifdef PSDK_ENABLED
    snprintf(buf, buflen,
             "{\"state\":\"%s\",\"uploaded\":%s,\"version\":\"m300-waypoint-v2\","
             "\"fc_state_valid\":%s,\"fc_state\":%u,\"fc_state_name\":\"%s\","
             "\"fc_cur_waypoint\":%u,\"fc_velocity\":%.2f,"
             "\"fc_event_valid\":%s,\"fc_event\":%u}",
             s_state, s_uploaded ? "true" : "false",
             s_has_fc_state ? "true" : "false",
             s_has_fc_state ? s_last_fc_state.state : 0,
             s_has_fc_state ? _fc_state_name(s_last_fc_state.state) : "unknown",
             s_has_fc_state ? s_last_fc_state.curWaypointIndex : 0,
             s_has_fc_state ? s_last_fc_state.velocity / 100.0f : 0.0f,
             s_has_fc_event ? "true" : "false",
             s_has_fc_event ? s_last_fc_event.event : 0);
#else
    snprintf(buf, buflen, "{\"state\":\"%s\",\"uploaded\":%s,\"version\":\"m300-waypoint-v2\"}",
             s_state, s_uploaded ? "true" : "false");
#endif
    return 0;
}

void waypoint_cleanup(void) {
#ifdef PSDK_ENABLED
    DjiWaypointV2_Deinit();
#endif
    s_uploaded = 0;
    s_state = "idle";
}
