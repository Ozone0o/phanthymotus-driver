#include "waypoint.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

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
    T_DjiReturnCode rc = DjiWaypointV2_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] DjiWaypointV2_Init failed: 0x%08llX\n", (unsigned long long)rc);
        return (int)rc;
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
        return (int)rc;
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
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return (int)rc;
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
    snprintf(buf, buflen, "{\"state\":\"%s\",\"uploaded\":%s,\"version\":\"m300-waypoint-v2\"}",
             s_state, s_uploaded ? "true" : "false");
    return 0;
}

void waypoint_cleanup(void) {
#ifdef PSDK_ENABLED
    DjiWaypointV2_Deinit();
#endif
    s_uploaded = 0;
    s_state = "idle";
}
