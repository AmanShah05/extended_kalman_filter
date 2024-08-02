#ifndef FCU_STATES_H_
#define FCU_STATES_H_

class FCU_STATES {
    private:
        // Version linked to: <repo>/desktop/controller/state.py
        // Increment for breaking change (re-ordering)
        static constexpr int VERSION_MAJOR = 4;
        // Increment for non-breaking change (add new state to end)
        static constexpr int VERSION_MINOR = 0;

    public:
        enum STATES : uint8_t {
            BOOT,
            IDLE,
            STANDBY,
            PREFLIGHT,
            TAKEOFF,
            FLIGHT,
            LANDING,
            POSTFLIGHT,
            KILL,
            DEMO,
            GROUND_TEST,
            RTH,
            APPROACH,
            BOARD,
            DOCK,
            RETREAT,
            NUM_STATES,
            INVALID
        };

        static bool is_in_contact_sequence(STATES state);
};

#define FCU_STATE_STRINGS \
    "BOOT", \
    "PARK", \
    "READY", \
    "PRE-FLIGHT", \
    "TAKE-OFF", \
    "FLIGHT", \
    "LAND", \
    "COOLDOWN", \
    "KILL", \
    "DEMO", \
    "TEST", \
    "RTH", \
    "APPROACH", \
    "BOARD", \
    "DOCK", \
    "RETREAT"

#endif