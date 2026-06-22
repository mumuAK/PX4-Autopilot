#include "test_driver.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace A320;
using namespace std;

void printState(const AircraftState& state, const ControlCommand& cmd) {
    cout << "Time: " << fixed << setprecision(2) << 0.0 << "s | "
         << "ALT: " << state.alt_msl / 0.3048f << "ft | "
         << "IAS: " << state.cas * MS_TO_KNOTS << "kt | "
         << "Pitch: " << state.pitch * RAD_TO_DEG << "deg | "
         << "Roll: " << state.roll * RAD_TO_DEG << "deg | "
         << "Thr: " << cmd.throttle_cmd << "% | "
         << endl;
}

int main() {
    cout << "=== A320 Autopilot Test Driver Demo ===" << endl << endl;

    test_driver driver;
    driver.init();

    AircraftState state = {};
    state.alt_msl = 1000.0f;
    state.alt_std = 1000.0f;
    state.hagl = 1000.0f;
    state.tas = 100.0f;
    state.cas = 100.0f;
    state.groundspeed = 100.0f;
    state.mach = 0.3f;
    state.pitch = 5.0f * DEG_TO_RAD;
    state.roll = 0.0f;
    state.yaw = 45.0f * DEG_TO_RAD;
    state.n1_left = 50.0f;
    state.n1_right = 50.0f;
    state.aoa = 5.0f * DEG_TO_RAD;
    state.alpha_prot = 15.0f * DEG_TO_RAD;

    float dt = 0.05f;
    float sim_time = 0.0f;

    cout << "Phase 1: 启用自动驾驶和自动油门" << endl;
    driver.execute_command("AP_ON");
    driver.execute_command("AT_ON");
    cout << "AP enabled: " << boolalpha << driver.is_autopilot_enabled() << endl;
    cout << "AT enabled: " << boolalpha << driver.is_autothrottle_enabled() << endl;

    cout << "\nPhase 2: 设置航向保持，目标航向 90°" << endl;
    driver.execute_command("SET_HDG", 90.0f);
    driver.execute_command("HDG_HOLD");

    for (int i = 0; i < 50; i++) {
        driver.map_in(state);
        driver.step(dt);

        ControlCommand cmd;
        driver.map_out(cmd);

        state.roll_rate = (cmd.roll_cmd - state.roll) * 3.0f;
        state.roll += state.roll_rate * dt;

        sim_time += dt;

        if (i % 10 == 0) {
            cout << "t=" << fixed << setprecision(1) << sim_time << "s  "
                 << "HDG=" << state.yaw * RAD_TO_DEG << "deg  "
                 << "Roll=" << state.roll * RAD_TO_DEG << "deg  "
                 << endl;
        }

        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }

    cout << "\nPhase 3: 设置高度保持，目标高度 5000ft" << endl;
    driver.execute_command("SET_ALT", 5000.0f);
    driver.execute_command("ALT_HOLD");

    for (int i = 0; i < 100; i++) {
        driver.map_in(state);
        driver.step(dt);

        ControlCommand cmd;
        driver.map_out(cmd);

        state.pitch_rate = (cmd.pitch_cmd - state.pitch) * 2.0f;
        state.pitch += state.pitch_rate * dt;
        state.thrust_left = cmd.throttle_cmd * 10.0f;
        state.tas += (cmd.throttle_cmd - 50.0f) * 0.02f;
        float vz = sin(state.pitch) * state.tas;
        state.alt_msl += vz * dt;
        state.alt_std = state.alt_msl;
        state.hagl = state.alt_msl;

        sim_time += dt;

        if (i % 20 == 0) {
            cout << "t=" << fixed << setprecision(1) << sim_time << "s  "
                 << "ALT=" << state.alt_msl / 0.3048f << "ft  "
                 << "Pitch=" << state.pitch * RAD_TO_DEG << "deg  "
                 << "IAS=" << state.cas * MS_TO_KNOTS << "kt  "
                 << "Thr=" << cmd.throttle_cmd << "%  "
                 << endl;
        }

        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }

    cout << "\nPhase 4: 模拟复飞" << endl;
    state.alt_msl = 500.0f;
    state.hagl = 500.0f;
    state.pitch = 2.0f * DEG_TO_RAD;
    driver.execute_command("GO_AROUND");

    for (int i = 0; i < 50; i++) {
        driver.map_in(state);
        driver.step(dt);

        ControlCommand cmd;
        driver.map_out(cmd);

        state.pitch_rate = (cmd.pitch_cmd - state.pitch) * 2.0f;
        state.pitch += state.pitch_rate * dt;
        state.thrust_left = cmd.throttle_cmd * 10.0f;
        state.tas += (cmd.throttle_cmd - 50.0f) * 0.03f;
        float vz = sin(state.pitch) * state.tas;
        state.alt_msl += vz * dt;
        state.hagl = state.alt_msl;

        sim_time += dt;

        if (i % 10 == 0) {
            cout << "t=" << fixed << setprecision(1) << sim_time << "s  "
                 << "ALT=" << state.alt_msl / 0.3048f << "ft  "
                 << "Pitch=" << state.pitch * RAD_TO_DEG << "deg  "
                 << "Thr=" << cmd.throttle_cmd << "%  "
                 << endl;
        }

        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }

    cout << "\n=== Demo Complete ===" << endl;
    return 0;
}