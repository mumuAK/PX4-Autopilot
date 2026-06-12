#include "a320_autopilot.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace A320;
using namespace std;

// 模拟A320状态更新
void updateAircraftState(AircraftState& state, float dt) {
    // 简化的动力学模拟
    state.roll += state.roll_rate * dt;
    state.pitch += state.pitch_rate * dt;
    state.yaw += state.yaw_rate * dt;
    
    // 速度更新
    float thrust = (state.thrust_left + state.thrust_right) / 2.0f / 100000.0f;
    state.tas += thrust * dt * 10.0f;
    
    // 高度更新（简化）
    float climb_rate = sin(state.pitch) * state.tas;
    state.alt_msl += climb_rate * dt;
    state.alt_std = state.alt_msl;
    
    // 航向保持
    state.yaw = Utils::wrapAngle(state.yaw * RAD_TO_DEG) * DEG_TO_RAD;
}

// 打印状态信息
void printState(const AircraftState& state, const ControlCommand& cmd) {
    cout << fixed << setprecision(2);
    cout << "\n=== A320 Autopilot Demo ===" << endl;
    cout << "Altitude: " << state.alt_msl << " m (" << state.alt_msl / 0.3048 << " ft)" << endl;
    cout << "TAS: " << state.tas << " m/s (" << state.tas * MS_TO_KNOTS << " knots)" << endl;
    cout << "Mach: " << state.mach << endl;
    cout << "Pitch: " << state.pitch * RAD_TO_DEG << " deg" << endl;
    cout << "Roll: " << state.roll * RAD_TO_DEG << " deg" << endl;
    cout << "Yaw: " << state.yaw * RAD_TO_DEG << " deg" << endl;
    cout << "\n--- Control Commands ---" << endl;
    cout << "Throttle: " << cmd.throttle_cmd << " %" << endl;
    cout << "Pitch Cmd: " << cmd.pitch_cmd * RAD_TO_DEG << " deg" << endl;
    cout << "Roll Cmd: " << cmd.roll_cmd * RAD_TO_DEG << " deg" << endl;
}

int main() {
    cout << "A320 Autopilot System Demo" << endl;
    cout << "=========================" << endl;
    
    // 初始化自动飞行系统
    AutoFlightSystem afs;
    afs.initialize();
    
    // 初始化飞行器状态
    AircraftState state = {};
    state.alt_msl = 1000.0f;      // 初始高度 1000m
    state.alt_std = 1000.0f;
    state.tas = 80.0f;             // 初始空速 80m/s
    state.cas = 80.0f;
    state.mach = 0.25f;
    state.pitch = 5.0f * DEG_TO_RAD;
    state.roll = 0.0f;
    state.yaw = 0.0f;
    state.thrust_left = 50000.0f;
    state.thrust_right = 50000.0f;
    state.n1_left = 80.0f;
    state.n1_right = 80.0f;
    state.is_armed = true;
    state.is_in_air = true;
    state.aoa = 5.0f * DEG_TO_RAD;
    state.alpha_prot = 15.0f * DEG_TO_RAD;
    
    // 启用自动驾驶和自动油门
    afs.enableAutopilot();
    afs.enableAutothrottle();
    
    // 设置初始目标
    afs.setAFMode(AFMode::CLIMB);
    afs.setTargetVerticalSpeed(2000.0f); // 2000 fpm 爬升率
    afs.setTargetSpeed(250.0f);          // 250 knots
    
    float dt = 0.05f; // 50Hz 更新频率
    float sim_time = 0.0f;
    
    cout << "\n--- Phase 1: Climb to 10,000 ft ---" << endl;
    
    // 爬升阶段
    while (state.alt_msl < 3000.0f) { // 约10,000 ft
        afs.update(state);
        ControlCommand cmd = afs.getControlCommand();
        
        // 应用控制指令到状态
        state.thrust_left = cmd.throttle_cmd * 1000.0f;
        state.thrust_right = cmd.throttle_cmd * 1000.0f;
        state.n1_left = cmd.throttle_cmd;
        state.n1_right = cmd.throttle_cmd;
        state.pitch_rate = (cmd.pitch_cmd - state.pitch) * 2.0f;
        
        updateAircraftState(state, dt);
        sim_time += dt;
        
        if ((int)(sim_time * 10) % 20 == 0) { // 每2秒打印一次
            printState(state, cmd);
        }
        
        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }
    
    // 切换到高度保持模式
    cout << "\n--- Phase 2: Altitude Hold at 10,000 ft ---" << endl;
    afs.setAFMode(AFMode::ALTITUDE_HOLD);
    afs.setTargetAltitude(10000.0f); // 10,000 ft
    
    // 高度保持阶段
    for (int i = 0; i < 50; i++) {
        afs.update(state);
        ControlCommand cmd = afs.getControlCommand();
        
        state.thrust_left = cmd.throttle_cmd * 800.0f;
        state.thrust_right = cmd.throttle_cmd * 800.0f;
        state.n1_left = cmd.throttle_cmd;
        state.n1_right = cmd.throttle_cmd;
        state.pitch_rate = (cmd.pitch_cmd - state.pitch) * 2.0f;
        
        updateAircraftState(state, dt);
        sim_time += dt;
        
        if (i % 10 == 0) {
            printState(state, cmd);
        }
        
        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }
    
    // 切换到航向保持模式
    cout << "\n--- Phase 3: Heading Hold (Turn to 90 degrees) ---" << endl;
    afs.setAFMode(AFMode::HEADING_HOLD);
    afs.setTargetHeading(90.0f); // 转向90度
    
    for (int i = 0; i < 100; i++) {
        afs.update(state);
        ControlCommand cmd = afs.getControlCommand();
        
        state.thrust_left = cmd.throttle_cmd * 800.0f;
        state.thrust_right = cmd.throttle_cmd * 800.0f;
        state.roll_rate = (cmd.roll_cmd - state.roll) * 3.0f;
        state.pitch_rate = (cmd.pitch_cmd - state.pitch) * 2.0f;
        
        updateAircraftState(state, dt);
        sim_time += dt;
        
        if (i % 10 == 0) {
            printState(state, cmd);
        }
        
        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }
    
    // 模拟复飞（模拟低空进近时触发复飞）
    cout << "\n--- Phase 4: Go Around ---" << endl;
    state.alt_msl = 300.0f;
    state.hagl   = 300.0f;   // 有离地高度时 state-machine 会优先使用 hagl
    state.alt_std = 300.0f;
    state.tas    = 75.0f;
    state.cas    = 75.0f;
    state.pitch  = 0.0f;
    state.roll   = 0.0f;
    afs.activateGoAround(state);  // 新接口：传入当前 state

    for (int i = 0; i < 50; i++) {
        afs.update(state);
        ControlCommand cmd = afs.getControlCommand();

        state.thrust_left = cmd.throttle_cmd * 1000.0f;
        state.thrust_right = cmd.throttle_cmd * 1000.0f;
        state.n1_left = cmd.throttle_cmd;
        state.n1_right = cmd.throttle_cmd;
        state.pitch_rate = (cmd.pitch_cmd - state.pitch) * 2.0f;
        state.roll_rate  = (cmd.roll_cmd  - state.roll)  * 3.0f;

        updateAircraftState(state, dt);
        sim_time += dt;

        if (i % 5 == 0) {
            printState(state, cmd);
            cout << "GA Phase     : " << afs.getGoAroundPhaseName() << endl;
            cout << "GA Target V/S: " << afs.getGoAroundTargetVS() << " fpm" << endl;
            cout << "GA Target Spd: " << afs.getGoAroundTargetSpeed() << " knots" << endl;
            cout << "Gear OK      : " << (afs.getGoAroundController().isGearRetractionAllowed() ? "YES" : "NO") << endl;
            cout << "Flap OK      : " << (afs.getGoAroundController().isFlapRetractionAllowed() ? "YES" : "NO") << endl;
        }

        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }
    
    // 切换到速度保持模式
    cout << "\n--- Phase 5: Speed Hold at 280 knots ---" << endl;
    afs.setAFMode(AFMode::SPEED_HOLD);
    afs.setTargetSpeed(280.0f);
    
    for (int i = 0; i < 50; i++) {
        afs.update(state);
        ControlCommand cmd = afs.getControlCommand();
        
        state.thrust_left = cmd.throttle_cmd * 800.0f;
        state.thrust_right = cmd.throttle_cmd * 800.0f;
        state.n1_left = cmd.throttle_cmd;
        state.n1_right = cmd.throttle_cmd;
        
        updateAircraftState(state, dt);
        sim_time += dt;
        
        if (i % 10 == 0) {
            printState(state, cmd);
        }
        
        this_thread::sleep_for(chrono::milliseconds((int)(dt * 1000)));
    }
    
    cout << "\n=== Simulation Complete ===" << endl;
    cout << "Total simulation time: " << sim_time << " seconds" << endl;
    
    return 0;
}