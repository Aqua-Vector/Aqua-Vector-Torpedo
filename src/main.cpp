#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include "core/TorpedoControlSystem.hpp"
#include "core/NetworkManager.hpp"
#include "communication/UartLink.hpp"
#include "protocol/ControlStationParser.hpp"
#include "control/STMControlParser.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "control/ModeMux.hpp"
#include "control/ControlHandlers.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/Marshaller.hpp"

#include "guidance/EbImuUart.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"

// Global pointer for signal handling to allow graceful shutdown
static TorpedoControlSystem* g_tcs = nullptr;

/**
 * @brief Signal handler for SIGINT (Ctrl+C) and SIGTERM
 * Ensures actuators are stopped and threads are joined safely.
 */
void signalHandler(int signum) {
    std::cout << "\n[Main] Signal (" << signum << ") received. Stopping system..." << std::endl;
    if (g_tcs) {
        g_tcs->stop();
    }
}

int main() {
    std::cout << "================================================" << std::endl;
    std::cout << "      Aqua Vector Torpedo Control System        " << std::endl;
    std::cout << "   (Architecture: GCS Mid-course & Terminal)    " << std::endl;
    std::cout << "================================================" << std::endl;

    // 1. Hardware & Communication Link Setup
    // Zynq-STM32: UART PS1, 230400 baud
    UartLink stm32_link("/dev/ttyPS1", 230400);
    // Zynq-GCS/Radio: UART S2, 115200 baud (GCS Standard)
    UartLink gcs_link("/dev/ttyS2", 115200);

    // 2. Protocol Parser & TX Queue Preparation
    // [Update] Use ControlStationParser for GCS
    ControlStationParser gcs_parser;
    STMControlParser stm32_parser;

    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_queue;
    StaticRingBuffer<STMPacket, 64> stm32_tx_queue;

    // 3. Logic Component Assembly
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mode_mux(manual_source.getMailbox(), auto_source.getMailbox());

    // Sensors & Estimators
    // EBIMU on /dev/ttyUSB0, 921600 baud (Match successful HW tests)
    torpedo::sensor::EbImuUart imu("/dev/ttyUSB0", 921600);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;

    // 4. Communication Manager Setup
    NetworkManager<ControlStationParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_manager(gcs_link, gcs_parser, gcs_tx_queue);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm32_manager(stm32_link, stm32_parser, stm32_tx_queue);

    // 5. Torpedo Control System (TCS) Creation
    TorpedoControlSystem tcs(mode_mux, manual_source, auto_source, imu, eskf, rps_tracker, gcs_manager, stm32_manager);
    g_tcs = &tcs;

    // 6. Callback Registration
    // [Update] Use ControlStationHandler for GCS (MSG_ID 0x00 fallback)
    ControlStationHandler gcs_handler(tcs, manual_source, 180.0f); // Default speed 180
    gcs_parser.registerHandler(0x00, &gcs_handler);

    Stm32FeedbackHandler stm32_handler(tcs);
    stm32_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm32_handler);

    // 7. Signal Handling
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 8. System Initialization & Startup
    std::cout << "[Main] Initializing system components..." << std::endl;
    if (!tcs.init()) {
        std::cerr << "[Main] CRITICAL: System initialization failed!" << std::endl;
        return -1;
    }

    std::cout << "[Main] Starting control loops and communication..." << std::endl;
    if (!tcs.start()) {
        std::cerr << "[Main] CRITICAL: System startup failed!" << std::endl;
        return -1;
    }

    std::cout << "[Main] System is RUNNING. Waiting for Launch Command (GCS Flag)." << std::endl;

    // 9. Main Thread Wait Loop
    while (tcs.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[Main] Exiting." << std::endl;
    return 0;
}
