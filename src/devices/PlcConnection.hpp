// SPDX-License-Identifier: MIT
// PlcConnection.hpp — Header-only wrapper for Beckhoff ADS communication
//
// Usage:
//   PlcConnection plc("192.168.88.21", {5,168,39,125,1,1}, {1,2,3,4,5,6});
//   double v = plc.read<double>("MAIN.fVoltage_Ch1");
//   plc.write("MAIN.fMultiplier_Ch1", 2.5);

#pragma once

#include <AdsLib/AdsLib.h>
#include <AdsLib/AdsVariable.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

/// RAII wrapper around a Beckhoff ADS connection.
/// Connects on construction, disconnects on destruction.
class PlcConnection {
 public:
  struct Config {
    std::string remoteIp;
    AmsNetId remoteNetId;
    AmsNetId localNetId;
    uint16_t adsPort = AMSPORT_R0_PLC_TC3;
    uint32_t timeoutMs = 5000;
  };

  explicit PlcConnection(const Config &cfg) : m_config(cfg) { connect(); }

  PlcConnection(const std::string &remoteIp, AmsNetId remoteId, AmsNetId localId, uint16_t port = AMSPORT_R0_PLC_TC3,
                uint32_t timeoutMs = 5000) {
    m_config = {remoteIp, remoteId, localId, port, timeoutMs};
    connect();
  }

  ~PlcConnection() = default;

  PlcConnection(const PlcConnection &) = delete;
  PlcConnection &operator=(const PlcConnection &) = delete;
  PlcConnection(PlcConnection &&) = default;
  PlcConnection &operator=(PlcConnection &&) = default;

  // ── Typed read / write by symbol name ───────────────────────────────

  template <typename T>
  T read(const std::string &symbol) {
    ensureConnected();
    uint32_t handle = cachedHandle(symbol);
    T buffer;
    uint32_t bytesRead = 0;
    auto error = m_device->ReadReqEx2(ADSIGRP_SYM_VALBYHND, handle, sizeof(T), &buffer, &bytesRead);
    // error handling with printing
    if (error) {
      std::cerr << "Error reading '" << symbol << "': " << error << "\n";
      throw std::runtime_error("Failed to read '" + symbol + "'");
    }
    if (bytesRead != sizeof(T)) {
      std::cerr << "Partial read for '" << symbol << "': expected " << sizeof(T) << " bytes, got " << bytesRead
                << " bytes\n";
      throw std::runtime_error("Failed to read '" + symbol + "' (partial read)");
    }
    return buffer;
  }

  template <typename T>
  void write(const std::string &symbol, const T &value) {
    ensureConnected();
    uint32_t handle = cachedHandle(symbol);
    auto error = m_device->WriteReqEx(ADSIGRP_SYM_VALBYHND, handle, sizeof(T), &value);
    if (error) throw std::runtime_error("Failed to write '" + symbol + "'");
  }

  template <typename T, size_t N>
  std::array<T, N> readArray(const std::string &symbol) {
    ensureConnected();
    uint32_t handle = cachedHandle(symbol);
    std::array<T, N> buffer;
    uint32_t bytesRead = 0;
    auto error = m_device->ReadReqEx2(ADSIGRP_SYM_VALBYHND, handle, sizeof(T) * N, buffer.data(), &bytesRead);
    if (error || bytesRead != sizeof(T) * N) throw std::runtime_error("Failed to read array '" + symbol + "'");
    return buffer;
  }

  template <typename T, size_t N>
  void writeArray(const std::string &symbol, const std::array<T, N> &data) {
    ensureConnected();
    uint32_t handle = cachedHandle(symbol);
    auto error = m_device->WriteReqEx(ADSIGRP_SYM_VALBYHND, handle, sizeof(T) * N, data.data());
    if (error) throw std::runtime_error("Failed to write array '" + symbol + "'");
  }

  // ── Raw read / write by index group + offset ────────────────────────

  template <typename T>
  T readRaw(uint32_t indexGroup, uint32_t indexOffset) {
    ensureConnected();
    T buffer;
    uint32_t bytesRead = 0;
    auto error = m_device->ReadReqEx2(indexGroup, indexOffset, sizeof(T), &buffer, &bytesRead);
    if (error || bytesRead != sizeof(T)) throw std::runtime_error("Failed raw read");
    return buffer;
  }

  template <typename T>
  void writeRaw(uint32_t indexGroup, uint32_t indexOffset, const T &value) {
    ensureConnected();
    auto error = m_device->WriteReqEx(indexGroup, indexOffset, sizeof(T), &value);
    if (error) throw std::runtime_error("Failed raw write");
  }

  // ── Device info / state ─────────────────────────────────────────────

  DeviceInfo deviceInfo() const {
    ensureConnected();
    return m_device->GetDeviceInfo();
  }
  AdsDeviceState state() const {
    ensureConnected();
    return m_device->GetState();
  }
  bool isRunning() const { return state().ads == ADSSTATE_RUN; }

  // ── Connection management ───────────────────────────────────────────

  bool isConnected() const noexcept { return m_device != nullptr; }

  void disconnect() {
    m_handleCache.clear();
    m_device.reset();
  }

  void reconnect() {
    disconnect();
    connect();
  }

  void clearHandleCache() { m_handleCache.clear(); }

  const AdsDevice &device() const {
    ensureConnected();
    return *m_device;
  }
  const Config &config() const noexcept { return m_config; }

 private:
  void connect() {
    static std::once_flag local_addr_once;
    std::call_once(local_addr_once, [&]() { bhf::ads::SetLocalAddress(m_config.localNetId); });
    m_device = std::make_unique<AdsDevice>(m_config.remoteIp, m_config.remoteNetId, m_config.adsPort);
    m_device->SetTimeout(m_config.timeoutMs);
  }

  uint32_t cachedHandle(const std::string &symbol) {
    auto it = m_handleCache.find(symbol);
    if (it != m_handleCache.end()) return *it->second;
    auto handle = m_device->GetHandle(symbol);
    uint32_t value = *handle;
    m_handleCache.emplace(symbol, std::move(handle));
    return value;
  }

  void ensureConnected() const {
    if (!m_device) throw std::runtime_error("Not connected to PLC");
  }

  Config m_config;
  std::unique_ptr<AdsDevice> m_device;
  std::unordered_map<std::string, AdsHandle> m_handleCache;
};
