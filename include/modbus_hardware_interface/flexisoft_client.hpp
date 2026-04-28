/*
 * Copyright (C) NODE Robotics GmbH - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Threaded FlexisoftClient providing cached raw register access specialized for Flexisoft usage.
 * Only read_raw_register and write_raw_register are implemented for consumption by
 * FlexisoftHardwareInterface. All other functionality of a generic Modbus client is intentionally
 * omitted to keep this component minimal.
 */

#pragma once

#include "modbus_hardware_interface/modbus_client.hpp"
#include "modbus_hardware_interface/modbus_exceptions.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
// Local default conversion function (mirrors CONVERSION_FN_DEFAULT in modbus_hardware_interface.hpp)
#include <thread>
#include <utility>
#include <unordered_map>

namespace modbus_hardware_interface
{

static constexpr char FLEXISOFT_DEFAULT_CONVERSION_FN[] = "float_dcba";

// A lightweight wrapper holding the raw read/write configs (address + count) as a key
struct FlexisoftRawKey
{
  int start_address{ 0 };
  int bit_count{ 0 };  // stored as bits; number_of_registers = bit_count / 16 when in register mode

  bool operator==(const FlexisoftRawKey& other) const noexcept
  {
    return start_address == other.start_address && bit_count == other.bit_count;
  }
};

struct FlexisoftRawKeyHasher
{
  std::size_t operator()(const FlexisoftRawKey& k) const noexcept
  {
    // Simple combination hash
    return static_cast<std::size_t>(k.start_address) ^ (static_cast<std::size_t>(k.bit_count) << 16);
  }
};

class FlexisoftClient
{
public:
  explicit FlexisoftClient(std::shared_ptr<ModbusClient> modbus_client,
                           double max_frequency_hz = 10.0)  // default 10 Hz
    : modbus_client_(modbus_client)
  {
    if (max_frequency_hz <= 0.0)
    {
      max_frequency_hz = 10.0;  // fallback
    }
    cycle_period_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / max_frequency_hz));
    if (cycle_period_.count() == 0)
    {
      cycle_period_ = std::chrono::milliseconds(10);  // clamp to 10 ms min
    }
  }

  ~FlexisoftClient()
  {
    shutdown();
  }

  // Non-copyable
  FlexisoftClient(const FlexisoftClient&) = delete;
  FlexisoftClient& operator=(const FlexisoftClient&) = delete;
  // Movable
  FlexisoftClient(FlexisoftClient&&) = delete;
  FlexisoftClient& operator=(FlexisoftClient&&) = delete;

  bool is_persistent_connection() const
  {
    return modbus_client_->is_persistent_connection();
  }
  bool connected() const
  {
    return modbus_client_->connected();
  }
  bool connect()
  {
    return modbus_client_->connect();
  }
  void disconnect()
  {
    modbus_client_->disconnect();
  }

  // Thread-safe API mirroring the subset used by FlexisoftHardwareInterface
  std::vector<uint16_t> read_raw_register(ModbusInterfaceReadConfig& config)
  {
    FlexisoftRawKey key{ config.get_register_address(), config.number_of_bits() };
    std::lock_guard<std::mutex> lk(cache_mutex_);
    auto it = read_cache_.find(key);
    if (it != read_cache_.end())
    {
      return it->second;  // return cached snapshot
    }
    // If we have no cached data yet, perform a synchronous read (first access path)
    try
    {
      auto data = modbus_client_->read_raw_register(config);
      read_cache_[key] = data;
      return data;
    }
    catch (...)
    {
      throw;  // propagate ModbusReadException or other exceptions
    }
  }

  void write_raw_register(ModbusInterfaceWriteConfig& config, const std::vector<uint16_t>& values)
  {
    FlexisoftRawKey key{ config.get_register_address(), config.number_of_bits() };
    {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      pending_write_cache_[key] = values;  // overwrite or insert
    }
    // Notify worker thread a write is pending
    cv_.notify_one();
  }

  void start()
  {
    if (!running_.load())
    {
      running_.store(true);
      worker_thread_ = std::thread([this]() { this->worker_loop(); });
    }
  }

  void shutdown()
  {
    if (running_.exchange(false))
    {
      cv_.notify_one();
      if (worker_thread_.joinable())
      {
        worker_thread_.join();
      }
    }
  }

private:
  void worker_loop()
  {
    auto next_cycle_time = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lk(loop_mutex_);
    while (running_.load())
    {
      cv_.wait_until(lk, next_cycle_time, [this]() { return !running_.load() || !pending_write_cache_.empty(); });
      if (!running_.load())
      {
        break;
      }

      auto now = std::chrono::steady_clock::now();
      if (now < next_cycle_time)
      {
        // Woken by write request but not yet time for next cycle -> continue waiting
        continue;
      }

      next_cycle_time = now + cycle_period_;

      // Ensure connection for this cycle if non-persistent
      if (!modbus_client_->is_persistent_connection())
      {
        modbus_client_->connect();
      }

      do_reads();
      do_writes();

      if (!modbus_client_->is_persistent_connection())
      {
        modbus_client_->disconnect();
      }
    }
  }

  void do_reads()
  {
    // Copy keys under lock to avoid long lock times during modbus operations
    std::vector<FlexisoftRawKey> keys;
    {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      keys.reserve(read_cache_.size());
      for (const auto& kv : read_cache_)
      {
        keys.push_back(kv.first);
      }
    }
    for (auto& k : keys)
    {
      // Construct a minimal ModbusInterfaceReadConfig to reuse underlying API.
      // Since FlexisoftHardwareInterface will prime cache via first synchronous read, we can skip if empty.
      try
      {
        // NOTE: We can't reconstruct full config without original read_function & conversion; using REGISTER.
        ModbusInterfaceReadConfig cfg(k.start_address, k.bit_count, REGISTER, FLEXISOFT_DEFAULT_CONVERSION_FN);
        auto data = modbus_client_->read_raw_register(cfg);
        std::lock_guard<std::mutex> lk(cache_mutex_);
        read_cache_[k] = std::move(data);
      }
      catch (const ModbusReadException& e)
      {
        RCLCPP_WARN(rclcpp::get_logger("FlexisoftClient"), "Background read failed: %s", e.what());
      }
    }
  }

  void do_writes()
  {
    // Move pending writes out under lock
    std::unordered_map<FlexisoftRawKey, std::vector<uint16_t>, FlexisoftRawKeyHasher> writes;
    {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      writes.swap(pending_write_cache_);
    }
    for (auto& kv : writes)
    {
      const auto& key = kv.first;
      auto& values = kv.second;
      try
      {
        // Reconstruct minimal write config (REGISTER + raw), no conversion
        ModbusInterfaceWriteConfig cfg(key.start_address, key.bit_count, REGISTER, FLEXISOFT_DEFAULT_CONVERSION_FN);
        modbus_client_->write_raw_register(cfg, values);
      }
      catch (const ModbusWriteException& e)
      {
        RCLCPP_WARN(rclcpp::get_logger("FlexisoftClient"), "Background write failed: %s", e.what());
      }
    }
  }

  std::shared_ptr<ModbusClient> modbus_client_;
  std::chrono::milliseconds cycle_period_{ 100 };  // default ~10Hz
  std::atomic<bool> running_{ false };
  std::thread worker_thread_;

  // Synchronization
  std::mutex loop_mutex_;
  std::condition_variable cv_;

  std::mutex cache_mutex_;
  std::unordered_map<FlexisoftRawKey, std::vector<uint16_t>, FlexisoftRawKeyHasher> read_cache_;
  std::unordered_map<FlexisoftRawKey, std::vector<uint16_t>, FlexisoftRawKeyHasher> pending_write_cache_;
};

}  // namespace modbus_hardware_interface
