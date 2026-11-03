#include <filesystem>
#include <iostream>
#include <fstream>

#include <algorithm>

#include "frontend/frontend.h"
#include "base/exception.h"
#include "dram/dram.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

namespace fs = std::filesystem;

class LoadStoreTrace : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, LoadStoreTrace, "LoadStoreTrace", "Load/Store memory address trace.")

  private:
    enum class Op { Load, Store, PimMac };
    struct Trace {
      Op op;
      Addr_t addr;
    };
    std::vector<Trace> m_trace;

    size_t m_trace_length = 0;
    size_t m_curr_trace_idx = 0;

    size_t m_trace_count = 0;

    int m_pim_mac_type = -1;   // Request id of PIM-MAC, which the DRAM defines

    Logger_t m_logger;

  public:
    void init() override {
      std::string trace_path_str = param<std::string>("path").desc("Path to the load store trace file.").required();
      m_clock_ratio = param<uint>("clock_ratio").required();

      m_logger = Logging::create_logger("LoadStoreTrace");
      m_logger->info("Loading trace file {} ...", trace_path_str);
      init_trace(trace_path_str);
      m_logger->info("Loaded {} lines.", m_trace.size());
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      bool has_pim = std::any_of(m_trace.begin(), m_trace.end(), [](const Trace& t) { return t.op == Op::PimMac; });
      if (has_pim) {
        IDRAM* dram = memory_system->get_ifce<IDRAM>();
        if (!dram->m_requests.contains("pim-mac")) {
          throw ConfigurationError("The trace has PIM lines, but the DRAM has no pim-mac request.");
        }
        m_pim_mac_type = dram->m_requests("pim-mac");
      }
    };


    void tick() override {
      const Trace& t = m_trace[m_curr_trace_idx];
      int type = Request::Type::Read;
      if (t.op == Op::Store) {
        type = Request::Type::Write;
      } else if (t.op == Op::PimMac) {
        type = m_pim_mac_type;
      }
      bool request_sent = m_memory_system->send({t.addr, type});
      if (request_sent) {
        m_curr_trace_idx = (m_curr_trace_idx + 1) % m_trace_length;
        m_trace_count++;
      }
    };


  private:
    void init_trace(const std::string& file_path_str) {
      fs::path trace_path(file_path_str);
      if (!fs::exists(trace_path)) {
        throw ConfigurationError("Trace {} does not exist!", file_path_str);
      }

      std::ifstream trace_file(trace_path);
      if (!trace_file.is_open()) {
        throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
      }

      std::string line;
      while (std::getline(trace_file, line)) {
        std::vector<std::string> tokens;
        tokenize(tokens, line, " ");

        // TODO: Add line number here for better error messages
        if (tokens.size() != 2) {
          throw ConfigurationError("Trace {} format invalid!", file_path_str);
        }

        Op op = Op::Load;
        if (tokens[0] == "LD") {
          op = Op::Load;
        } else if (tokens[0] == "ST") {
          op = Op::Store;
        } else if (tokens[0] == "PIM") {
          op = Op::PimMac;
        } else {
          throw ConfigurationError("Trace {} format invalid!", file_path_str);
        }

        Addr_t addr = -1;
        if (tokens[1].compare(0, 2, "0x") == 0 | tokens[1].compare(0, 2, "0X") == 0) {
          addr = std::stoll(tokens[1].substr(2), nullptr, 16);
        } else {
          addr = std::stoll(tokens[1]);
        }
        m_trace.push_back({op, addr});
      }

      trace_file.close();

      m_trace_length = m_trace.size();
    };

    // TODO: FIXME
    bool is_finished() override {
      return m_trace_count >= m_trace_length; 
    };
};

}        // namespace Ramulator