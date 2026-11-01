#include <algorithm>
#include <string>
#include <vector>

#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

// Decodes each address with the bit layout of the memory region it falls in, so
// different regions of memory can use different address mappings.
//
// The paper selects a mapping with a MapID stored in the MDT (page 8). A trace
// only holds addresses, so an address range stands in for the MapID here.
//
// Example config, with one list of address bits per DRAM level, low to high:
//
//   AddrMapper:
//     impl: RegionMapper
//     page_bytes: 2097152
//     regions:
//       - name: npu_friendly
//         base: 0
//         size: 1073741824
//         bits: {channel: [5, 6, 7, 8], pseudochannel: [9], bankgroup: [10, 11],
//                bank: [12, 13], row: [19, 20], column: [14, 15, 16, 17, 18]}
class RegionMapper final : public IAddrMapper, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RegionMapper, "RegionMapper", "Applies a separate address mapping to each memory region.");

  private:
    struct Region {
      std::string name;
      Addr_t base = 0;
      Addr_t size = 0;
      // level_bits[level] lists that level's address bits, low to high.
      std::vector<std::vector<int>> level_bits;
    };

    IDRAM* m_dram = nullptr;
    std::vector<Region> m_regions;

    Addr_t m_page_bytes = -1;
    int m_page_bits = -1;           // Bits inside one page, which the layouts cover
    int m_num_levels = -1;
    int m_row_level = -1;
    int m_tx_offset = -1;           // Bits inside one request, which select no DRAM level
    std::vector<int> m_addr_bits;   // Most address bits each level can use

  public:
    void init() override {
      m_page_bytes = param<Addr_t>("page_bytes").desc("Bytes covered by one layout.").required();
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();

      // A size must be a power of two for a whole number of bits to select in it.
      if (m_page_bytes <= 0 || (m_page_bytes & (m_page_bytes - 1)) != 0) {
        throw ConfigurationError("RegionMapper: page_bytes {} is not a power of two.", m_page_bytes);
      }
      m_page_bits = calc_log2(m_page_bytes);

      // Same calculation as the linear mappers: n places need log2(n) bits, and
      // one column address in the request covers a whole prefetch.
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (int level = 0; level < m_num_levels; level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);
      m_row_level = m_dram->m_levels("row");

      if (!m_config["regions"] || m_config["regions"].size() == 0) {
        throw ConfigurationError("RegionMapper: no regions given.");
      }
      for (const auto& node : m_config["regions"]) {
        m_regions.push_back(read_region(node));
        check_region(m_regions.back());
      }
      check_no_overlap();
    };

    void apply(Request& req) override {
      const Region& region = find_region(req.addr);
      req.addr_vec.resize(m_num_levels, 0);

      // Collect each level's bits into one number. For example, row bits [19, 20]
      // make bit 19 of the address bit 0 of the row, and bit 20 bit 1 of the row.
      for (int level = 0; level < m_num_levels; level++) {
        int value = 0;
        const auto& bits = region.level_bits[level];
        for (size_t i = 0; i < bits.size(); i++) {
          value |= ((req.addr >> bits[i]) & 1) << i;
        }
        req.addr_vec[level] = value;
      }

      // Every page of a region shares one layout, so the page number goes above
      // the layout's row bits to give each page its own rows.
      Addr_t page = req.addr >> m_page_bits;
      req.addr_vec[m_row_level] |= page << region.level_bits[m_row_level].size();
    };

  private:
    Region read_region(const YAML::Node& node) const {
      Region region;
      region.name = node["name"].as<std::string>();
      region.base = node["base"].as<Addr_t>();
      region.size = node["size"].as<Addr_t>();

      // Look levels up by the names the DRAM defines, so this suits any DRAM type.
      region.level_bits.resize(m_num_levels);
      for (int level = 0; level < m_num_levels; level++) {
        std::string level_name(m_dram->m_levels(level));
        if (!node["bits"][level_name]) {
          throw ConfigurationError("RegionMapper: region {} has no bits for {}.", region.name, level_name);
        }
        region.level_bits[level] = node["bits"][level_name].as<std::vector<int>>();
      }
      return region;
    };

    // Stops a wrong config at startup, before it can silently corrupt results.
    void check_region(const Region& region) const {
      // A layout only makes sense over whole pages.
      if (region.size <= 0 || region.base % m_page_bytes != 0 || region.size % m_page_bytes != 0) {
        throw ConfigurationError("RegionMapper: region {} is not a whole number of pages.", region.name);
      }

      // Each level stays within its capacity (paper page 6).
      std::vector<int> used;
      for (int level = 0; level < m_num_levels; level++) {
        const auto& bits = region.level_bits[level];
        if ((int)bits.size() > m_addr_bits[level]) {
          throw ConfigurationError("RegionMapper: region {} has too many bits for {}.", region.name, m_dram->m_levels(level));
        }
        used.insert(used.end(), bits.begin(), bits.end());
      }

      // Each bit inside a page, above the request offset, goes to exactly one level
      // (paper page 6). Sorted, they must count up from the offset with no gaps.
      std::sort(used.begin(), used.end());
      if ((int)used.size() != m_page_bits - m_tx_offset) {
        throw ConfigurationError("RegionMapper: region {} must use each of bits {} to {} once.", region.name, m_tx_offset, m_page_bits - 1);
      }
      for (size_t i = 0; i < used.size(); i++) {
        if (used[i] != m_tx_offset + (int)i) {
          throw ConfigurationError("RegionMapper: region {} must use each of bits {} to {} once.", region.name, m_tx_offset, m_page_bits - 1);
        }
      }

      // The region's last page must still fit in the row level. For example, with
      // 2 layout row bits and 15 row bits in total, 13 bits are left for the page.
      Addr_t last_page = (region.base + region.size - 1) >> m_page_bits;
      int page_bits_left = m_addr_bits[m_row_level] - (int)region.level_bits[m_row_level].size();
      if ((last_page >> page_bits_left) != 0) {
        throw ConfigurationError("RegionMapper: region {} reaches past the last row.", region.name);
      }
    };

    void check_no_overlap() const {
      std::vector<Region> sorted = m_regions;
      std::sort(sorted.begin(), sorted.end(), [](const Region& a, const Region& b) { return a.base < b.base; });
      for (size_t i = 1; i < sorted.size(); i++) {
        if (sorted[i - 1].base + sorted[i - 1].size > sorted[i].base) {
          throw ConfigurationError("RegionMapper: regions {} and {} overlap.", sorted[i - 1].name, sorted[i].name);
        }
      }
    };

    // A short loop is enough, as there are only a few regions.
    const Region& find_region(Addr_t addr) const {
      for (const auto& region : m_regions) {
        if (addr >= region.base && addr < region.base + region.size) {
          return region;
        }
      }
      throw std::runtime_error(fmt::format("RegionMapper: address {:#x} is in no region.", addr));
    };
};

}   // namespace Ramulator