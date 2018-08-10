// Conversion & writing of voxels to real-space centers (header)

#ifndef vox_to_cents_hh
#define vox_to_cents_hh

#include <fstream>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include <sparsepp/spp.h>

#include "ind.hh"
#include "pt.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"

class out_cents
{
  public:
    out_cents(spp::sparse_hash_map<pt, occ_data> & occ,
              spp::sparse_hash_map<pt, int> & unk
              )
    {
      std::cout << "occupied voxels: " << occ.size() << '\n';

      write_cents(occ,
                  "occupied_octo"
                  );

      write_cents(unk,
                  "unknown_octo"
                  );

    }

    // Write cents to file
    void write_cents(spp::sparse_hash_map<pt, occ_data> & cents,
                     std::string filename
                     )
    {
      std::ofstream file;
      file.open(filename+".txt");
      spp::sparse_hash_map<pt, occ_data> ::iterator it;
      for(it = cents.begin(); it != cents.end(); ++it)
      {
        file << it->first.x << ", "
             << it->first.y << ", "
             << it->first.z << ", "
             << it->second.probability << ", "
             << it->second.sr_extent << "\n";
      }
      file.close();
    }

    void write_cents(spp::sparse_hash_map<pt, int> & cents,
                     std::string filename
                     )
    {
      std::ofstream file;
      file.open(filename+".txt");
      spp::sparse_hash_map<pt, int> ::iterator it;
      for(it = cents.begin(); it != cents.end(); ++it)
      {
        file << it->first.x << ", "
             << it->first.y << ", "
             << it->first.z << "\n";
      }
      file.close();
    }

};

#endif
