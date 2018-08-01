// Conversion & writing of voxels to real-space centers (header)

#ifndef vox_to_cents_hh
#define vox_to_cents_hh

#include <fstream>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

// Sparse hash map
#include <sparsepp/spp.h>

#include "ind.hh"
#include "pt.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"

class out_cents
{
  public:
    out_cents(spp::sparse_hash_map<ind, occ_data> & occ,
              spp::sparse_hash_map<ind, free_unk_data> & freev,
              spp::sparse_hash_map<ind, free_unk_data> & unk,
              float resolution
              )
    {
      std::cout << "free voxels: " << freev.size() << '\n';
      std::cout << "occupied voxels: " << occ.size() << '\n';
      std::cout << "unknown voxels: " << unk.size() << '\n';

      std::vector<pt> cents_occ;
      std::vector<pt> cents_free;
      std::vector<pt> cents_unknown;

      get_vox_cents(occ,
                    cents_occ,
                    resolution
                    );

      get_vox_cents(freev,
                    cents_free,
                    resolution
                    );

      get_vox_cents(unk,
                    cents_unknown,
                    resolution
                    );

      write_cents(cents_occ,
                  "occupied"
                  );

      write_cents(cents_free,
                  "free"
                  );

      write_cents(cents_unknown,
                  "unknown"
                  );

    }

    // Overload for occupied voxels
    void get_vox_cents(spp::sparse_hash_map <ind, occ_data> & vox,
                       std::vector<pt> & cents,
                       float resolution
                       )
    {
      // Reserve for centers
      cents.reserve(vox.size());
      // Construct map iterator
      spp::sparse_hash_map <ind, occ_data> ::iterator vox_iterator;
      for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
      {
        if(vox_iterator->second.mask)
        {
          float half = 0.5f * vox_iterator->second.sr_extent;
          cents.push_back({(vox_iterator->first.x + half) * resolution, (vox_iterator->first.y + half) * resolution, (vox_iterator->first.z + half) * resolution});
        }
      }
    }

    // Overload for free/unk voxels
    void get_vox_cents(spp::sparse_hash_map <ind, free_unk_data> & vox,
                      std::vector<pt> & cents,
                      float resolution
                      )
    {
      // Reserve for centers
      cents.reserve(vox.size());
      // Construct map iterator
      spp::sparse_hash_map <ind, free_unk_data> ::iterator vox_iterator;
      for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
      {
        float half = 0.5f * vox_iterator->second.sr_extent;
        cents.push_back({(vox_iterator->first.x + half) * resolution, (vox_iterator->first.y + half) * resolution, (vox_iterator->first.z + half) * resolution});
      }
    }

    // Write cents to file
    void write_cents(std::vector<pt> & cents,
                     std::string filename
                     )
    {
      std::ofstream file;
      file.open(filename+".txt");
      for(int i = 0; i < cents.size(); ++i)
      {
        file << cents[i].x << ", " << cents[i].y << ", " << cents[i].z << "\n";
      }
      file.close();
    }

};

#endif
