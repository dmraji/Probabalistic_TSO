// Conversion & writing of voxels to real-space centers (header)

#ifndef vox_to_cents_hh
#define vox_to_cents_hh

#include <fstream>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

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

      // spp::sparse_hash_map<ind, int> blacklist;
      // for(auto it = occ.cbegin(); it != occ.cend(); ++it)
      // {
      //   ind cind = it->first;
      //
      //   // If key has been marked for masking, mask it
      //   if(blacklist.count(cind) != 0)
      //   {
      //     // std::cout << "bl" << '\n';
      //     occ[cind].mask = false;
      //     blacklist.erase(cind);
      //     continue;
      //   }
      //
      //   // Otherwise, determine if node is pruneable; if it is, mark nodes for masking;
      //   std::vector<ind> node;
      //   if(cind.pruneable(occ,
      //                     node
      //                     ))
      //   {
      //     // std::cout << "pr?" << '\n';
      //     occ[node[0]].extent *= 2;
      //     // Don't blacklist corner index; it will represent larger vox
      //     for(int i = 1; i < node.size(); ++i) {
      //       // std::cout << "i: " << i << '\n';
      //        ++blacklist[node[i]]; }
      //   }
      //   node.clear();
      // }

      std::cout << "occupied voxels: " << occ.size() << '\n';
      std::cout << "free voxels: " << freev.size() << '\n';
      std::cout << "unknown voxels: " << unk.size() << '\n';

      std::vector<pt_write> cents_occ;
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

    // Get real-space centers of occupied voxels
    void get_vox_cents(spp::sparse_hash_map <ind, occ_data> & vox,
                       std::vector<pt_write> & cents,
                       float resolution
                       )
    {
      int sizeo = 0;
      // Reserve for centers
      cents.reserve(vox.size());
      // Construct map iterator
      spp::sparse_hash_map <ind, occ_data> ::iterator it;
      for(it = vox.begin(); it != vox.end(); ++it)
      {
        // if((it->second.mask))
        if((it->second.mask) || !(it->second.mask))
        {
          ++sizeo;
          cents.push_back( {(it->first.x + (0.5f * it->second.extent)) * resolution,
                            (it->first.y + (0.5f * it->second.extent)) * resolution,
                            (it->first.z + (0.5f * it->second.extent)) * resolution,
                            (it->second.probability),
                            (it->second.intensity)} );
        }
      }
      std::cout << sizeo << '\n';
    }

    // Get real-space centers of free/unk voxels
    void get_vox_cents(spp::sparse_hash_map <ind, free_unk_data> & vox,
                      std::vector<pt> & cents,
                      float resolution
                      )
    {
      // Reserve for centers
      cents.reserve(vox.size());
      // Construct map iterator
      spp::sparse_hash_map <ind, free_unk_data> ::iterator it;
      for(it = vox.begin(); it != vox.end(); ++it)
      {
        cents.push_back( {(it->first.x + 0.5f) * resolution,
                          (it->first.y + 0.5f) * resolution,
                          (it->first.z + 0.5f) * resolution} );
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
        file << cents[i].x << ", "
             << cents[i].y << ", "
             << cents[i].z << "\n";
      }
      file.close();
    }

    void write_cents(std::vector<pt_write> & cents,
                     std::string filename
                     )
    {
      std::ofstream file;
      file.open(filename+".txt");
      for(int i = 0; i < cents.size(); ++i)
      {
        file << cents[i].x << ", "
             << cents[i].y << ", "
             << cents[i].z << ", "
             << cents[i].probability << ", "
             << cents[i].intensity << "\n";
      }
      file.close();
    }

};

#endif
