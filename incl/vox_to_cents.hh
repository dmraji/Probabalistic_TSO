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

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

class out_cents
{
  public:

    // Writing work is done in constructor after instantiation in main
    out_cents(spp::sparse_hash_map<ind, occ_data> & occ,
              spp::sparse_hash_map<ind, free_unk_data> & freev,
              spp::sparse_hash_map<ind, free_unk_data> & unk,
              spp::sparse_hash_map<ind, int> & pocc,
              float resolution
              )
    {

      /* @ Write AMR method, indev (BEGIN)

      spp::sparse_hash_map<ind, int> blacklist;
      for(auto it = occ.cbegin(); it != occ.cend(); ++it)
      {
        ind cind = it->first;

        // If key has been marked for masking, mask it
        if(blacklist.count(cind) != 0)
        {
          // std::cout << "bl" << '\n';
          occ[cind].mask = false;
          blacklist.erase(cind);
          continue;
        }

        // Otherwise, determine if node is pruneable; if it is, mark nodes for masking;
        std::vector<ind> node;
        if(cind.pruneable(occ,
                          node
                          ))
        {
          // std::cout << "pr?" << '\n';
          occ[node[0]].extent *= 2;
          // Don't blacklist corner index; it will represent larger vox
          for(int i = 1; i < node.size(); ++i) {
            // std::cout << "i: " << i << '\n';
             ++blacklist[node[i]]; }
        }
        node.clear();
      }

      @ Write AMR method, indev (END) */

      // std::cout << "occupied voxels: " << occ.size() << '\n';
      // std::cout << "free voxels: " << freev.size() << '\n';
      // std::cout << "unknown voxels: " << unk.size() << '\n';

      // Vectors to hold voxel centers
      std::vector<pt_occ> cents_occ;
      std::vector<pt> cents_free;
      std::vector<pt> cents_unknown;

      // Adjust probabilities for "definitely occupied voxels"
      spp::sparse_hash_map <ind, int> ::iterator it;
      for(it = pocc.begin(); it != pocc.end(); ++it)
      {
        occ[it->first].probability = 1;
      }

      // Calls to members to fetch center coords of voxels given sparse hash data structures
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

      // Calls to members to write center coords to file
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

    //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_/

    // Get real-space centers of occupied voxels
    void get_vox_cents(spp::sparse_hash_map <ind, occ_data> & vox,
                       std::vector<pt_occ> & cents,
                       float resolution
                       )
    {
      int sizeo = 0;

      // Reserve for centers
      cents.reserve(vox.size());

      // Construct map iterator
      spp::sparse_hash_map <ind, occ_data> ::iterator it;

      // Iterate through map and add center data to vector for each voxel
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
      // std::cout << sizeo << '\n';
    }

    //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_/

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

      // Iterate through map and add center data to vector for each voxel
      for(it = vox.begin(); it != vox.end(); ++it)
      {
        cents.push_back( {(it->first.x + 0.5f) * resolution,
                          (it->first.y + 0.5f) * resolution,
                          (it->first.z + 0.5f) * resolution} );
      }
    }

    //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_/

    // Write cents to file (free and unknown)
    void write_cents(std::vector<pt> & cents,
                     std::string filename
                     )
    {
      // Instantiate output file stream
      std::ofstream file;
      file.open(filename+".txt");

      // Iterate through vector and write each center to file
      for(int i = 0; i < cents.size(); ++i)
      {
        file << cents[i].x << ", "
             << cents[i].y << ", "
             << cents[i].z << "\n";
      }
      file.close();
    }

    //_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_/

    // Write cents to file (occupied)
    void write_cents(std::vector<pt_occ> & cents,
                     std::string filename
                     )
    {
      // Instantiate output file stream
      std::ofstream file;
      file.open(filename+".txt");

      // Iterate through vector and write center to file, along with probability and intensity
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
