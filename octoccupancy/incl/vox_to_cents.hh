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
    out_cents(std::vector<pt> & occ,
              std::vector<pt> & unk
              )
    {
      std::cout << "occupied voxels: " << occ.size() << '\n';
      std::cout << "unknown voxels: " << unk.size() << '\n';

      write_cents(occ,
                  "occupied"
                  );

      write_cents(unk,
                  "unknown"
                  );

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
